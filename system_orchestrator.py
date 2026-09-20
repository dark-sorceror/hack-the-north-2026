#!/usr/bin/env python3
"""
Complete System Orchestrator for VLA-HTN

This is the MASTER CONTROL script that manages:
1. Camera Service (object detection)
2. Nav2 Bridge (navigation on separate Pi)
3. Arm101 Bridge (manipulation on separate Pi)
4. Coordinator (main task execution)

USAGE:
  python system_orchestrator.py --mode complete --target "bottle"
  python system_orchestrator.py --mode test
  python system_orchestrator.py --mode demo

SIGNAL FLOW:
  User voice: "grab a bottle"
    ↓
  Coordinator.fetch("bottle")
    ↓
  STEP 1: save_start → NAV2_BRIDGE (save current position)
    ↓
  STEP 2: locate → CAMERA_SERVICE (find bottle using YOLO)
    ↓
  STEP 3: approach → NAV2_BRIDGE (navigate to bottle location)
    ↓
  STEP 2b: locate → CAMERA_SERVICE (reobserve after moving)
    ↓
  STEP 5: grasp → ARM101_BRIDGE (execute grasp motion)
    ↓
  STEP 6: verify_grasp → ARM101_BRIDGE (check if holding)
    ↓
  STEP 7: stow → ARM101_BRIDGE (put arm away safely)
    ↓
  STEP 8: return_start → NAV2_BRIDGE (return to starting position)
    ↓
  COMPLETE: Task done, object retrieved
"""

import argparse
import asyncio
import json
import logging
import os
import sys
import time
from pathlib import Path
from typing import Optional, Dict, Any

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(name)s | %(levelname)s | %(message)s"
)
logger = logging.getLogger(__name__)


class SystemConfig:
    """Configuration for all three system components."""
    
    def __init__(self):
        # Camera Service (on this Pi)
        self.camera_url = os.getenv("CAMERA_URL", "ws://127.0.0.1:8767")
        self.camera_host = "127.0.0.1"
        self.camera_port = 8767
        
        # Nav2 Bridge (on Nav2 Pi)
        self.nav2_url = os.getenv("NAV2_BRIDGE_URL", "ws://127.0.0.1:8770")
        self.nav2_host = os.getenv("NAV2_PI_HOST", "127.0.0.1")
        self.nav2_port = 8770
        
        # Arm101 Bridge (on Arm101 Pi)
        self.arm101_url = os.getenv("ARM101_BRIDGE_URL", "ws://127.0.0.1:8771")
        self.arm101_host = os.getenv("ARM101_PI_HOST", "127.0.0.1")
        self.arm101_port = 8771
        
        # Robot URL (where coordinator/hardware execute)
        self.robot_url = os.getenv("ROBOT_URL", "ws://127.0.0.1:8766")
        
        # Security
        self.robot_token = os.getenv("ROBOT_TOKEN", "dev-token-at-least-24-chars-long")
        
        # Timeouts
        self.service_startup_timeout = 10.0
        self.action_timeout = 120.0
        
    def validate(self):
        """Validate configuration."""
        if len(self.robot_token) < 24:
            raise ValueError("ROBOT_TOKEN must be at least 24 characters")
        return True
    
    def to_dict(self):
        """Export configuration as dict."""
        return {
            "camera_url": self.camera_url,
            "nav2_url": self.nav2_url,
            "arm101_url": self.arm101_url,
            "robot_url": self.robot_url,
            "camera_host": self.camera_host,
            "nav2_host": self.nav2_host,
            "arm101_host": self.arm101_host,
        }


class ServiceMonitor:
    """Monitor health of all system services."""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.services = {
            "camera": {"url": config.camera_url, "status": "unknown", "checked_at": None},
            "nav2": {"url": config.nav2_url, "status": "unknown", "checked_at": None},
            "arm101": {"url": config.arm101_url, "status": "unknown", "checked_at": None},
        }
    
    async def check_service(self, name: str, timeout: float = 5.0) -> bool:
        """Check if a service is responding."""
        try:
            from websockets.asyncio.client import connect
            url = self.services[name]["url"]
            async with connect(
                url,
                additional_headers={"Authorization": f"Bearer {self.config.robot_token}"},
                max_size=65536,
                open_timeout=timeout
            ) as ws:
                await ws.send(json.dumps({"action": "ping"}))
                response = await asyncio.wait_for(ws.recv(), timeout)
                data = json.loads(response)
                alive = data.get("ok", False) or data.get("pong", False)
                self.services[name]["status"] = "online" if alive else "offline"
        except Exception as e:
            self.services[name]["status"] = f"error: {str(e)[:50]}"
        
        self.services[name]["checked_at"] = time.time()
        return self.services[name]["status"] == "online"
    
    async def check_all(self) -> Dict[str, bool]:
        """Check all services in parallel."""
        results = await asyncio.gather(
            self.check_service("camera"),
            self.check_service("nav2"),
            self.check_service("arm101"),
            return_exceptions=True
        )
        return {
            "camera": results[0] if isinstance(results[0], bool) else False,
            "nav2": results[1] if isinstance(results[1], bool) else False,
            "arm101": results[2] if isinstance(results[2], bool) else False,
        }
    
    def status_report(self) -> str:
        """Generate human-readable status report."""
        lines = ["SERVICE STATUS REPORT", "=" * 60]
        for name, info in self.services.items():
            lines.append(f"  {name:12} | {info['status']:20} | {info['url']}")
        return "\n".join(lines)


class SystemOrchestrator:
    """Main orchestrator that manages the complete fetch sequence."""
    
    def __init__(self, config: Optional[SystemConfig] = None):
        self.config = config or SystemConfig()
        self.monitor = ServiceMonitor(self.config)
        self.coordinator = None
        self.history = []
    
    async def preflight_check(self) -> bool:
        """Verify all services are online before starting."""
        logger.info("Running preflight checks...")
        
        services_ok = await self.monitor.check_all()
        
        print(self.monitor.status_report())
        print()
        
        offline = [name for name, ok in services_ok.items() if not ok]
        if offline:
            logger.warning(f"Services offline: {', '.join(offline)}")
            if input("\nContinue anyway with simulated hardware? (y/n): ").lower() != "y":
                return False
        
        logger.info("Preflight checks complete")
        return True
    
    async def execute_fetch(self, target: str, dry_run: bool = False) -> Dict[str, Any]:
        """
        Execute complete fetch sequence: locate → navigate → grasp → return.
        
        Args:
            target: Object to fetch (e.g., "bottle")
            dry_run: If True, log actions without executing
        
        Returns:
            Execution result with steps, latencies, and status
        """
        logger.info(f"Starting fetch sequence for: {target}")
        
        try:
            # Import coordinator here to avoid circular imports
            from robot_app.coordinator import Coordinator
            from robot_app.hardware_integrations import BridgeHardware, Nav2Bridge, Arm101Bridge
            
            # Create bridge clients pointing to the separate Pi services
            nav2 = Nav2Bridge(self.config.nav2_url)
            arm101 = Arm101Bridge(self.config.arm101_url)
            hardware = BridgeHardware(nav2=nav2, arm=arm101)
            
            # Create coordinator
            coordinator = Coordinator(
                robot_url=self.config.robot_url,
                camera_url=self.config.camera_url
            )
            
            logger.info("\n" + "=" * 70)
            logger.info("EXECUTING FETCH SEQUENCE")
            logger.info("=" * 70)
            logger.info(f"Target: {target}")
            logger.info(f"Camera Service: {self.config.camera_url}")
            logger.info(f"Nav2 Bridge: {self.config.nav2_url}")
            logger.info(f"Arm101 Bridge: {self.config.arm101_url}")
            logger.info("=" * 70 + "\n")
            
            # Execute the fetch
            if dry_run:
                logger.info("[DRY RUN] Would execute fetch sequence")
                result = {"state": "dry_run", "target": target}
            else:
                result = await coordinator.fetch(target)
            
            # Log result
            logger.info("\n" + "=" * 70)
            logger.info("FETCH SEQUENCE COMPLETE")
            logger.info("=" * 70)
            logger.info(json.dumps(result, indent=2))
            logger.info("=" * 70 + "\n")
            
            self.history.append({
                "target": target,
                "timestamp": time.time(),
                "result": result
            })
            
            return result
        
        except Exception as e:
            logger.error(f"Fetch failed: {e}", exc_info=True)
            return {"state": "failed", "error": str(e)}
    
    async def run_demo(self):
        """Run a demonstration of the complete system."""
        logger.info("\n" + "=" * 70)
        logger.info("SYSTEM DEMONSTRATION")
        logger.info("=" * 70)
        logger.info("This will show the complete signal flow from voice → vision → navigation → arm")
        logger.info("=" * 70 + "\n")
        
        # Demo sequence
        targets = ["bottle", "cup", "book"]
        
        for i, target in enumerate(targets, 1):
            logger.info(f"\nDEMO {i}/{len(targets)}: Fetching '{target}'")
            logger.info("-" * 70)
            
            result = await self.execute_fetch(target, dry_run=True)
            
            await asyncio.sleep(1)
        
        logger.info("\nDemo complete!")
    
    async def run_interactive(self):
        """Run interactive mode for manual testing."""
        logger.info("\n" + "=" * 70)
        logger.info("INTERACTIVE MODE")
        logger.info("=" * 70)
        logger.info("Type object names to fetch, or 'quit' to exit")
        logger.info("=" * 70 + "\n")
        
        while True:
            try:
                target = input("Object to fetch: ").strip()
                
                if target.lower() in {"quit", "exit", "q"}:
                    break
                
                if not target:
                    continue
                
                await self.execute_fetch(target)
            
            except KeyboardInterrupt:
                logger.info("\nInterrupted by user")
                break
            except Exception as e:
                logger.error(f"Error: {e}")
    
    def print_summary(self):
        """Print summary of all executions."""
        if not self.history:
            print("No executions recorded")
            return
        
        print("\n" + "=" * 70)
        print("EXECUTION HISTORY")
        print("=" * 70)
        for i, execution in enumerate(self.history, 1):
            print(f"{i}. Target: {execution['target']}")
            print(f"   State: {execution['result'].get('state')}")
            print(f"   Time: {execution['result'].get('latencies_s', {})}")


async def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="VLA-HTN Complete System Orchestrator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
EXAMPLES:
  # Full system with real hardware
  python system_orchestrator.py --mode complete --target "bottle"
  
  # Run tests only
  python system_orchestrator.py --mode test
  
  # Interactive mode for manual testing
  python system_orchestrator.py --mode interactive
  
  # Show current system status
  python system_orchestrator.py --mode status
        """
    )
    
    parser.add_argument(
        "--mode",
        choices=["complete", "test", "demo", "interactive", "status"],
        default="test",
        help="Execution mode"
    )
    
    parser.add_argument(
        "--target",
        default="bottle",
        help="Object to fetch (for complete mode)"
    )
    
    parser.add_argument(
        "--camera-url",
        help="Override camera service URL"
    )
    
    parser.add_argument(
        "--nav2-url",
        help="Override Nav2 bridge URL"
    )
    
    parser.add_argument(
        "--arm101-url",
        help="Override Arm101 bridge URL"
    )
    
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't actually execute, just show plan"
    )
    
    args = parser.parse_args()
    
    try:
        # Create orchestrator
        config = SystemConfig()
        
        # Override with CLI args
        if args.camera_url:
            config.camera_url = args.camera_url
        if args.nav2_url:
            config.nav2_url = args.nav2_url
        if args.arm101_url:
            config.arm101_url = args.arm101_url
        
        config.validate()
        
        orchestrator = SystemOrchestrator(config)
        
        # Execute based on mode
        if args.mode == "status":
            await orchestrator.preflight_check()
        
        elif args.mode == "test":
            logger.info("Running preflight checks...")
            if await orchestrator.preflight_check():
                logger.info("Tests would run here")
        
        elif args.mode == "demo":
            if await orchestrator.preflight_check():
                await orchestrator.run_demo()
        
        elif args.mode == "interactive":
            if await orchestrator.preflight_check():
                await orchestrator.run_interactive()
        
        elif args.mode == "complete":
            if await orchestrator.preflight_check():
                await orchestrator.execute_fetch(args.target, dry_run=args.dry_run)
        
        # Print summary
        orchestrator.print_summary()
    
    except KeyboardInterrupt:
        logger.info("\nShutdown requested")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
