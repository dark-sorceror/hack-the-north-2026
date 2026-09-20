"""Run on the hardware Pi. Simulator only until a real driver is implemented."""
import argparse
import asyncio
import time

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from .protocol import Pose, Reply, Request, authenticated, token
from .hardware_integrations import Arm101Bridge, BridgeHardware, Nav2Bridge


class SimulatedHardware:
    simulated = True

    def __init__(self):
        self.pose = Pose(frame="map", x=0, y=0)
        self.held = False
        self.stopped = asyncio.Event()
        self.history = []

    async def stop(self):
        self.stopped.set()

    async def execute(self, request):
        self.stopped.clear()
        self.history.append(request.action)
        try:
            await asyncio.wait_for(self.stopped.wait(), timeout=0.05)
            raise RuntimeError("Stopped")
        except TimeoutError:
            pass
        action = request.action
        if action == "save_start":
            return {"pose": self.pose.model_dump()}
        if action == "locate":
            if not request.target:
                raise ValueError("A target description is required")
            return {"target": request.target, "pose": Pose(frame="map", x=1, y=0.5, z=0.7).model_dump(),
                    "observed_at": time.time(), "confidence": 0.99}
        if action in {"approach", "grasp", "return_start"} and request.pose is None:
            raise ValueError("Measured pose is required")
        if action == "approach":
            self.pose = request.pose
        elif action == "grasp":
            self.held = True
        elif action == "verify_grasp":
            return {"held": self.held}
        elif action == "stow" and not self.held:
            raise ValueError("No object held")
        elif action == "return_start":
            self.pose = request.pose
        return {"completed": True}


class HardwareService:
    def __init__(self, driver):
        self.driver = driver
        self.lock = asyncio.Lock()
        # IDs live for this process lifetime; full cache rejects further commands.
        self.seen = set()

    async def handler(self, ws):
        if not authenticated(ws):
            await ws.close(1008, "Unauthorized")
            return
        try:
            request = Request.model_validate_json(await asyncio.wait_for(ws.recv(), 5))
        except Exception:
            await ws.close(1008, "Invalid request")
            return
        reply = Reply(id=request.id, ok=False, simulated=self.driver.simulated)
        try:
            if request.expires_at < time.time() or request.expires_at > time.time() + 180:
                raise ValueError("Expired command or invalid deadline")
            if request.id in self.seen:
                raise ValueError("Duplicate command rejected; query robot state before retrying")
            if request.action == "stop":
                await self.driver.stop()
                reply.result = {"stopped": True}
            else:
                if len(self.seen) >= 10000:
                    raise ValueError("Command cache full; restart only while idle")
                if self.lock.locked():
                    raise ValueError("Hardware is busy")
                self.seen.add(request.id)
                async with self.lock:
                    operation = asyncio.create_task(self.driver.execute(request))
                    disconnected = asyncio.create_task(ws.wait_closed())
                    try:
                        done, _ = await asyncio.wait(
                            [operation, disconnected], timeout=max(0, request.expires_at - time.time()),
                            return_when=asyncio.FIRST_COMPLETED)
                        if disconnected in done or operation not in done:
                            await self.driver.stop()
                            raise RuntimeError("Controller disconnected or command expired")
                        reply.result = operation.result()
                    except BaseException:
                        await self.driver.stop()
                        raise
                    finally:
                        operation.cancel()
                        disconnected.cancel()
                        await asyncio.gather(operation, disconnected, return_exceptions=True)
            reply.ok = True
        except Exception as exc:
            reply.error = str(exc)
        try:
            await ws.send(reply.model_dump_json())
        except ConnectionClosed:
            pass


async def run(host, port, driver=None):
    token()
    service = HardwareService(driver or SimulatedHardware())
    async with serve(service.handler, host, port, max_size=65536):
        label = "SIMULATED" if service.driver.simulated else "BRIDGE"
        print(f"{label} hardware listening on {host}:{port}", flush=True)
        await asyncio.Future()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--backend", choices=("simulated", "bridge"), default="simulated")
    parser.add_argument("--nav2-url", default=None)
    parser.add_argument("--arm-url", default=None)
    args = parser.parse_args()
    driver = SimulatedHardware()
    if args.backend == "bridge":
        driver = BridgeHardware(Nav2Bridge(args.nav2_url), Arm101Bridge(args.arm_url))
    asyncio.run(run(args.host, args.port, driver))


if __name__ == "__main__":
    main()
