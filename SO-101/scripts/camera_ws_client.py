#!/usr/bin/env python3
"""
Receive the board's camera stream on the Raspberry Pi.

    pip install websockets opencv-python      # numpy comes with cv2
    python3 camera_ws_client.py --host 10.0.0.112

Prints stats by default. Pass --show to open a window, or import
`frames()` and consume decoded BGR arrays in your own code.

Reconnects on its own: the board may reboot, or the robot may drive out of
wifi range mid-run, and neither should kill the consumer.
"""

import argparse
import asyncio
import time

import cv2
import numpy as np
from websockets.asyncio.client import connect


async def frames(host: str, port: int, retry_delay: float = 2.0):
    """Yield decoded BGR frames, reconnecting forever on failure."""
    url = f"ws://{host}:{port}"
    while True:
        try:
            async with connect(url, max_queue=1) as ws:
                print(f"connected to {url}")
                async for message in ws:
                    buf = np.frombuffer(message, dtype=np.uint8)
                    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if frame is not None:
                        yield frame
        except Exception as exc:  # noqa: BLE001 - any failure means retry
            print(f"disconnected ({type(exc).__name__}: {exc}); retrying in {retry_delay}s")
            await asyncio.sleep(retry_delay)


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True, help="board IP, e.g. 10.0.0.112")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--show", action="store_true", help="display in a window (needs a desktop)")
    args = ap.parse_args()

    n, t0 = 0, time.time()
    async for frame in frames(args.host, args.port):
        n += 1
        if args.show:
            cv2.imshow("board camera", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        if n % 30 == 0:
            now = time.time()
            print(f"{frame.shape[1]}x{frame.shape[0]}  {30 / (now - t0):.1f} fps")
            t0 = now

    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
