"""Bridge a JPEG birdseye WebSocket into the VLA camera protocol."""
import argparse
import asyncio
import io
import math
import os
import time

from websockets.sync.client import connect
from websockets.asyncio.server import serve

from .camera import CameraCalibration, CameraDriver, CapturedFrame, YoloWorldDetector
from .hardware import HardwareService
from .protocol import token


class BirdseyeSource:
    """Capture one JPEG and map its pixel coordinates into the navigation frame."""

    def __init__(self, url, map_center, meters_per_pixel, object_height, timeout=10):
        if not url.startswith(("ws://", "wss://")):
            raise ValueError("Birdseye feed URL must use ws:// or wss://")
        if any(not math.isfinite(value) for value in (*map_center, *meters_per_pixel, object_height)):
            raise ValueError("Birdseye calibration values must be finite")
        if not all(meters_per_pixel):
            raise ValueError("meters_per_pixel values must be non-zero")
        self.url = url
        self.map_center = map_center
        self.meters_per_pixel = meters_per_pixel
        self.object_height = object_height
        self.timeout = timeout

    def capture(self):
        from PIL import Image

        with connect(self.url, max_size=4 * 1024 * 1024, open_timeout=self.timeout) as ws:
            while True:
                payload = ws.recv(timeout=self.timeout)
                if isinstance(payload, bytes):
                    image = Image.open(io.BytesIO(payload)).convert("RGB")
                    image.load()
                    return CapturedFrame(image, time.time())

    def point_in_camera(self, frame, pixel_x, pixel_y):
        width, height = frame.image.size
        x = self.map_center[0] + (pixel_x - width / 2) * self.meters_per_pixel[0]
        y = self.map_center[1] + (pixel_y - height / 2) * self.meters_per_pixel[1]
        return x, y, self.object_height

    def close(self):
        pass


async def run(feed_url, host, port, model, frame, map_center, meters_per_pixel, object_height):
    token()
    source = BirdseyeSource(feed_url, map_center, meters_per_pixel, object_height)
    driver = CameraDriver(source, YoloWorldDetector(model), CameraCalibration(frame))
    async with serve(HardwareService(driver).handler, host, port, max_size=65536):
        print(f"BIRDSEYE bridge listening on {host}:{port}", flush=True)
        await asyncio.Future()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--feed-url", default=os.getenv("BIRDSEYE_URL", "ws://10.0.0.112:8765/birdseye"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--model", default="yolov8s-worldv2.pt")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--map-center", type=float, nargs=2, required=True, metavar=("X", "Y"))
    parser.add_argument("--meters-per-pixel", type=float, nargs=2, required=True, metavar=("X", "Y"))
    parser.add_argument("--object-height", type=float, required=True)
    args = parser.parse_args()
    asyncio.run(run(args.feed_url, args.host, args.port, args.model, args.frame,
                     tuple(args.map_center), tuple(args.meters_per_pixel), args.object_height))


if __name__ == "__main__":
    main()