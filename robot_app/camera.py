"""Camera service for fresh object observations over the robot protocol."""
import argparse
import asyncio
import time
from dataclasses import dataclass
from typing import Protocol

from websockets.asyncio.server import serve

from .hardware import HardwareService
from .protocol import Pose, Request, token


@dataclass
class CapturedFrame:
    image: object
    observed_at: float
    metadata: object = None


@dataclass
class Detection:
    left: float
    top: float
    right: float
    bottom: float
    confidence: float


class FrameSource(Protocol):
    def capture(self) -> CapturedFrame:
        ...

    def point_in_camera(self, frame: CapturedFrame, pixel_x: float, pixel_y: float) -> tuple[float, float, float]:
        ...

    def close(self) -> None:
        ...


class Detector(Protocol):
    def detect(self, image: object, target: str) -> Detection | None:
        ...


@dataclass
class CameraCalibration:
    frame: str = "map"
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def to_navigation(self, point: tuple[float, float, float]) -> Pose:
        return Pose(frame=self.frame,
                    x=point[0] + self.translation[0],
                    y=point[1] + self.translation[1],
                    z=point[2] + self.translation[2])


class CameraDriver:
    simulated = False

    def __init__(self, source: FrameSource, detector: Detector, calibration=None):
        self.source = source
        self.detector = detector
        self.calibration = calibration or CameraCalibration()

    async def stop(self):
        return None

    async def execute(self, request: Request):
        if request.action != "locate":
            raise ValueError("Camera service only supports locate")
        if not request.target or not request.target.strip():
            raise ValueError("A target description is required")
        frame = await asyncio.to_thread(self.source.capture)
        detection = await asyncio.to_thread(self.detector.detect, frame.image, request.target)
        if detection is None:
            raise ValueError(f"Target not detected: {request.target}")
        if not 0 <= detection.confidence <= 1:
            raise ValueError("Detector confidence must be between 0 and 1")
        center_x = (detection.left + detection.right) / 2
        center_y = (detection.top + detection.bottom) / 2
        camera_point = await asyncio.to_thread(
            self.source.point_in_camera, frame, center_x, center_y)
        pose = self.calibration.to_navigation(camera_point)
        return {"target": request.target, "pose": pose.model_dump(),
                "observed_at": frame.observed_at, "confidence": detection.confidence}


class RealSenseSource:
    def __init__(self, width=640, height=480, fps=30):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError(
                "pyrealsense2 is unavailable for this platform. Install the Intel "
                "RealSense SDK for Raspberry Pi ARM64, or run the camera service "
                "on a host with a supported pyrealsense2 wheel.") from exc

        self.rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

    def capture(self):
        frames = self.align.process(self.pipeline.wait_for_frames())
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        if not depth or not color:
            raise RuntimeError("Camera did not provide synchronized color and depth frames")
        intrinsics = color.profile.as_video_stream_profile().intrinsics
        return CapturedFrame(self._color_image(color), time.time(), (depth, intrinsics))

    @staticmethod
    def _color_image(frame):
        import numpy as np
        return np.asanyarray(frame.get_data())

    def point_in_camera(self, frame, pixel_x, pixel_y):
        depth, intrinsics = frame.metadata
        x = max(0, min(int(pixel_x), intrinsics.width - 1))
        y = max(0, min(int(pixel_y), intrinsics.height - 1))
        distance = depth.get_distance(x, y)
        if distance <= 0:
            raise ValueError("No depth measurement at detected object")
        return tuple(self.rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], distance))

    def close(self):
        self.pipeline.stop()


class YoloWorldDetector:
    def __init__(self, model_name="yolov8s-worldv2.pt", confidence=0.25):
        from ultralytics import YOLOWorld

        self.model = YOLOWorld(model_name)
        self.confidence = confidence

    def detect(self, image, target):
        self.model.set_classes([target])
        results = self.model.predict(image, conf=self.confidence, verbose=False)
        best = None
        for result in results:
            boxes = result.boxes
            for confidence, coordinates in zip(boxes.conf.tolist(), boxes.xyxy.tolist()):
                candidate = Detection(*coordinates, confidence)
                if best is None or candidate.confidence > best.confidence:
                    best = candidate
        return best


async def run(host, port, model, frame, translation):
    source = RealSenseSource()
    driver = CameraDriver(source, YoloWorldDetector(model),
                           CameraCalibration(frame, translation))
    token()
    try:
        async with serve(HardwareService(driver).handler, host, port, max_size=65536):
            print(f"CAMERA service listening on {host}:{port}", flush=True)
            await asyncio.Future()
    finally:
        source.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--model", default="yolov8s-worldv2.pt")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--translation", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                        metavar=("X", "Y", "Z"))
    args = parser.parse_args()
    asyncio.run(run(args.host, args.port, args.model, args.frame, tuple(args.translation)))


if __name__ == "__main__":
    main()