"""Camera, person-tag tracking, and AI trash-can detection."""
import math
import threading
import time
from dataclasses import dataclass

import cv2

import config


def get_camera(source=config.CAMERA_SOURCE):
    """Open a USB webcam (int index) or a phone camera stream (URL string)."""
    camera = cv2.VideoCapture(source)
    if not camera.isOpened():
        raise RuntimeError(f"Could not open camera {source!r}")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always use the newest frame
    return camera


def focal_length_px(frame_width):
    return (frame_width / 2) / math.tan(math.radians(config.CAMERA_HFOV_DEG) / 2)


def bearing_of(x_px, frame_width):
    """Angle to a pixel column in radians. Positive = to the robot's LEFT."""
    return -math.atan((x_px - frame_width / 2) / focal_length_px(frame_width))


@dataclass
class Target:
    """Something seen by the camera: where it is relative to the robot."""
    bearing: float     # radians, + is left
    distance: float    # meters (estimated from apparent size)
    box: tuple         # (x1, y1, x2, y2) pixels, for drawing
    confidence: float = 1.0


# ------------------------------------------------------------ person tag ----
class PersonTracker:
    def __init__(self):
        from pupil_apriltags import Detector
        self.detector = Detector(families=config.TAG_FAMILY, quad_decimate=2.0)

    def find(self, frame):
        """Return a Target for the person's tag, or None if not visible."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for tag in self.detector.detect(gray):
            if tag.tag_id != config.PERSON_TAG_ID:
                continue
            corners = tag.corners
            side_px = sum(
                math.dist(corners[i], corners[(i + 1) % 4]) for i in range(4)
            ) / 4
            w = frame.shape[1]
            distance = focal_length_px(w) * config.TAG_SIZE_M / max(side_px, 1)
            xs, ys = corners[:, 0], corners[:, 1]
            return Target(
                bearing=bearing_of(tag.center[0], w),
                distance=distance,
                box=(int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())),
            )
        return None


# ------------------------------------------------------------ trash cans ----
class BinDetector:
    """YOLO trash-can detector. Slow on a Pi, so use AsyncBinDetector."""

    def __init__(self):
        from ultralytics import YOLO
        self.model = YOLO(config.BIN_MODEL)
        if "world" in config.BIN_MODEL.lower():
            self.model.set_classes(config.BIN_PROMPTS)
            self.class_ids = None  # every prompt class is a trash can
        else:
            wanted = {n.lower() for n in config.BIN_CLASS_NAMES}
            self.class_ids = [
                i for i, name in self.model.names.items() if name.lower() in wanted
            ]
            if not self.class_ids:
                raise ValueError(f"{config.BIN_MODEL} has no class {config.BIN_CLASS_NAMES}")

    def find(self, frame):
        """Return a list of Targets, most confident first."""
        result = self.model.predict(
            frame,
            imgsz=config.BIN_IMGSZ,
            conf=config.BIN_CONFIDENCE,
            classes=self.class_ids,
            agnostic_nms=True,  # one box per can even if several prompts match
            verbose=False,
        )[0]
        h, w = frame.shape[:2]
        f = focal_length_px(w)
        targets = []
        for box, conf in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist()):
            x1, y1, x2, y2 = box
            # Pinhole estimate from box height. Underestimates when the can is
            # cut off by the frame edge, which only happens when it's close.
            distance = f * config.BIN_HEIGHT_M / max(y2 - y1, 1)
            targets.append(Target(
                bearing=bearing_of((x1 + x2) / 2, w),
                distance=distance,
                box=(int(x1), int(y1), int(x2), int(y2)),
                confidence=conf,
            ))
        return sorted(targets, key=lambda t: -t.confidence)


class AsyncBinDetector:
    """Runs BinDetector on a background thread so following stays smooth.

    submit() hands over the newest frame plus the robot pose at capture time;
    latest() returns (targets, pose, timestamp) of the most recent result.
    """

    def __init__(self):
        self.detector = BinDetector()
        self._lock = threading.Lock()
        self._pending = None
        self._result = ([], None, 0.0)
        self._wake = threading.Event()
        threading.Thread(target=self._loop, daemon=True).start()

    def submit(self, frame, pose):
        with self._lock:
            self._pending = (frame.copy(), pose)
        self._wake.set()

    def latest(self):
        with self._lock:
            return self._result

    def _loop(self):
        while True:
            self._wake.wait()
            self._wake.clear()
            with self._lock:
                job, self._pending = self._pending, None
            if job is None:
                continue
            frame, pose = job
            targets = self.detector.find(frame)
            with self._lock:
                self._result = (targets, pose, time.time())


def show_camera():
    """Quick check that the camera works. Press q to quit."""
    camera = get_camera()
    while True:
        success, frame = camera.read()
        if not success:
            print("Could not read frame")
            break
        cv2.imshow("Robot Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    show_camera()
