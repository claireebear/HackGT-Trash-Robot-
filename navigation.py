"""Where the robot is (dead reckoning) and where the trash cans are."""
import json
import math
import os
import time
from dataclasses import asdict, dataclass

import config


@dataclass
class Pose:
    x: float = 0.0      # meters, +x = the direction the robot faced at startup
    y: float = 0.0      # meters, +y = to the left of that
    theta: float = 0.0  # radians, counter-clockwise


class Odometry:
    """Estimates pose from motor commands. No encoders, so it drifts: use it
    to get roughly back to a trash can, then let the camera take over."""

    def __init__(self):
        self.pose = Pose()

    def update(self, left_pwm, right_pwm, dt):
        v = (left_pwm + right_pwm) / 2 * config.M_PER_S_PER_PWM
        w = (right_pwm - left_pwm) / 2 * config.RAD_PER_S_PER_PWM
        p = self.pose
        p.x += v * math.cos(p.theta) * dt
        p.y += v * math.sin(p.theta) * dt
        p.theta = wrap_angle(p.theta + w * dt)

    def snapshot(self):
        return Pose(self.pose.x, self.pose.y, self.pose.theta)


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def to_world(pose, bearing, distance):
    """Convert a camera sighting (relative to the robot) to map coordinates."""
    angle = pose.theta + bearing
    return pose.x + distance * math.cos(angle), pose.y + distance * math.sin(angle)


@dataclass
class TrashCan:
    id: int
    x: float
    y: float
    sightings: int = 1
    last_seen: float = 0.0


class BinMap:
    """Remembered trash can locations, saved to disk between runs."""

    def __init__(self, path=config.BIN_MAP_FILE):
        self.path = path
        self.bins = []
        if path and os.path.exists(path):
            with open(path) as f:
                self.bins = [TrashCan(**b) for b in json.load(f)]
            print(f"[MAP] loaded {len(self.bins)} trash can(s) from {path}")

    def record(self, x, y):
        """Add a sighting. Nearby sightings are averaged into one trash can."""
        now = time.time()
        for b in self.bins:
            if math.hypot(b.x - x, b.y - y) < config.BIN_MERGE_RADIUS_M:
                # running average, capped so a moved can eventually updates
                n = min(b.sightings, 10)
                b.x = (b.x * n + x) / (n + 1)
                b.y = (b.y * n + y) / (n + 1)
                b.sightings += 1
                b.last_seen = now
                self._save()
                return b
        b = TrashCan(id=len(self.bins), x=x, y=y, last_seen=now)
        self.bins.append(b)
        print(f"[MAP] new trash can #{b.id} at ({x:.2f}, {y:.2f})")
        self._save()
        return b

    def nearest(self, pose):
        if not self.bins:
            return None
        return min(self.bins, key=lambda b: math.hypot(b.x - pose.x, b.y - pose.y))

    def _save(self):
        if self.path:
            with open(self.path, "w") as f:
                json.dump([asdict(b) for b in self.bins], f, indent=2)
