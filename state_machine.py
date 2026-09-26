"""TrashBot behaviour.

    FOLLOW ─(person tag lost)─► FIND_PERSON ─(tag seen)─► FOLLOW
    FOLLOW ─(trash in compartment + a trash can is known)─► GO_TO_BIN
    GO_TO_BIN ─(can visible)─► APPROACH_BIN ─(ultrasonic close)─► THROW
    GO_TO_BIN ─(reached remembered spot, can not visible)─► SEARCH_BIN
    SEARCH_BIN ─(can visible)─► APPROACH_BIN
    THROW ─► FIND_PERSON

Trash cans are recorded into the map in every state whenever the camera sees one.
"""
import math
import time
from dataclasses import dataclass, field

import config
from navigation import to_world, wrap_angle

FOLLOW = "FOLLOW"
FIND_PERSON = "FIND_PERSON"
GO_TO_BIN = "GO_TO_BIN"
SEARCH_BIN = "SEARCH_BIN"
APPROACH_BIN = "APPROACH_BIN"
THROW = "THROW"


@dataclass
class Observation:
    """Everything the state machine gets to know each tick."""
    now: float
    pose: object                 # navigation.Pose, current estimate
    person: object = None        # vision.Target or None
    bins: list = field(default_factory=list)  # vision.Targets from the latest YOLO run
    bins_pose: object = None     # robot pose when that YOLO frame was captured
    bins_time: float = 0.0       # when that YOLO result finished
    distance_cm: float = None    # ultrasonic, None if unknown
    has_trash: bool = False


class TrashBotBrain:
    def __init__(self, robot, bin_map):
        self.robot = robot
        self.bin_map = bin_map
        self.state = FIND_PERSON
        self.state_since = time.time()
        self.person_last_seen = 0.0
        self.person_last_bearing = 0.0
        self.trash_since = None
        self.target_bin = None   # TrashCan we are heading to
        self.bin_fix = None      # (x, y, time) of the latest can sighting
        self._last_bins_time = 0.0

    # ---------------------------------------------------------------------
    def step(self, obs):
        self._record_bins(obs)
        if obs.person:
            self.person_last_seen = obs.now
            self.person_last_bearing = obs.person.bearing
        self.trash_since = (self.trash_since or obs.now) if obs.has_trash else None

        handler = getattr(self, "_" + self.state.lower())
        handler(obs)

    def _go(self, state):
        if state != self.state:
            print(f"[STATE] {self.state} -> {state}")
            self.state = state
            self.state_since = time.time()

    def _time_in_state(self, obs):
        return obs.now - self.state_since

    # ------------------------------------------------------------ states --
    def _follow(self, obs):
        if self._should_dump(obs):
            self.target_bin = self.bin_map.nearest(obs.pose)
            self._go(GO_TO_BIN)
            return
        if obs.person is None:
            if obs.now - self.person_last_seen > config.PERSON_LOST_TIMEOUT_S:
                self._go(FIND_PERSON)
            else:
                self.robot.stop()  # brief dropout: wait rather than guess
            return
        forward = (obs.person.distance - config.FOLLOW_DISTANCE_M) * config.FOLLOW_KP_DIST
        forward = max(0, forward)  # never back up into things
        self._drive(forward, obs.person.bearing, obs)

    def _find_person(self, obs):
        if obs.person is not None:
            self._go(FOLLOW)
            return
        if self._should_dump(obs):
            self.target_bin = self.bin_map.nearest(obs.pose)
            self._go(GO_TO_BIN)
            return
        # spin toward the side the person was last seen on
        direction = 1 if self.person_last_bearing >= 0 else -1
        self._spin(direction)

    def _go_to_bin(self, obs):
        if self._bin_visible(obs):
            self._go(APPROACH_BIN)
            return
        dx = self.target_bin.x - obs.pose.x
        dy = self.target_bin.y - obs.pose.y
        if math.hypot(dx, dy) < config.NAV_ARRIVE_M:
            self._go(SEARCH_BIN)
            return
        bearing = wrap_angle(math.atan2(dy, dx) - obs.pose.theta)
        # turn in place first if facing the wrong way, then drive
        forward = 0 if abs(bearing) > math.radians(30) else config.MAX_SPEED * 0.7
        self._drive(forward, bearing, obs)

    def _search_bin(self, obs):
        if self._bin_visible(obs):
            self._go(APPROACH_BIN)
            return
        if self._time_in_state(obs) > config.SEARCH_BIN_TIMEOUT_S:
            print("[STATE] couldn't find a trash can here, going back to the person")
            self._go(FIND_PERSON)
            return
        self._spin(1)

    def _approach_bin(self, obs):
        if not self._bin_visible(obs):
            self._go(SEARCH_BIN)
            return
        if obs.distance_cm is not None and obs.distance_cm <= config.DUMP_DISTANCE_CM:
            self._go(THROW)
            self.robot.stop()
            return
        x, y, _ = self.bin_fix
        dx, dy = x - obs.pose.x, y - obs.pose.y
        bearing = wrap_angle(math.atan2(dy, dx) - obs.pose.theta)
        # slow down as we get close; the ultrasonic decides when we've arrived
        forward = min(config.MAX_SPEED * 0.6, 60 + 80 * math.hypot(dx, dy))
        if abs(bearing) > math.radians(25):
            forward = 0
        self._drive(forward, bearing, obs, obstacle_stop=False)

        if self.robot.simulated and self._time_in_state(obs) > 3:
            # no ultrasonic in simulation: pretend we arrived
            self._go(THROW)

    def _throw(self, obs):
        print("[STATE] dumping trash!")
        self.robot.throw()
        # we're right next to the can now: good moment to pin down its location
        self.bin_map.record(*to_world(obs.pose, 0.0, config.DUMP_DISTANCE_CM / 100))
        self.trash_since = None
        self._go(FIND_PERSON)

    # ----------------------------------------------------------- helpers --
    def _should_dump(self, obs):
        return (
            self.trash_since is not None
            and obs.now - self.trash_since >= config.TRASH_CONFIRM_S
            and self.bin_map.bins
        )

    def _record_bins(self, obs):
        """Store each fresh YOLO sighting in the map and remember the best one."""
        if obs.bins_time <= self._last_bins_time or obs.bins_pose is None:
            return
        self._last_bins_time = obs.bins_time
        for i, target in enumerate(obs.bins):
            x, y = to_world(obs.bins_pose, target.bearing, target.distance)
            self.bin_map.record(x, y)
            if i == 0:
                self.bin_fix = (x, y, obs.bins_time)

    def _bin_visible(self, obs):
        return (
            self.bin_fix is not None
            and obs.now - self.bin_fix[2] < config.BIN_LOST_TIMEOUT_S
        )

    def _drive(self, forward, bearing, obs, obstacle_stop=True):
        """Drive forward while steering toward `bearing` (radians, + = left)."""
        if (obstacle_stop and obs.distance_cm is not None
                and obs.distance_cm < config.OBSTACLE_STOP_CM):
            forward = 0
        turn = bearing * config.TURN_KP
        self.robot.drive(forward - turn, forward + turn)

    def _spin(self, direction):
        s = config.SEARCH_TURN_SPEED * direction
        self.robot.drive(-s, s)
