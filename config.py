"""All tunable constants for TrashBot. Tweak these during testing."""

# ---------------------------------------------------------------- camera ----
# 0 = first USB webcam. For a phone camera, install "IP Webcam" (Android) or
# "DroidCam" (iOS/Android) and put the stream URL here, e.g.
#   CAMERA_SOURCE = "http://192.168.1.42:8080/video"
CAMERA_SOURCE = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_HFOV_DEG = 60.0  # horizontal field of view; most webcams are 55-70

# Show the debug window with detections drawn on it (turn off when headless).
DISPLAY = True

# --------------------------------------------------------- person (tag) ----
# The person wears a printed AprilTag (family tag36h11) on their back.
TAG_FAMILY = "tag36h11"
PERSON_TAG_ID = 0
TAG_SIZE_M = 0.15  # black border edge length of the printed tag, in meters

# --------------------------------------------------------- trash cans -------
# Default: YOLO-World, an open-vocabulary model: you describe what to find in
# words (BIN_PROMPTS). In testing it found trash cans at 0.6-0.8 confidence
# where the Open Images model ("yolov8n-oiv7.pt", class "Waste container")
# scored 0.35 or missed them. Switch to the oiv7 model if the Pi is too slow.
BIN_MODEL = "yolov8s-worldv2.pt"
BIN_PROMPTS = ["trash can", "garbage bin", "recycling bin"]  # used by YOLO-World
BIN_CLASS_NAMES = ["Waste container"]  # used by the Open Images model
BIN_CONFIDENCE = 0.3
BIN_IMGSZ = 320
BIN_HEIGHT_M = 0.8  # typical trash can height, used to estimate distance

# Two sightings closer than this are treated as the same trash can.
BIN_MERGE_RADIUS_M = 1.0
BIN_MAP_FILE = "bins.json"

# ---------------------------------------------------------------- arduino ---
SERIAL_PORT = "/dev/ttyACM0"  # Mac: something like /dev/tty.usbmodem14101
SERIAL_BAUD = 115200

# ------------------------------------------------------------- behaviour ----
MAX_SPEED = 180            # motor PWM, 0-255
FOLLOW_DISTANCE_M = 1.0    # how far behind the person to stay
FOLLOW_KP_DIST = 150       # PWM per meter of distance error
TURN_KP = 220              # PWM per radian of bearing error
SEARCH_TURN_SPEED = 90     # PWM when spinning in place to look around
PERSON_LOST_TIMEOUT_S = 1.0

OBSTACLE_STOP_CM = 25      # ultrasonic: never drive forward closer than this
DUMP_DISTANCE_CM = 30      # ultrasonic: close enough to the trash can to dump
BIN_LOST_TIMEOUT_S = 2.0
SEARCH_BIN_TIMEOUT_S = 12.0  # give up spinning for a trash can after this
NAV_ARRIVE_M = 1.2         # dead-reckoned "close enough, start looking" radius

# Robot goes to dump once trash has been sensed in the compartment this long.
TRASH_CONFIRM_S = 1.5

# ------------------------------------------------------------- odometry -----
# No wheel encoders, so position is estimated from the motor commands.
# Calibrate: drive at PWM 150 for 2 s, measure meters travelled -> M_PER_S_PER_PWM
# = meters / 2 / 150. Spin at +/-150 for 2 s, measure radians -> RAD_PER_S_PER_PWM
# = radians / 2 / 150.
M_PER_S_PER_PWM = 0.0025
RAD_PER_S_PER_PWM = 0.012
