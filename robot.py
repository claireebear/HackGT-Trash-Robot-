"""Talks to the Arduino (which drives the TB6612) over USB serial.

Protocol, one line per message:
    Pi -> Arduino   "M <left> <right>"   motor PWM, -255..255
                    "S"                  stop
                    "T"                  throw (tilt servo, dump, return)
    Arduino -> Pi   "D <cm> L <0|1>"     ultrasonic distance, trash-in-compartment

If no Arduino is connected, the robot runs in simulation mode and just prints
commands, so everything can be tested on a laptop.
"""
import threading
import time

import config

try:
    import serial
except ImportError:
    serial = None


class Robot:
    def __init__(self, port=config.SERIAL_PORT, baud=config.SERIAL_BAUD):
        self.left_speed = 0
        self.right_speed = 0
        self.distance_cm = None      # latest ultrasonic reading
        self.has_trash_sensor = False
        self.simulated_trash = False  # toggled with the 't' key in sim mode
        self._last_printed = None
        self._serial = None

        if serial is not None:
            try:
                self._serial = serial.Serial(port, baud, timeout=0.1)
                time.sleep(2)  # Arduino resets when the port opens
                threading.Thread(target=self._read_loop, daemon=True).start()
                print(f"[ROBOT] connected to Arduino on {port}")
            except (serial.SerialException, OSError) as e:
                print(f"[ROBOT] no Arduino ({e}); running in SIMULATION mode")
        else:
            print("[ROBOT] pyserial not installed; running in SIMULATION mode")

    @property
    def simulated(self):
        return self._serial is None

    @property
    def has_trash(self):
        return self.has_trash_sensor or self.simulated_trash

    # ------------------------------------------------------------ motion --
    def drive(self, left, right):
        """Set wheel speeds, -255..255 each. Positive = forward."""
        left = int(max(-config.MAX_SPEED, min(config.MAX_SPEED, left)))
        right = int(max(-config.MAX_SPEED, min(config.MAX_SPEED, right)))
        self.left_speed, self.right_speed = left, right
        self._send(f"M {left} {right}")

    def forward(self, speed=150):
        self.drive(speed, speed)

    def backward(self, speed=150):
        self.drive(-speed, -speed)

    def left(self, speed=120):
        self.drive(-speed, speed)

    def right(self, speed=120):
        self.drive(speed, -speed)

    def stop(self):
        self.left_speed = self.right_speed = 0
        self._send("S")

    def throw(self):
        """Stop and dump the compartment. Blocks until the servo is done."""
        self.stop()
        self._send("T")
        time.sleep(2.5)
        self.simulated_trash = False

    def get_distance(self):
        return self.distance_cm

    # ---------------------------------------------------------- serial ----
    def _send(self, line):
        if self._serial is not None:
            self._serial.write((line + "\n").encode())
        elif line != self._last_printed:  # don't spam the terminal at 30 Hz
            print(f"[ROBOT] {line}")
            self._last_printed = line

    def _read_loop(self):
        while True:
            try:
                raw = self._serial.readline().decode(errors="ignore").split()
            except (serial.SerialException, OSError):
                print("[ROBOT] lost serial connection")
                return
            # expected: ["D", "<cm>", "L", "<0|1>"]
            if len(raw) == 4 and raw[0] == "D" and raw[2] == "L":
                try:
                    cm = int(raw[1])
                    self.distance_cm = cm if cm > 0 else None
                    self.has_trash_sensor = raw[3] == "1"
                except ValueError:
                    pass
