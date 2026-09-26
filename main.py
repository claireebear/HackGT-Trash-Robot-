"""TrashBot entry point.

    python3 main.py            # run the robot
    python3 main.py --no-yolo  # person following only (fast laptop test)

Keys in the debug window: q = quit, t = toggle "trash loaded" (simulation).
"""
import argparse
import math
import time

import cv2

import config
from navigation import BinMap, Odometry
from robot import Robot
from state_machine import Observation, TrashBotBrain
from vision import AsyncBinDetector, PersonTracker, get_camera


def draw(frame, brain, obs, pose):
    if obs.person:
        x1, y1, x2, y2 = obs.person.box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"person {obs.person.distance:.1f}m", (x1, y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    for t in obs.bins:
        x1, y1, x2, y2 = t.box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 140, 255), 2)
        cv2.putText(frame, f"trash can {t.confidence:.2f} ~{t.distance:.1f}m",
                    (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 1)
    lines = [
        f"state: {brain.state}",
        f"pose: ({pose.x:.2f}, {pose.y:.2f}) {math.degrees(pose.theta):.0f}deg",
        f"known cans: {len(brain.bin_map.bins)}   trash: {obs.has_trash}",
        f"ultrasonic: {obs.distance_cm} cm",
    ]
    for i, text in enumerate(lines):
        cv2.putText(frame, text, (10, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 2)
    cv2.imshow("TrashBot", frame)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-yolo", action="store_true", help="disable trash-can detection")
    args = parser.parse_args()

    robot = Robot()
    camera = get_camera()
    person_tracker = PersonTracker()
    bin_detector = None if args.no_yolo else AsyncBinDetector()
    odometry = Odometry()
    brain = TrashBotBrain(robot, BinMap())

    last = time.time()
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                print("[MAIN] camera frame failed")
                robot.stop()
                time.sleep(0.1)
                continue

            now = time.time()
            # integrate the speeds that were active since the last tick
            odometry.update(robot.left_speed, robot.right_speed, now - last)
            last = now
            pose = odometry.snapshot()

            obs = Observation(
                now=now,
                pose=pose,
                person=person_tracker.find(frame),
                distance_cm=robot.get_distance(),
                has_trash=robot.has_trash,
            )
            if bin_detector:
                bin_detector.submit(frame, pose)
                obs.bins, obs.bins_pose, obs.bins_time = bin_detector.latest()

            brain.step(obs)
            last = time.time()  # THROW blocks; don't count that time as motion

            if config.DISPLAY:
                draw(frame, brain, obs, pose)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("t"):
                    robot.simulated_trash = not robot.simulated_trash
                    print(f"[MAIN] simulated trash loaded: {robot.simulated_trash}")
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
