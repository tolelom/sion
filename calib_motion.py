#!/usr/bin/env python3
import time
from jetbot import Robot   # test_robot.py 안의 Robot 클래스 사용

robot = Robot()

def drive_test(speed, duration):
    print(f"\n=== DRIVE TEST: speed={speed}, duration={duration}s ===")
    robot.set_motors(speed, speed)
    time.sleep(duration)
    robot.set_motors(0, 0)
    print(">>> Measure the forward distance (cm) and write it down.\n")
    time.sleep(2)


def turn_test(turn_speed, duration):
    print(f"\n=== TURN TEST: turn_speed={turn_speed}, duration={duration}s ===")
    # 좌-우 반대방향으로 회전
    robot.set_motors(-turn_speed, turn_speed)
    time.sleep(duration)
    robot.set_motors(0, 0)
    print(">>> Measure the rotation angle (degrees) and write it down.\n")
    time.sleep(2)


if __name__ == "__main__":
    print("=== Calibration Start ===")
    time.sleep(2)

    # 직진 테스트
    #drive_test(0.2, 2.0)
    #drive_test(0.4, 2.0)
    #drive_test(0.6, 2.0)

    # 회전 테스트
    #turn_test(0.2, 2.0)
    #turn_test(0.4, 2.0)
    #turn_test(0.6, 2.0)

    robot.set_motors(0, 0)
    print("=== Calibration Done ===")
