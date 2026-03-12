#!/usr/bin/env python3
"""
IMU 축 방향 확인 테스트 스크립트

ROS2 REP 103 기준 (ENU):
  x = 전방(Forward), y = 좌측(Left), z = 위(Up)
  yaw+ = 반시계 방향 (왼쪽 회전)

테스트 방법:
  1. 로봇을 평평한 곳에 놓고 실행
  2. 안내에 따라 로봇을 기울이거나 회전
  3. 출력 값을 보고 축이 REP 103과 일치하는지 확인

실행: python3 ~/my-ros2/scripts/test_imu_axes.py
"""

import time
import math
import board
import digitalio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ACCELEROMETER,
)

RESET_GPIO = 18
I2C_ADDR = 0x4B


def init_sensor():
    rst = digitalio.DigitalInOut(board.D18)
    rst.direction = digitalio.Direction.OUTPUT
    rst.value = False
    time.sleep(0.3)
    rst.value = True
    time.sleep(0.3)

    i2c = board.I2C()
    bno = BNO08X_I2C(i2c, address=I2C_ADDR)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    time.sleep(0.05)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    time.sleep(0.05)
    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    time.sleep(0.05)
    _ = bno.quaternion
    time.sleep(0.2)
    return bno


def quat_to_euler(x, y, z, w):
    """Quaternion (x,y,z,w) to roll, pitch, yaw in degrees."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def print_separator():
    print("-" * 60)


def wait_and_read(bno, seconds=3):
    """Read IMU for N seconds and return the last values."""
    end = time.time() + seconds
    while time.time() < end:
        q = bno.quaternion
        g = bno.gyro
        a = bno.acceleration
        time.sleep(0.05)
    return q, g, a


def main():
    print("BNO085 IMU 축 방향 테스트")
    print("ROS2 REP 103 기준: x=전방, y=좌측, z=위")
    print_separator()

    print("센서 초기화 중...")
    bno = init_sensor()
    print("초기화 완료!\n")

    # ========== TEST 1: 중력 방향 (가속도) ==========
    print_separator()
    print("테스트 1: 중력 방향 (로봇을 평평한 곳에 놓아주세요)")
    print("  REP 103 기준: az ≈ +9.8 (위쪽이 +z)")
    input("  준비되면 Enter...")
    _, _, accel = wait_and_read(bno, 2)
    if accel:
        print(f"  가속도: ax={accel[0]:+.2f}  ay={accel[1]:+.2f}  az={accel[2]:+.2f}")
        print(f"  → z축 방향: {'OK (위=+z)' if accel[2] > 5 else 'WRONG'}")
    print()

    # ========== TEST 2: 전방 기울이기 (pitch) ==========
    print_separator()
    print("테스트 2: 로봇 앞부분을 아래로 기울여주세요 (앞으로 숙이기)")
    print("  REP 103 기준: pitch < 0 (코가 아래), ax < 0")
    input("  준비되면 Enter (기울인 상태 유지)...")
    q, _, accel = wait_and_read(bno, 2)
    if q and accel:
        r, p, y = quat_to_euler(q[0], q[1], q[2], q[3])
        print(f"  가속도: ax={accel[0]:+.2f}  ay={accel[1]:+.2f}  az={accel[2]:+.2f}")
        print(f"  오일러:  roll={r:+.1f}°  pitch={p:+.1f}°  yaw={y:+.1f}°")
        print(f"  → 전방(x축): ax={'OK (-)' if accel[0] < -1 else 'REVERSED (+) → x축 반전 필요'}")
    print()

    # ========== TEST 3: 좌측 기울이기 (roll) ==========
    print_separator()
    print("테스트 3: 로봇을 왼쪽으로 기울여주세요")
    print("  REP 103 기준: roll < 0 (왼쪽 기울임), ay > 0")
    input("  준비되면 Enter (기울인 상태 유지)...")
    q, _, accel = wait_and_read(bno, 2)
    if q and accel:
        r, p, y = quat_to_euler(q[0], q[1], q[2], q[3])
        print(f"  가속도: ax={accel[0]:+.2f}  ay={accel[1]:+.2f}  az={accel[2]:+.2f}")
        print(f"  오일러:  roll={r:+.1f}°  pitch={p:+.1f}°  yaw={y:+.1f}°")
        print(f"  → 좌측(y축): ay={'OK (+)' if accel[1] > 1 else 'REVERSED (-) → y축 반전 필요'}")
    print()

    # ========== TEST 4: 왼쪽 회전 (yaw) ==========
    print_separator()
    print("테스트 4: 로봇을 평평하게 놓고, 위에서 봤을 때 반시계 방향(왼쪽)으로 천천히 회전")
    print("  REP 103 기준: gyro_z > 0, yaw 증가")
    input("  준비되면 Enter (회전 시작)...")
    print("  3초간 측정 중 (계속 회전하세요)...")
    q_start, _, _ = wait_and_read(bno, 0.5)
    _, gyro, _ = wait_and_read(bno, 3)
    q_end, _, _ = wait_and_read(bno, 0.5)
    if gyro and q_start and q_end:
        _, _, yaw_start = quat_to_euler(q_start[0], q_start[1], q_start[2], q_start[3])
        _, _, yaw_end = quat_to_euler(q_end[0], q_end[1], q_end[2], q_end[3])
        print(f"  자이로: gx={gyro[0]:+.2f}  gy={gyro[1]:+.2f}  gz={gyro[2]:+.2f} rad/s")
        print(f"  yaw 변화: {yaw_start:+.1f}° → {yaw_end:+.1f}°")
        print(f"  → 회전(z축): gz={'OK (+)' if gyro[2] > 0.1 else 'REVERSED (-) → z축 회전 반전 필요'}")
    print()

    # ========== 요약 ==========
    print_separator()
    print("결과 요약:")
    print("  위 테스트 결과를 보고 어떤 축이 반전되어 있는지 알려주세요.")
    print("  imu_node.py에서 해당 축의 부호를 반전시키면 됩니다.")
    print_separator()


if __name__ == '__main__':
    main()
