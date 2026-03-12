#!/usr/bin/env python3
"""BNO085 IMU sensor node — publishes sensor_msgs/Imu on /imu/data."""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

try:
    import board
    import digitalio
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import (
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_MAGNETOMETER,
    )
except ImportError as e:
    raise RuntimeError(
        'adafruit-circuitpython-bno08x not found. '
        'Install: pip install adafruit-circuitpython-bno08x'
    ) from e


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('i2c_address', 0x4B)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('reset_gpio', 18)

        self.frame_id = self.get_parameter('frame_id').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.reset_gpio = self.get_parameter('reset_gpio').value

        self.bno = self._init_sensor()

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)
        self.get_logger().info(
            f'IMU node started — {self.publish_rate} Hz, frame: {self.frame_id}')

    def _get_reset_pin(self):
        gpio_map = {
            4: board.D4, 5: board.D5, 6: board.D6,
            12: board.D12, 13: board.D13, 16: board.D16,
            17: board.D17, 18: board.D18, 19: board.D19,
            20: board.D20, 21: board.D21, 22: board.D22,
            23: board.D23, 24: board.D24, 25: board.D25,
            26: board.D26, 27: board.D27,
        }
        pin = gpio_map.get(self.reset_gpio)
        if pin is None:
            self.get_logger().warn(f'GPIO {self.reset_gpio} not mapped, skipping HW reset')
            return None
        rst = digitalio.DigitalInOut(pin)
        rst.direction = digitalio.Direction.OUTPUT
        return rst

    def _init_sensor(self):
        self.get_logger().info('Initializing BNO085...')

        reset_pin = self._get_reset_pin()
        if reset_pin is not None:
            self.get_logger().info('  Hardware reset via GPIO...')
            reset_pin.value = False
            time.sleep(0.3)
            reset_pin.value = True
            time.sleep(0.3)

        i2c = board.I2C()
        bno = BNO08X_I2C(i2c, address=self.i2c_address)

        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        time.sleep(0.05)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)
        time.sleep(0.05)
        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        time.sleep(0.05)
        bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        time.sleep(0.05)

        # Verify first read
        _ = bno.quaternion
        time.sleep(0.1)

        self.get_logger().info('  BNO085 initialized successfully')
        return bno

    def _publish(self):
        try:
            now = self.get_clock().now().to_msg()

            # --- IMU message ---
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = self.frame_id

            quat = self.bno.quaternion
            if quat is not None:
                imu_msg.orientation.x = float(quat[0])
                imu_msg.orientation.y = float(quat[1])
                imu_msg.orientation.z = float(quat[2])
                imu_msg.orientation.w = float(quat[3])
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.01

            gyro = self.bno.gyro
            if gyro is not None:
                imu_msg.angular_velocity.x = -float(gyro[0])
                imu_msg.angular_velocity.y = float(gyro[1])
                imu_msg.angular_velocity.z = float(gyro[2])
            imu_msg.angular_velocity_covariance[0] = 0.001
            imu_msg.angular_velocity_covariance[4] = 0.001
            imu_msg.angular_velocity_covariance[8] = 0.001

            accel = self.bno.acceleration
            if accel is not None:
                imu_msg.linear_acceleration.x = -float(accel[0])
                imu_msg.linear_acceleration.y = float(accel[1])
                imu_msg.linear_acceleration.z = float(accel[2])
            imu_msg.linear_acceleration_covariance[0] = 0.1
            imu_msg.linear_acceleration_covariance[4] = 0.1
            imu_msg.linear_acceleration_covariance[8] = 0.1

            self.imu_pub.publish(imu_msg)

            # --- MagneticField message ---
            mag = self.bno.magnetic
            if mag is not None:
                mag_msg = MagneticField()
                mag_msg.header.stamp = now
                mag_msg.header.frame_id = self.frame_id
                mag_msg.magnetic_field.x = float(mag[0]) * 1e-6
                mag_msg.magnetic_field.y = float(mag[1]) * 1e-6
                mag_msg.magnetic_field.z = float(mag[2]) * 1e-6
                self.mag_pub.publish(mag_msg)

        except Exception as e:
            self.get_logger().error(f'IMU read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
