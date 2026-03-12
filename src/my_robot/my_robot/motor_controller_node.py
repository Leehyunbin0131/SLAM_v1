#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    GroupSyncRead,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_LOWORD,
    DXL_HIWORD,
)

# XL430-W250-T Control Table (Protocol 2.0)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

LEN_GOAL_VELOCITY = 4
LEN_PRESENT_VELOCITY = 4
LEN_PRESENT_POSITION = 4

VELOCITY_CONTROL_MODE = 1
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# XL430-W250-T: 1 unit ≈ 0.229 rev/min
DXL_VELOCITY_UNIT = 0.229  # rev/min per unit


def dxl_velocity_to_rad_per_sec(dxl_value: int) -> float:
    """Convert Dynamixel velocity unit to rad/s."""
    rpm = dxl_value * DXL_VELOCITY_UNIT
    return rpm * 2.0 * math.pi / 60.0


def rad_per_sec_to_dxl_velocity(rad_s: float, max_dxl: int = 265) -> int:
    """Convert rad/s to Dynamixel velocity unit (signed, clamped)."""
    rpm = rad_s * 60.0 / (2.0 * math.pi)
    dxl = int(round(rpm / DXL_VELOCITY_UNIT))
    return max(-max_dxl, min(max_dxl, dxl))


def signed32(val: int) -> int:
    """Convert unsigned 32-bit read value to signed."""
    if val > 0x7FFFFFFF:
        return val - 0x100000000
    return val


def to_unsigned32(val: int) -> int:
    """Convert signed int to unsigned 32-bit for Dynamixel write."""
    if val < 0:
        return val + 0x100000000
    return val


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Parameters
        self.declare_parameter('device_port', '/dev/dynamixel')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('left_motor_id', 1)
        self.declare_parameter('right_motor_id', 2)
        self.declare_parameter('left_motor_sign', 1)
        self.declare_parameter('right_motor_sign', -1)
        self.declare_parameter('wheel_radius', 0.026)
        self.declare_parameter('wheel_separation', 0.12)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('max_dxl_velocity', 180)

        self.device_port = self.get_parameter('device_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.left_id = self.get_parameter('left_motor_id').value
        self.right_id = self.get_parameter('right_motor_id').value
        self.left_motor_sign = int(self.get_parameter('left_motor_sign').value)
        self.right_motor_sign = int(self.get_parameter('right_motor_sign').value)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.publish_odom = self.get_parameter('publish_odom').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.max_dxl_velocity = self.get_parameter('max_dxl_velocity').value

        self._validate_motor_sign(self.left_motor_sign, 'left_motor_sign')
        self._validate_motor_sign(self.right_motor_sign, 'right_motor_sign')

        self.port_handler = PortHandler(self.device_port)
        self.packet_handler = PacketHandler(2.0)

        if not self.port_handler.openPort():
            self.get_logger().fatal(f'Failed to open port: {self.device_port}')
            raise RuntimeError(f'Cannot open port {self.device_port}')
        self.get_logger().info(f'Port opened: {self.device_port}')

        if not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().fatal(f'Failed to set baudrate: {self.baudrate}')
            raise RuntimeError(f'Cannot set baudrate {self.baudrate}')
        self.get_logger().info(f'Baudrate set: {self.baudrate}')

        self._init_motor(self.left_id, 'left')
        self._init_motor(self.right_id, 'right')

        # GroupSyncWrite for goal velocity
        self.sync_write_vel = GroupSyncWrite(
            self.port_handler, self.packet_handler,
            ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)

        # GroupSyncRead for present velocity
        self.sync_read_vel = GroupSyncRead(
            self.port_handler, self.packet_handler,
            ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
        self.sync_read_vel.addParam(self.left_id)
        self.sync_read_vel.addParam(self.right_id)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()

        # cmd_vel tracking
        self.last_cmd_vel_time = self.get_clock().now()
        self.target_linear = 0.0
        self.target_angular = 0.0

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for control loop at 30 Hz
        self.control_timer = self.create_timer(1.0 / 30.0, self._control_loop)

        self.get_logger().info(
            f'Motor controller started — '
            f'L:{self.left_id} R:{self.right_id} '
            f'Lsign:{self.left_motor_sign} Rsign:{self.right_motor_sign} '
            f'wheel_r:{self.wheel_radius} separation:{self.wheel_separation}')

    def _validate_motor_sign(self, sign: int, param_name: str):
        if sign not in (-1, 1):
            raise ValueError(f'{param_name} must be either -1 or 1, got {sign}')

    def _init_motor(self, motor_id: int, name: str):
        """Set motor to velocity control mode and enable torque."""
        # Disable torque first (required to change operating mode)
        res, err = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if res != COMM_SUCCESS:
            self.get_logger().error(
                f'[{name} ID:{motor_id}] torque disable failed: '
                f'{self.packet_handler.getTxRxResult(res)}')
            return

        # Set velocity control mode
        res, err = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
        if res != COMM_SUCCESS:
            self.get_logger().error(
                f'[{name} ID:{motor_id}] set velocity mode failed: '
                f'{self.packet_handler.getTxRxResult(res)}')
            return

        # Enable torque
        res, err = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if res != COMM_SUCCESS:
            self.get_logger().error(
                f'[{name} ID:{motor_id}] torque enable failed: '
                f'{self.packet_handler.getTxRxResult(res)}')
            return

        self.get_logger().info(f'[{name} ID:{motor_id}] initialized — velocity mode, torque ON')

    def _cmd_vel_callback(self, msg: Twist):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

    def _control_loop(self):
        now = self.get_clock().now()

        # Safety: stop if no cmd_vel received within timeout
        dt_cmd = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if dt_cmd > self.cmd_vel_timeout:
            self.target_linear = 0.0
            self.target_angular = 0.0

        # Differential drive inverse kinematics (rad/s at wheel)
        left_wheel_rad_s = (
            self.target_linear - self.target_angular * self.wheel_separation / 2.0
        ) / self.wheel_radius
        right_wheel_rad_s = (
            self.target_linear + self.target_angular * self.wheel_separation / 2.0
        ) / self.wheel_radius

        # Motor shaft positive direction depends on mounting, so keep it configurable.
        left_dxl = rad_per_sec_to_dxl_velocity(
            self.left_motor_sign * left_wheel_rad_s,
            self.max_dxl_velocity)
        right_dxl = rad_per_sec_to_dxl_velocity(
            self.right_motor_sign * right_wheel_rad_s,
            self.max_dxl_velocity)

        try:
            self._sync_write_velocity(left_dxl, right_dxl)

            left_vel_dxl, right_vel_dxl = self._sync_read_velocity()
            if left_vel_dxl is not None and right_vel_dxl is not None:
                left_rad_s = (
                    self.left_motor_sign * dxl_velocity_to_rad_per_sec(left_vel_dxl))
                right_rad_s = (
                    self.right_motor_sign * dxl_velocity_to_rad_per_sec(right_vel_dxl))

                self._update_odometry(left_rad_s, right_rad_s, now)
        except Exception as e:
            self.get_logger().warn(f'Serial error (retrying): {e}')
            self.port_handler.is_using = False

    def _sync_write_velocity(self, left_dxl: int, right_dxl: int):
        left_u = to_unsigned32(left_dxl)
        right_u = to_unsigned32(right_dxl)

        left_data = [
            DXL_LOBYTE(DXL_LOWORD(left_u)),
            DXL_HIBYTE(DXL_LOWORD(left_u)),
            DXL_LOBYTE(DXL_HIWORD(left_u)),
            DXL_HIBYTE(DXL_HIWORD(left_u)),
        ]
        right_data = [
            DXL_LOBYTE(DXL_LOWORD(right_u)),
            DXL_HIBYTE(DXL_LOWORD(right_u)),
            DXL_LOBYTE(DXL_HIWORD(right_u)),
            DXL_HIBYTE(DXL_HIWORD(right_u)),
        ]

        self.sync_write_vel.addParam(self.left_id, left_data)
        self.sync_write_vel.addParam(self.right_id, right_data)
        result = self.sync_write_vel.txPacket()
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'SyncWrite failed: {self.packet_handler.getTxRxResult(result)}')
        self.sync_write_vel.clearParam()

    def _sync_read_velocity(self):
        result = self.sync_read_vel.txRxPacket()
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'SyncRead failed: {self.packet_handler.getTxRxResult(result)}')
            return None, None

        if not self.sync_read_vel.isAvailable(
                self.left_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY):
            return None, None
        if not self.sync_read_vel.isAvailable(
                self.right_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY):
            return None, None

        left_raw = self.sync_read_vel.getData(
            self.left_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
        right_raw = self.sync_read_vel.getData(
            self.right_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)

        return signed32(left_raw), signed32(right_raw)

    def _update_odometry(self, left_rad_s: float, right_rad_s: float,
                         now: rclpy.time.Time):
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        left_linear = left_rad_s * self.wheel_radius
        right_linear = right_rad_s * self.wheel_radius

        linear_vel = (left_linear + right_linear) / 2.0
        angular_vel = (right_linear - left_linear) / self.wheel_separation

        # Integrate pose
        if abs(angular_vel) < 1e-6:
            self.x += linear_vel * math.cos(self.theta) * dt
            self.y += linear_vel * math.sin(self.theta) * dt
        else:
            radius = linear_vel / angular_vel
            self.x += radius * (math.sin(self.theta + angular_vel * dt) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + angular_vel * dt) - math.cos(self.theta))
        self.theta += angular_vel * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        if not self.publish_odom:
            return

        # Quaternion from yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.03  # yaw
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        odom.twist.covariance[0] = 0.01   # vx
        odom.twist.covariance[35] = 0.03  # vyaw
        self.odom_pub.publish(odom)

    def shutdown(self):
        """Stop motors and disable torque on shutdown."""
        self.get_logger().info('Shutting down — stopping motors...')
        try:
            for motor_id in [self.left_id, self.right_id]:
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, motor_id, ADDR_GOAL_VELOCITY, 0)
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')
        finally:
            self.port_handler.closePort()
            self.get_logger().info('Port closed.')


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
