"""Publish the ADMA INS attitude as autoware_sensing_msgs/GnssInsOrientationStamped.

autoware_gnss_poser (use_gnss_ins_orientation: true) reads the antenna orientation from
/autoware_orientation; without it the GNSS pose always has yaw 0 (facing east).

ADMA ins_yaw is a heading: degrees clockwise from north. Autoware's map frame is ENU, where
yaw is counter-clockwise from east, so yaw_enu = 90 - heading. ins_roll / ins_pitch already
match the ROS x-forward / y-left / z-up body frame (checked against acc_body at standstill).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from adma_ros_driver_msgs.msg import AdmaDataScaled
from autoware_sensing_msgs.msg import GnssInsOrientationStamped


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class AutowareOrientationPublisher(Node):

    def __init__(self):
        super().__init__('adma_autoware_orientation')
        self.frame_id = self.declare_parameter('frame_id', 'gnss_link').value
        # extra yaw in degrees added after the conversion (ADMA mounting / map rotation)
        self.yaw_offset_deg = self.declare_parameter('yaw_offset_deg', 0.0).value
        self.pub = self.create_publisher(GnssInsOrientationStamped, 'autoware_orientation', 1)
        self.create_subscription(
            AdmaDataScaled, 'adma_data_scaled', self.on_adma, qos_profile_sensor_data)

    def on_adma(self, msg):
        yaw_enu = math.radians(90.0 - msg.ins_yaw + self.yaw_offset_deg)
        yaw_enu = math.atan2(math.sin(yaw_enu), math.cos(yaw_enu))
        qx, qy, qz, qw = quaternion_from_rpy(
            math.radians(msg.ins_roll), math.radians(msg.ins_pitch), yaw_enu)

        out = GnssInsOrientationStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.orientation.orientation.x = qx
        out.orientation.orientation.y = qy
        out.orientation.orientation.z = qz
        out.orientation.orientation.w = qw
        out.orientation.rmse_rotation_x = math.radians(msg.ins_stddev_roll)
        out.orientation.rmse_rotation_y = math.radians(msg.ins_stddev_pitch)
        out.orientation.rmse_rotation_z = math.radians(msg.ins_stddev_yaw)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AutowareOrientationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
