#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_system_default


class fake_mir_joint_publisher(Node):

    def __init__(self):
        super().__init__('fake_mir_joint_publisher')

        self.declare_parameter('tf_prefix', '')
        self.tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value.strip('/')
        if self.tf_prefix != "":
            self.tf_prefix = self.tf_prefix + '/'

        self.pub = self.create_publisher(
                msg_type=JointState,
                topic='joint_states',  # no prefix to joint states, just namespace
                qos_profile=qos_profile_system_default  # TODO Check QoS Settings
        )

        pub_rate = 1.0  # seconds
        self.timer = self.create_timer(pub_rate, self.publish_joint_states)

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.tf_prefix + 'left_wheel_joint', self.tf_prefix + 'right_wheel_joint',
                   self.tf_prefix + 'fl_caster_rotation_joint', self.tf_prefix + 'fl_caster_wheel_joint',
                   self.tf_prefix + 'fr_caster_rotation_joint', self.tf_prefix + 'fr_caster_wheel_joint',
                   self.tf_prefix + 'bl_caster_rotation_joint', self.tf_prefix + 'bl_caster_wheel_joint',
                   self.tf_prefix + 'br_caster_rotation_joint', self.tf_prefix + 'br_caster_wheel_joint']
        js.position = [0.0 for _ in js.name]
        js.velocity = [0.0 for _ in js.name]
        js.effort = [0.0 for _ in js.name]
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = fake_mir_joint_publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
