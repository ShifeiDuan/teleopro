#!/usr/bin/env python3
"""ROS 2 node that publishes a teleoperation target frame.

Subscribes:  TF input_base_frame -> input_source_frame   (input_type: pose)
             twist_topic                              (input_type: twist)
             TF robot_base_frame -> robot_source_frame
Publishes:   TF robot_base_frame -> target_frame
             ~/engaged (std_msgs/Bool)
Services:    ~/engage (std_srvs/SetBool)
"""

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from teleopro_core import as_pos_quat, Teleop
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleopro')

        self.input_type = self.declare_parameter('input_type', 'pose').value
        if self.input_type not in ('pose', 'twist'):
            raise ValueError(f'input_type must be "pose" or "twist", got "{self.input_type}"')

        # Frames
        self.input_base = self.declare_parameter('input_base_frame', '').value
        self.input_source = self.declare_parameter('input_source_frame', '').value
        self.robot_base = self.declare_parameter('robot_base_frame', '').value
        self.robot_source = self.declare_parameter('robot_source_frame', '').value
        self.target_frame = self.declare_parameter('target_frame', 'teleop_target').value
        required = ['robot_base_frame', 'robot_source_frame']
        if self.input_type == 'pose':
            required += ['input_base_frame', 'input_source_frame']
        for name in required:
            if not self.get_parameter(name).value:
                raise ValueError(f'Parameter "{name}" must be set')

        # Twist input
        twist_topic = self.declare_parameter('twist_topic', '/teleopro/twist').value
        twist_stamped = self.declare_parameter('twist_stamped', False).value
        twist_frame = self.declare_parameter('twist_frame', 'base').value

        workspace_min = self.declare_parameter('workspace_min', [0.0]).value
        workspace_max = self.declare_parameter('workspace_max', [0.0]).value
        self.teleop = Teleop(
            align_rpy=self.declare_parameter('align_rpy', [0.0, 0.0, 0.0]).value,
            translation_scale=self.declare_parameter('translation_scale', 1.0).value,
            rotation_scale=self.declare_parameter('rotation_scale', 1.0).value,
            max_lin_vel=self.declare_parameter('max_lin_vel', 0.5).value,
            max_ang_vel=self.declare_parameter('max_ang_vel', 1.0).value,
            dof_mask=self.declare_parameter('dof_mask', [1, 1, 1, 1, 1, 1]).value,
            # [0.0] means no limit
            workspace_min=workspace_min if len(workspace_min) == 3 else None,
            workspace_max=workspace_max if len(workspace_max) == 3 else None,
            input_timeout=self.declare_parameter('input_timeout', 0.2).value,
            twist_frame=twist_frame,
            logger=self.get_logger(),
        )
        rate = self.declare_parameter('rate', 100.0).value
        pose_topic = self.declare_parameter('pose_topic', '').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10) if pose_topic else None

        self.twist = None
        self.twist_stamp = None
        if self.input_type == 'twist':
            msg_type = TwistStamped if twist_stamped else Twist
            self.create_subscription(msg_type, twist_topic, self.twist_callback, 10)

        self.create_service(SetBool, '~/engage', self.engage_srv_callback)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.engaged_pub = self.create_publisher(Bool, '~/engaged', latched)
        self.engaged_state = None
        self.create_timer(1.0 / rate, self.tick)

        if self.input_type == 'pose':
            source = f'{self.input_base} -> {self.input_source}'
        else:
            source = twist_topic
        self.get_logger().info(
            f'Ready. input: {source}, output: {self.robot_base} -> {self.target_frame}, '
            f'engage: {self.get_fully_qualified_name()}/engage')

    def lookup(self, parent, child):
        """Look up a transform.

        Returns:
            (pose, stamp), stamp is None for static transforms. (None, None) if unavailable.
        """
        try:
            t = self.tf_buffer.lookup_transform(parent, child, Time())
        except TransformException:
            return None, None
        tr, q = t.transform.translation, t.transform.rotation
        stamp = Time.from_msg(t.header.stamp).nanoseconds * 1e-9
        return ((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w)), (stamp or None)

    def publish_target(self, T):
        (x, y, z), (qx, qy, qz, qw) = as_pos_quat(T)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.robot_base
        t.child_frame_id = self.target_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(t)

        if self.pose_pub is not None:
            msg = PoseStamped()
            msg.header = t.header
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
            msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
            msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
            self.pose_pub.publish(msg)

    def twist_callback(self, msg):
        if isinstance(msg, TwistStamped):
            stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
            msg = msg.twist
        else:
            stamp = 0.0
        # Use receive time for unstamped messages
        self.twist_stamp = stamp or self.get_clock().now().nanoseconds * 1e-9
        self.twist = ((msg.linear.x, msg.linear.y, msg.linear.z),
                      (msg.angular.x, msg.angular.y, msg.angular.z))

    def engage_srv_callback(self, request, response):
        if request.data:
            self.teleop.engage()
        else:
            self.teleop.disengage()
        response.success = True
        response.message = 'engage requested' if request.data else 'disengaged'
        return response

    def tick(self):
        robot_pose = None
        if self.teleop.needs_robot_pose:
            robot_pose, _ = self.lookup(self.robot_base, self.robot_source)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.input_type == 'pose':
            input_pose, input_stamp = self.lookup(self.input_base, self.input_source)
            target = self.teleop.update(now, input_pose, robot_pose, input_stamp)
        else:
            target = self.teleop.update_twist(now, self.twist, robot_pose, self.twist_stamp)
        if target is not None:
            self.publish_target(target)
        if self.teleop.engaged != self.engaged_state:
            self.engaged_state = self.teleop.engaged
            self.engaged_pub.publish(Bool(data=self.engaged_state))


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
