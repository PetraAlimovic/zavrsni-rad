import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster


class DummyOdomNode(Node):

    def __init__(self):
        super().__init__('dummy_odom_node')

        # Sub to vel commands
        self.create_subscription(
            Twist, '/cmd_vel_hand', self.cmd_vel_callback, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot Pose State in Odom Frame
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.wz = 0.0

        self.last_time = self.get_clock().now()
        self.create_timer(0.05, self.update_and_publish)

        self.get_logger().info('Dummy Odom Publisher started!')

    def cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def update_and_publish(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Kinematic integration
        delta_x = self.vx * math.cos(self.th) * dt
        delta_y = self.vx * math.sin(self.th) * dt
        delta_th = self.wz * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Broadcast odom to base_footprint transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'  # ASTRO root frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Yaw to Quaternion conversion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DummyOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()