import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from hand_teleop_msgs.msg import HandPose
import math

class HandTeleopNode(Node):

    def __init__(self):
        super().__init__('hand_teleop_node')

        # CONFIG / PARAMETERS
        self.declare_parameter('deadzone_radius', 0.05)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('map_range_linear', 0.20)
        self.declare_parameter('map_range_angular', 0.20)
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('tracking_timeout', 0.4)
        self.declare_parameter('publish_rate', 20.0)

        self.declare_parameter('stability_window_size', 15)
        self.declare_parameter('stability_threshold', 0.015)
        self.declare_parameter('stability_hold_duration', 1.0)
        self.declare_parameter('averaging_sample_count', 30)
        self.declare_parameter('min_landmark_confidence', 0.6)

        # Load parameter values
        self.deadzone_radius = self.get_parameter('deadzone_radius').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.map_range_linear = self.get_parameter('map_range_linear').value
        self.map_range_angular = self.get_parameter('map_range_angular').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.tracking_timeout = self.get_parameter('tracking_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        self.stability_window_size = self.get_parameter('stability_window_size').value
        self.stability_threshold = self.get_parameter('stability_threshold').value
        self.stability_hold_duration = self.get_parameter('stability_hold_duration').value
        self.averaging_sample_count = self.get_parameter('averaging_sample_count').value
        self.min_landmark_confidence = self.get_parameter('min_landmark_confidence').value
        
        # STATE
        self.neutral_position = None
        self.last_msg_time = None
        self.latest_hand_pos = None
        self.latest_gesture = "OPEN"
        self.filtered_linear_x = 0.0
        self.filtered_angular_z = 0.0
        self.calibration_active = False



        # ON_START
        self.sub_pose = self.create_subscription(
            HandPose, 
            '/hand_pose', 
            self.on_hand_pose_received, 
            10
        )
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel_hand', 10)
        self.create_timer(1.0 / self.publish_rate, self.publish_velocity)
        
        self.create_subscription(Point, '/null_position', self.on_null_position, 10)

    def on_null_position(self, msg: Point):
        self.neutral_position = msg
        self.get_logger().info(
            f"Neutral position received: X:{msg.x:.3f}, Y:{msg.y:.3f}, Z:{msg.z:.3f}"
        )

    # RUNTIME

    def on_hand_pose_received(self, msg: HandPose):
        self.last_msg_time = self.get_clock().now()
        self.latest_hand_pos = msg.position
        self.latest_gesture = msg.gesture


    def publish_velocity(self):
        if self.calibration_active or self.neutral_position is None:
            self.publish_zero_twist()
            return

        if self.last_msg_time is None:
            self.publish_zero_twist()
            return

        now = self.get_clock().now()
        time_since_last_msg = (now - self.last_msg_time).nanoseconds / 1e9

        # Safety Stop: Hand lost or camera dropout
        if time_since_last_msg > self.tracking_timeout:
            self.publish_zero_twist()
            return

        # Deadman Switch: Requires holding a FIST to drive
        if self.latest_gesture != "FIST":
            self.publish_zero_twist()
            return

        # Offset from calibrated neutral position
        dx = self.latest_hand_pos.x - self.neutral_position.x
        dz = self.neutral_position.z - self.latest_hand_pos.z

        # Radial deadzone
        if math.hypot(dx, dz) < self.deadzone_radius:
            dx, dz = 0.0, 0.0

        # Normalize and clamp (-1 to 1)
        norm_forward = max(-1.0, min(1.0, dz / self.map_range_linear))
        norm_turn = max(-1.0, min(1.0, dx / self.map_range_angular))

        # Scaling to robot limits
        target_linear_x = norm_forward * self.max_linear_speed
        target_angular_z = norm_turn * self.max_angular_speed

        # Smoothing Filter
        self.filtered_linear_x = (self.smoothing_alpha * target_linear_x) + \
                                 ((1.0 - self.smoothing_alpha) * self.filtered_linear_x)
        self.filtered_angular_z = (self.smoothing_alpha * target_angular_z) + \
                                  ((1.0 - self.smoothing_alpha) * self.filtered_angular_z)

        # Publish
        twist = Twist()
        twist.linear.x = float(self.filtered_linear_x)
        twist.angular.z = float(self.filtered_angular_z)
        self.pub_cmd_vel.publish(twist)

    def publish_zero_twist(self):
        # Reset filter state to prevent sudden jump on re-engagement
        self.filtered_linear_x = 0.0
        self.filtered_angular_z = 0.0
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = HandTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero_twist()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()