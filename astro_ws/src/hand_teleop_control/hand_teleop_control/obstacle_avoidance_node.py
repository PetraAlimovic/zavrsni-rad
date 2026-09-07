#!/usr/bin/env python3
import math
import time
from collections import deque

import numpy as np
import rclpy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped transform support
from geometry_msgs.msg import Twist, PointStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class State:
    IDLE       = 'IDLE'
    STOPPING   = 'STOPPING'
    BACKING_UP = 'BACKING_UP'
    SCANNING   = 'SCANNING'
    FOLLOWING  = 'FOLLOWING'
    UNABLE     = 'UNABLE_TO_AVOID'
    NUDGE_FORWARD = 'NUDGE_FORWARD'


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('obstacle_threshold',  0.3)    # m — trigger detection
        self.declare_parameter('safe_distance',       0.5)    # m — target after backup
        self.declare_parameter('scan_threshold',      0.8)    # m — range threshold to measure extent
        self.declare_parameter('target_wall_dist',    0.3)    # m — wall following gap
        self.declare_parameter('linear_speed',        0.15)   # m/s
        self.declare_parameter('turn_speed',          0.5)    # rad/s
        self.declare_parameter('backup_speed',        0.12)   # m/s
        self.declare_parameter('k_angle',             1.2)    # proportional gain — heading
        self.declare_parameter('k_dist',              1.5)    # proportional gain — distance
        self.declare_parameter('backup_timeout',      5.0)    # s
        self.declare_parameter('follow_timeout',     30.0)    # s
        self.declare_parameter('unable_hold_time',    3.0)    # s
        self.declare_parameter('min_follow_time',     0.8)    # s before corner detection active
        self.declare_parameter('corner_angle_deg',   60.0)    # deg — turn angle corner threshold (fallback method)
        self.declare_parameter('lidar_front_offset', 180.0)   # deg — mounting offset

        # ── TF-based corner tracking ─────────────────────────────────────────
        self.declare_parameter('use_tf_corner_tracking',  True)      # prefer TF over angle heuristic
        self.declare_parameter('odom_frame',              'odom')    # fixed frame the corner point is stored in
        self.declare_parameter('base_frame',              'base_footprint')  # robot frame — verify against your TF tree
        self.declare_parameter('corner_pass_clearance',   0.2)      # m — how far past the corner before turning back
        self.declare_parameter('corner_clear_front_dist', 0.9)      # m — required clear path ahead to finish

        gp = lambda name: self.get_parameter(name).value

        self.obstacle_threshold     = gp('obstacle_threshold')
        self.safe_distance          = gp('safe_distance')
        self.scan_threshold         = gp('scan_threshold')
        self.target_wall_dist       = gp('target_wall_dist')
        self.linear_speed           = gp('linear_speed')
        self.turn_speed             = gp('turn_speed')
        self.backup_speed           = gp('backup_speed')
        self.k_angle                = gp('k_angle')
        self.k_dist                 = gp('k_dist')
        self.backup_timeout         = gp('backup_timeout')
        self.follow_timeout         = gp('follow_timeout')
        self.unable_hold_time       = gp('unable_hold_time')
        self.min_follow_time        = gp('min_follow_time')
        self.corner_angle_threshold = math.radians(gp('corner_angle_deg'))
        self.lidar_offset           = math.radians(gp('lidar_front_offset'))

        self.use_tf_corner_tracking  = gp('use_tf_corner_tracking')
        self.odom_frame              = gp('odom_frame')
        self.base_frame              = gp('base_frame')
        self.corner_pass_clearance   = gp('corner_pass_clearance')
        self.corner_clear_front_dist = gp('corner_clear_front_dist')

        # ── State machine ────────────────────────────────────────────────────
        self.state            = State.IDLE
        self.state_start_time = time.time()
        self.follow_side      = 'RIGHT'
        self.travel_direction = 1
        self.obstacle_angle   = 0.0
        self.status_msg       = 'Idle — operator in control'

        # Corner tracking state (angle-heuristic fallback)
        self.net_turn_rad = 0.0
        self.aligned_with_wall    = False
        self.wall_angle_history   = deque(maxlen=10)

        # Corner tracking state (TF-based) — PointStamped in odom_frame, or
        # None if not yet captured / TF unavailable for this encounter.
        self.corner_point_odom = None

        self.latest_scan = None
        self.latest_cmd  = Twist()

        # ── TF ───────────────────────────────────────────────────────────────
        # Requires tf2_ros / tf2_geometry_msgs / geometry_msgs as package deps.
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── ROS interfaces ───────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',         self.on_scan,    10)
        self.create_subscription(Twist,     '/cmd_vel_hand', self.on_cmd_vel, 10)
        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel_obstacle', 10)
        self.status_pub = self.create_publisher(String, '/obstacle_status',  10)

        self.create_timer(0.05, self.update)
        self.get_logger().info('Obstacle avoidance node started')

    def on_scan(self, msg):
        self.latest_scan = msg

    def on_cmd_vel(self, msg):
        self.latest_cmd = msg

    def _get_closest_point(self, angle_min_deg, angle_max_deg):
        """Return (min_range, angle) of the closest valid reading in a sector."""
        if self.latest_scan is None:
            return float('inf'), 0.0

        scan   = self.latest_scan
        ranges = np.array(scan.ranges, dtype=np.float32)
        n      = len(ranges)

        raw_angles   = scan.angle_min + np.arange(n) * scan.angle_increment
        robot_angles = (raw_angles - self.lidar_offset + np.pi) % (2 * np.pi) - np.pi

        a_min = math.radians(angle_min_deg)
        a_max = math.radians(angle_max_deg)

        if a_min <= a_max:
            sector_mask = (robot_angles >= a_min) & (robot_angles <= a_max)
        else:
            sector_mask = (robot_angles >= a_min) | (robot_angles <= a_max)

        valid_mask = (
            sector_mask &
            np.isfinite(ranges) &
            (ranges >= scan.range_min) &
            (ranges <= scan.range_max)
        )

        valid_ranges = ranges[valid_mask]
        valid_angles = robot_angles[valid_mask]

        if len(valid_ranges) == 0:
            return float('inf'), 0.0

        min_idx = np.argmin(valid_ranges)
        return float(valid_ranges[min_idx]), float(valid_angles[min_idx])

    def _get_angular_extent_and_point(self, start_deg, end_deg):
        """
        Sweeps from start_deg to end_deg outwards from 0°.
        Returns (max_extent_deg, range_m, angle_rad) for the farthest-out
        angle where an obstacle (range < scan_threshold) is still detected —
        this is the obstacle's near corner on that side. range_m/angle_rad
        are None if no obstacle was found anywhere in the sector.
        """
        step = 2 if end_deg > start_deg else -2
        stop = end_deg + step
        max_extent   = 0.0
        corner_range = None
        corner_angle = None

        for angle_deg in range(int(start_deg), int(stop), int(step)):
            dist, meas_angle = self._get_closest_point(angle_deg - 2, angle_deg + 2)
            if dist < self.scan_threshold:
                max_extent   = float(abs(angle_deg - start_deg))
                corner_range = dist
                corner_angle = meas_angle

        return max_extent, corner_range, corner_angle

    def _capture_corner_point(self, range_m, angle_rad):
        """
        Projects the obstacle's near-corner (range, angle) reading — given in
        the robot base frame — into a fixed frame (odom_frame) via TF, and
        stores it. This gives state_following a world-anchored point to
        measure progress against, instead of relying only on the live scan.

        Assumes the lidar origin is close enough to base_frame's origin that
        the range/angle pair can be treated as already being in that frame.
        If your lidar has a meaningful mounting offset, transform from the
        scan's own header.frame_id instead for better accuracy.
        """
        self.corner_point_odom = None

        if not self.use_tf_corner_tracking or range_m is None:
            return

        point = PointStamped()
        point.header.frame_id = self.base_frame
        point.header.stamp    = Time().to_msg()
        point.point.x = range_m * math.cos(angle_rad)
        point.point.y = range_m * math.sin(angle_rad)
        point.point.z = 0.0

        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time())
            self.corner_point_odom = tf2_geometry_msgs.do_transform_point(point, transform)
            self.get_logger().info(
                f'Corner point captured in "{self.odom_frame}": '
                f'({self.corner_point_odom.point.x:.2f}, '
                f'{self.corner_point_odom.point.y:.2f})')
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(f'Could not capture corner point via TF: {ex}')
            self.corner_point_odom = None

    def _corner_point_passed(self):
        """
        Transforms the stored corner point from odom_frame into the robot's
        current base_frame. Once its x-coordinate is behind the robot by more
        than corner_pass_clearance, the robot has physically driven past the
        corner with margin to spare.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.odom_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(f'TF unavailable for corner tracking: {ex}')
            return False

        point_in_base = tf2_geometry_msgs.do_transform_point(self.corner_point_odom, transform)
        return point_in_base.point.x < -self.corner_pass_clearance

    def update(self):
        if self.latest_scan is None:
            return

        {
            State.IDLE:       self.state_idle,
            State.STOPPING:   self.state_stopping,
            State.BACKING_UP: self.state_backing_up,
            State.SCANNING:   self.state_scanning,
            State.FOLLOWING:  self.state_following,
            State.UNABLE:     self.state_unable,
            State.NUDGE_FORWARD: self.state_nudge_forward,
        }[self.state]()

        msg      = String()
        msg.data = self.status_msg
        self.status_pub.publish(msg)

    def state_idle(self):
        linear  = self.latest_cmd.linear.x
        angular = self.latest_cmd.angular.z

        if abs(linear) < 0.01 and abs(angular) < 0.01:
            self.status_msg = 'Idle — operator in control'
            return

        if linear >= 0:
            min_dist, angle = self._get_closest_point(-35.0, 35.0)
        else:
            min_dist, angle = self._get_closest_point(145.0, -145.0)

        self.status_msg = 'Idle — operator in control'

        if min_dist < self.obstacle_threshold:
            self.travel_direction = 1 if linear >= 0 else -1
            self.obstacle_angle   = angle

            if self.travel_direction == -1:
            # Rear obstacle — don't run avoidance, just creep forward to clear it
                self.status_msg = f'REAR OBSTACLE at {min_dist:.2f}m — nudging forward'
                self._transition(State.NUDGE_FORWARD)
            else:
                self.status_msg = f'OBSTACLE DETECTED at {min_dist:.2f}m'
                self._transition(State.STOPPING)

    def state_stopping(self):
        self._publish(0.0, 0.0)
        self.status_msg = 'Obstacle detected — stopping'
        self._transition(State.BACKING_UP)

    def state_backing_up(self):
        elapsed = self._elapsed()

        if elapsed > self.backup_timeout:
            self.status_msg = 'UNABLE TO AVOID: backup timed out'
            self._transition(State.UNABLE)
            return

        if self.travel_direction >= 0:
            check_dist, _ = self._get_closest_point(-90.0, 90.0)
        else:
            check_dist, _ = self._get_closest_point(90.0, -90.0)

        if check_dist >= self.safe_distance:
            self._publish(0.0, 0.0)
            self.status_msg = 'Safe distance reached — scanning directions'
            self._transition(State.SCANNING)
            return

        self.status_msg = f'Backing up... {check_dist:.2f}m / {self.safe_distance:.2f}m'
        self._publish(-self.backup_speed * self.travel_direction, 0.0)
    

    def state_nudge_forward(self):
        """Rear obstacle — move forward slowly until clear, then return control."""
        rear_dist, _ = self._get_closest_point(90.0, -90.0)
        elapsed = self._elapsed()

        if rear_dist >= self.safe_distance or elapsed > 3.0:
            self._publish(0.0, 0.0)
            self.status_msg = 'Rear cleared — operator in control'
            self._transition(State.IDLE)
            return

        self.status_msg = f'Rear obstacle ({rear_dist:.2f}m) — nudging forward'
        self._publish(self.backup_speed, 0.0)

    
    def state_scanning(self):
    # Find where the obstacle actually is after backing up (not where it was at detection)
        _, center_angle = self._get_closest_point(-90.0, 90.0)
        center_deg = max(-60.0, min(60.0, math.degrees(center_angle)))

        # Sweep outward from obstacle center on each side
        left_end  = min(center_deg + 90.0,  90.0)
        right_end = max(center_deg - 90.0, -90.0)

        left_extent,  left_range,  left_angle  = self._get_angular_extent_and_point(center_deg, left_end)
        right_extent, right_range, right_angle = self._get_angular_extent_and_point(center_deg, right_end)

        self.get_logger().info(
            f'Scan (center at {center_deg:.1f}°) — '
            f'Left: {left_extent:.1f}°  Right: {right_extent:.1f}°')

        if left_extent < right_extent:
            # Obstacle ends sooner on LEFT -> steer LEFT -> wall will be on RIGHT
            self.follow_side = 'RIGHT'
            corner_range, corner_angle = left_range, left_angle
            self.get_logger().info('Shorter on LEFT -> Steering LEFT (wall on RIGHT)')
        elif right_extent < left_extent:
            # Obstacle ends sooner on RIGHT -> steer RIGHT -> wall will be on LEFT
            self.follow_side = 'LEFT'
            corner_range, corner_angle = right_range, right_angle
            self.get_logger().info('Shorter on RIGHT -> Steering RIGHT (wall on LEFT)')
        else:
            # Equal extent tiebreaker based on initial obstacle angle offset
            if self.obstacle_angle >= 0:
                self.follow_side = 'RIGHT'
                corner_range, corner_angle = left_range, left_angle
            else:
                self.follow_side = 'LEFT'
                corner_range, corner_angle = right_range, right_angle
            self.get_logger().info(f'Equal extents -> Tiebreaker wall side: {self.follow_side}')

        self._capture_corner_point(corner_range, corner_angle)

        self.status_msg = f'Avoiding — wall on {self.follow_side}'
        self._transition(State.FOLLOWING)

    def state_following(self):
        elapsed = self._elapsed()
        dt = 0.05  # timer period (20 Hz)

        if elapsed > self.follow_timeout:
            self.status_msg = 'UNABLE TO AVOID: obstacle appears endless'
            self._transition(State.UNABLE)
            return

        # Target wall side sector setup
        if self.follow_side == 'LEFT':
            dist, angle   = self._get_closest_point(-45.0, 100.0)
            desired_angle = math.pi / 2.0     # wall at +90°
        else:
            dist, angle   = self._get_closest_point(-100.0, 45.0)
            desired_angle = -math.pi / 2.0    # wall at -90°

        # Front crash prevention
        front_dist, _ = self._get_closest_point(-30.0, 30.0)
        if front_dist < self.obstacle_threshold and elapsed > 1.0:
            self.status_msg = 'New obstacle while following — backing up'
            self._transition(State.BACKING_UP)
            return

        # Proportional control
        angle_error = math.atan2(
            math.sin(angle - desired_angle),
            math.cos(angle - desired_angle))
        dist_error = dist - self.target_wall_dist

        if self.follow_side == 'RIGHT':
            angular_z = (self.k_angle * angle_error) - (self.k_dist * dist_error)
        else:
            angular_z = (self.k_angle * angle_error) + (self.k_dist * dist_error)

        angular_z = max(-self.turn_speed, min(self.turn_speed, angular_z))

        # Check wall alignment status
        if not self.aligned_with_wall:
            if abs(dist_error) < 0.15 and abs(angle_error) < math.radians(20.0):
                self.aligned_with_wall = True
                self.get_logger().info('Aligned with wall — turn tracking active')

        # Keep the turn-accumulation running regardless of which corner check
        # ends up being used below — it's cheap, and it's needed as a ready
        # fallback if TF ever becomes unavailable mid-encounter.
        wall_angle_shift = 0.0
        if elapsed > self.min_follow_time and self.aligned_with_wall:
            self.net_turn_rad += angular_z * dt
            self.wall_angle_history.append(angle)

            if len(self.wall_angle_history) >= 2:
                oldest = self.wall_angle_history[0]
                newest = self.wall_angle_history[-1]
                wall_angle_shift = abs(math.atan2(
                    math.sin(newest - oldest),
                    math.cos(newest - oldest)))

        # ── Corner clearance check ──────────────────────────────────────────
        # Prefer the TF-anchored check (it knows the robot actually drove
        # past the physical corner with clearance); fall back to the angle
        # heuristic if TF wasn't available when the corner was captured.
        corner_cleared = False

        if self.use_tf_corner_tracking and self.corner_point_odom is not None:
            if self._corner_point_passed() and front_dist > self.corner_clear_front_dist:
                self.get_logger().info(
                    f'Corner cleared by >{self.corner_pass_clearance:.2f}m, path ahead clear')
                corner_cleared = True
        elif elapsed > self.min_follow_time and self.aligned_with_wall:
            if (abs(self.net_turn_rad) >= self.corner_angle_threshold or
                    wall_angle_shift >= self.corner_angle_threshold):
                self.get_logger().info(
                    f'Corner detected (>=60° turn): accumulated='
                    f'{math.degrees(self.net_turn_rad):.1f}°, '
                    f'shift={math.degrees(wall_angle_shift):.1f}°')
                corner_cleared = True

        if corner_cleared:
            self._publish(0.0, 0.0)
            self.status_msg = 'Obstacle avoided — operator in control'
            self._transition(State.IDLE)
            return

        # Linear speed controller
        if dist < 0.25:
            linear_x = -self.backup_speed
            self.status_msg = f'TOO CLOSE ({dist:.2f}m) — backing away'
        elif abs(angle) < (math.pi / 3.0) and dist < (self.target_wall_dist + 0.15):
            linear_x = 0.0
            self.status_msg = f'Pivoting to align with {self.follow_side} wall'
        else:
            linear_x = self.linear_speed
            turn_deg = math.degrees(abs(self.net_turn_rad))
            self.status_msg = (f'Following {self.follow_side} wall — dist: {dist:.2f}m, '
                               f'turn: {turn_deg:.1f}°/60°')

        self._publish(linear_x, angular_z)

    def state_unable(self):
        self._publish(0.0, 0.0)
        if self._elapsed() > self.unable_hold_time:
            self.status_msg = 'Avoidance failed — operator in control'
            self._transition(State.IDLE)

    def _transition(self, new_state):
        self.get_logger().info(f'[OA] {self.state} -> {new_state}')
        self.state            = new_state
        self.state_start_time = time.time()

        if new_state == State.FOLLOWING:
            self.net_turn_rad = 0.0
            self.aligned_with_wall    = False
            self.wall_angle_history.clear()

        if new_state == State.STOPPING:
            # Fresh obstacle encounter — discard any stale corner point.
            self.corner_point_odom = None

    def _elapsed(self):
        return time.time() - self.state_start_time

    def _publish(self, linear_x, angular_z):
        twist           = Twist()
        twist.linear.x  = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)

    def destroy_node(self):
        self._publish(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()