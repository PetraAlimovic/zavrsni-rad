import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from std_msgs.msg import String
from hand_teleop_msgs.msg import HandPose

import pyrealsense2 as rs
import mediapipe as mp
import numpy as np
import cv2
import time

from sensor_msgs.msg import CompressedImage

class HandCaptureNode(Node):

    def __init__(self):
        super().__init__('hand_capture_node')

        # CONFIG
        self.PUBLISH_RATE           = 20    # Hz
        self.MIN_CONFIDENCE         = 0.6
        self.DEBOUNCE_FRAMES        = 5
        self.FIST_CURL_THRESHOLD    = 0.8
        self.CALIBRATION_DURATION   = 3.0   # seconds

        # STATE
        self.confirmed_gesture  = 'OPEN'
        self.pending_gesture    = 'OPEN'
        self.pending_count      = 0
        self.latest_position    = None
        self.latest_confidence  = 0.0
        self.last_known_position = Point()

        # NULL POSITION & CALIBRATION STATE
        self.null_position          = None
        self.is_calibrating         = False
        self.calibration_start_time = 0.0

        # ROS PUBLISHER
        self.publisher = self.create_publisher(HandPose, '/hand_pose', 10)
        self.null_pub = self.create_publisher(Point, '/null_position', 10)

        self.robot_image = None
        self.create_subscription(CompressedImage, '/camera/camera/color/image_raw/compressed', self.robot_camera_callback, 10)

        self.obstacle_status = "Idle — operator in control"
        self.create_subscription(String, '/obstacle_status', self.obstacle_status_callback, 10)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # CAM INTRINSICS & ALIGNMENT
        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        self.intrinsics = depth_profile.get_intrinsics()
        self.align = rs.align(rs.stream.color)

        # MEDIAPIPE HANDS & DRAWING UTILS
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=self.MIN_CONFIDENCE,
            min_tracking_confidence=self.MIN_CONFIDENCE
        )

        # TIMERS
        self.create_timer(1.0 / self.PUBLISH_RATE, self.publish_latest_state)
        self.create_timer(1.0 / 30.0, self.camera_loop)

        self.get_logger().info('Hand capture node started with GUI Dashboard')


    # CAMERA & GUI LOOP
    def obstacle_status_callback(self, msg):
        self.obstacle_status = msg.data

    def robot_camera_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.robot_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Robot camera decode error: {e}')

    def camera_loop(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        aligned = self.align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            self.debounce_gesture('OPEN')
            return

        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Create depth colormap for GUI visualization
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )

        # Process MediaPipe (RGB)
        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_image)

        if not results.multi_hand_landmarks:
            self.debounce_gesture('OPEN')
            self.latest_confidence = 0.0
            self.render_gui(color_image, depth_colormap, hand_detected=False)
            return

        raw_landmarks = results.multi_hand_landmarks[0]

        # Draw hand markers on RGB feed
        self.mp_drawing.draw_landmarks(
            color_image,
            raw_landmarks,
            self.mp_hands.HAND_CONNECTIONS,
            self.mp_drawing_styles.get_default_hand_landmarks_style(),
            self.mp_drawing_styles.get_default_hand_connections_style()
        )

        # Confidence
        if results.multi_handedness:
            self.latest_confidence = results.multi_handedness[0].classification[0].score
        else:
            self.latest_confidence = 1.0

        if self.latest_confidence < self.MIN_CONFIDENCE:
            self.debounce_gesture('OPEN')
            self.render_gui(color_image, depth_colormap, hand_detected=False)
            return

        # Gesture Classification
        raw_gesture = self.classify_gesture(raw_landmarks)
        self.debounce_gesture(raw_gesture)

        # 3D Position Deprojection
        h, w = color_image.shape[:2]
        landmarks_3d = self.deproject_landmarks(raw_landmarks, depth_frame, w, h)
        raw_position = self.compute_palm_center(landmarks_3d)

        if raw_position is not None:
            self.latest_position = raw_position
            self.last_known_position = raw_position

        # Update Calibration State Logic
        self.update_calibration()

        # Render GUI
        self.render_gui(color_image, depth_colormap, hand_detected=True)


    # GESTURE & DEPROJECTION
    def classify_gesture(self, raw_landmarks):
        knuckle_indices = [5, 9, 13, 17]
        palm_x = float(np.mean([raw_landmarks.landmark[i].x for i in knuckle_indices]))
        palm_y = float(np.mean([raw_landmarks.landmark[i].y for i in knuckle_indices]))
        palm_z = float(np.mean([raw_landmarks.landmark[i].z for i in knuckle_indices]))

        finger_pairs = [(8, 6), (12, 10), (16, 14), (20, 18)]
        curled = 0

        for tip_idx, pip_idx in finger_pairs:
            tip = raw_landmarks.landmark[tip_idx]
            pip = raw_landmarks.landmark[pip_idx]

            tip_dist = np.sqrt((tip.x - palm_x)**2 + (tip.y - palm_y)**2 + (tip.z - palm_z)**2)
            pip_dist = np.sqrt((pip.x - palm_x)**2 + (pip.y - palm_y)**2 + (pip.z - palm_z)**2)

            if pip_dist > 0 and tip_dist < pip_dist * self.FIST_CURL_THRESHOLD:
                curled += 1

        if curled >= 3:
            return 'FIST'
        elif curled <= 1:
            return 'OPEN'
        else:
            return 'UNKNOWN'

    def deproject_landmarks(self, landmarks, depth_frame, w, h):
        points_3d = []
        for lm in landmarks.landmark:
            px = max(0, min(int(lm.x * w), w - 1))
            py = max(0, min(int(lm.y * h), h - 1))
            depth = depth_frame.get_distance(px, py)
            if depth == 0:
                points_3d.append(None)
                continue
            point = rs.rs2_deproject_pixel_to_point(self.intrinsics, [px, py], depth)
            p = Point()
            p.x, p.y, p.z = float(point[0]), float(point[1]), float(point[2])
            points_3d.append(p)
        return points_3d

    def compute_palm_center(self, landmarks_3d):
        knuckle_indices = [5, 9, 13, 17]
        valid = [landmarks_3d[i] for i in knuckle_indices
                 if i < len(landmarks_3d) and landmarks_3d[i] is not None]
        if not valid:
            return None
        p = Point()
        p.x = float(np.mean([v.x for v in valid]))
        p.y = float(np.mean([v.y for v in valid]))
        p.z = float(np.mean([v.z for v in valid]))
        return p

    def debounce_gesture(self, raw_gesture):
        if raw_gesture == self.pending_gesture:
            self.pending_count += 1
        else:
            self.pending_gesture = raw_gesture
            self.pending_count = 1
        if self.pending_count >= self.DEBOUNCE_FRAMES:
            self.confirmed_gesture = self.pending_gesture


    # CALIBRATION LOGIC
    def trigger_calibration(self):
        self.is_calibrating = True
        self.calibration_start_time = time.time()
        self.get_logger().info("Calibration countdown started (3 seconds)...")

    def update_calibration(self):
        if not self.is_calibrating:
            return

        elapsed = time.time() - self.calibration_start_time
        if elapsed >= self.CALIBRATION_DURATION:
            if self.latest_position is not None:
                self.null_position = Point()
                self.null_position.x = self.latest_position.x
                self.null_position.y = self.latest_position.y
                self.null_position.z = self.latest_position.z
                self.get_logger().info(
                    f"Null position set to: ({self.null_position.x:.2f}, "
                    f"{self.null_position.y:.2f}, {self.null_position.z:.2f})"
                )
                self.null_pub.publish(self.null_position)
            self.is_calibrating = False


    # GUI RENDERER
    def render_gui(self, color_image, depth_colormap, hand_detected):
        # Gesture Color
        gesture_colors = {
            'OPEN': (0, 255, 0),
            'FIST': (0, 0, 255),
            'UNKNOWN': (0, 255, 255)
        }
        g_color = gesture_colors.get(self.confirmed_gesture, (255, 255, 255))

        # Top Overlay Panel
        cv2.rectangle(color_image, (0, 0), (640, 110), (0, 0, 0), -1)

        # Gesture & Confidence
        cv2.putText(color_image, f"GESTURE: {self.confirmed_gesture}", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, g_color, 2)
        cv2.putText(color_image, f"Confidence: {self.latest_confidence * 100:.1f}%", (15, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Distance from Null Position
        if self.null_position is not None and self.latest_position is not None:
            dx = self.latest_position.x - self.null_position.x
            dy = self.latest_position.y - self.null_position.y
            dz = self.latest_position.z - self.null_position.z
            dist_3d = np.sqrt(dx**2 + dy**2 + dz**2)

            cv2.putText(color_image, f"Null Offset: {dist_3d*100:.1f} cm", (15, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(color_image, f"dX:{dx*100:+.1f}  dY:{dy*100:+.1f}  dZ:{dz*100:+.1f} (cm)", (15, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        else:
            cv2.putText(color_image, "Null Position: UNCALIBRATED (Press 'C')", (15, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)

        # Calibration Countdown Overlay
        if self.is_calibrating:
            remaining = self.CALIBRATION_DURATION - (time.time() - self.calibration_start_time)
            if remaining > 0:
                cv2.rectangle(color_image, (80, 200), (560, 280), (0, 0, 0), -1)
                cv2.putText(color_image, f"CALIBRATING NULL POS IN {remaining:.1f}s", (100, 235),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(color_image, "Hold hand steady at resting position...", (115, 260),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Help overlay
        cv2.putText(color_image, "Press 'C': Calibrate Null | 'Q': Exit", (10, 470),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # 1. Robot Camera Panel (Bottom Left)
        if self.robot_image is not None:
            # Resize the robot feed to match the 640x480 resolution of the hand camera
            robot_cam_resized = cv2.resize(self.robot_image, (640, 480))
        else:
            robot_cam_resized = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(robot_cam_resized, "NO ROBOT CAM FEED YET", (130, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(robot_cam_resized, "(Check topic name)", (220, 270),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        # 2. Info Panel (Bottom Right)
        info_panel = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Shifted original text slightly up
        cv2.putText(info_panel, "ASTRO Robot Teleop Active", (120, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Safely get the status (defaults to idle if not yet initialized in __init__)
        current_status = getattr(self, 'obstacle_status', 'Idle — operator in control')

        # Determine background color based on status severity
        if "OBSTACLE DETECTED" in current_status or "UNABLE" in current_status:
            banner_color = (0, 0, 180)     # Red
        elif any(keyword in current_status for keyword in ["Avoiding", "Backing", "Turning", "Following", "Safe"]):
            banner_color = (0, 140, 255)   # Orange/Yellow
        else:
            banner_color = (0, 150, 0)     # Green

        # Draw the Obstacle Feedback Banner
        cv2.rectangle(info_panel, (20, 200), (620, 320), banner_color, -1)
        cv2.putText(info_panel, "OBSTACLE AVOIDANCE SYSTEM:", (40, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(info_panel, current_status, (40, 285),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 3. Combine into a 2x2 grid (1280x960 total resolution)
        top_row = np.hstack((color_image, depth_colormap))
        bottom_row = np.hstack((robot_cam_resized, info_panel))
        dashboard = np.vstack((top_row, bottom_row))

        # Show Window
        cv2.imshow("Hand Teleop Dashboard", dashboard)
        
        # Keyboard Inputs
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c') or key == ord('C'):
            self.trigger_calibration()
        elif key == ord('q') or key == ord('Q') or key == 27:  # Esc / Q
            rclpy.shutdown()


    # ROS 2 PUB & CLEANUP
    def publish_latest_state(self):
        msg = HandPose()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.gesture = self.confirmed_gesture
        msg.confidence = float(self.latest_confidence)
        msg.position = self.latest_position \
            if self.latest_position is not None \
            else self.last_known_position
        self.publisher.publish(msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandCaptureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()