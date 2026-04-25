#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import os

# ── World z of table surface in Gazebo (metres) ──────────────────────────────
# Check in Gazebo: click the table → Pose → Z position + half table height
# Typical value for a 0.05m thick table at z=0.0 ground = 0.025
# For CoffeeTables in your world, measure from Gazebo Entity Tree → Pose
TABLE_Z = 0.78  # metres — tune this to match your Gazebo world

class FastenerDetector(Node):
    def __init__(self):
        super().__init__('fastener_detector')

        # Load YOLOv12 model
        model_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', '..', '..',
            'share', 'panda_vision', 'models', 'best.pt'
        )
        model_path = os.path.normpath(model_path)
        self.get_logger().info(f"Loading YOLOv12 model from: {model_path}")
        self.model = YOLO(model_path)
        self.class_names = self.model.names
        self.get_logger().info(f"Classes: {self.class_names}")

        # Camera intrinsics — will be updated from /camera/camera_info
        self.fx = 585.0
        self.fy = 588.0
        self.cx = 320.0
        self.cy = 160.0
        self.camera_height = None  # set from TF or hardcoded below

        # Camera pose in world frame
        # Check Gazebo: panda → panda_link0 → camera link pose
        # These are approximate — tune after checking Gazebo
        self.camera_world_x = 0.0    # camera x in world frame
        self.camera_world_y = 0.0    # camera y in world frame
        self.camera_world_z = 1.5    # camera height above ground

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Publisher — publishes ALL detected fasteners, one message each
        self.coords_pub = self.create_publisher(String, '/color_coordinates', 10)

        self.bridge = CvBridge()

        # Track published objects to avoid flooding (reset every 5 seconds)
        self.published_this_frame = set()

        self.get_logger().info("YOLOv12 Fastener Detector Node Started!")
        self.get_logger().info(
            f"Publishing ALL fastener classes to /color_coordinates")

    def camera_info_callback(self, msg):
        """Update camera intrinsics from actual camera info."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def pixel_to_world(self, u, v):
        """
        Convert pixel coordinates to world XY coordinates.
        Assumes camera is looking straight down at the table.
       
        depth = distance from camera to table surface
        x_world = camera_x + (u - cx) * depth / fx
        y_world = camera_y + (v - cy) * depth / fy
        z_world = TABLE_Z (fixed table height)
        """
        depth = self.camera_world_z - TABLE_Z
        x_world = self.camera_world_x + (u - self.cx) * depth / self.fx
        y_world = self.camera_world_y + (v - self.cy) * depth / self.fy
        return x_world, y_world, TABLE_Z

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        results = self.model(frame, verbose=False)
        self.published_this_frame = set()

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf  = float(box.conf[0])
                label = self.class_names[cls_id]

                if conf < 0.25:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx_pix = (x1 + x2) // 2
                cy_pix = (y1 + y2) // 2

                # Draw detection
                color_map = {
                    'Clip':  (255, 0,   0),
                    'Rivet': (0,   255, 0),
                    'Screw': (0,   0,   255)
                }
                draw_color = color_map.get(label, (0, 255, 255))
                cv2.rectangle(frame, (x1, y1), (x2, y2), draw_color, 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)

                # Convert to world coordinates
                x_w, y_w, z_w = self.pixel_to_world(cx_pix, cy_pix)

                # model_name matches Gazebo entity name
                # e.g. "green_box", "red_box", "blue_box" from your world
                model_name_map = {
                    'Clip':  'blue_box',
                    'Rivet': 'red_box',
                    'Screw': 'green_box',
                }
                model_name = model_name_map.get(label, label.lower())

                # Publish: "Class,x,y,z,model_name"
                msg_str = (f"{label},"
                          f"{x_w:.4f},"
                          f"{y_w:.4f},"
                          f"{z_w:.4f},"
                          f"{model_name}")

                self.coords_pub.publish(String(data=msg_str))
                self.get_logger().info(
                    f"Published: {msg_str}  (conf={conf:.2f}  "
                    f"pixel=[{cx_pix},{cy_pix}])")

        # Show window
        try:
            cv2.namedWindow("YOLOv12 Fastener Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("YOLOv12 Fastener Detection", 640, 320)
            cv2.imshow("YOLOv12 Fastener Detection", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"Display error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FastenerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
