import socket
import struct
import numpy as np
import cv2
from ultralytics import YOLO
import csv
import time
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion
import math

def extend_line_by_percentage(x1, y1, x2, y2, extension_percent):
    """
    Extend a line segment on both ends by a specified percentage of its original length.

    Args:
        x1 (float): Starting point x-coordinate.
        y1 (float): Starting point y-coordinate.
        x2 (float): Ending point x-coordinate.
        y2 (float): Ending point y-coordinate.
        extension_percent (float): Percentage (e.g., 0.3 for 30%) by which to extend the line on each end.

    Returns:
        tuple: New coordinates (new_x1, new_y1, new_x2, new_y2) of the extended line.
    """

    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx**2 + dy**2)
    if length == 0:
        return x1, y1, x2, y2
    extension = length * extension_percent
    ux = dx / length
    uy = dy / length
    new_x1 = x1 - ux * extension
    new_y1 = y1 - uy * extension
    new_x2 = x2 + ux * extension
    new_y2 = y2 + uy * extension
    return new_x1, new_y1, new_x2, new_y2

class GlassMarkerPublisher(Node):
    """
    ROS2 node for publishing detected glass lines as MarkerArray messages in RViz.

    Attributes:
        publisher (Publisher): Publisher for MarkerArray messages.
        tf_buffer (Buffer): Buffer for TF transformations.
        tf_listener (TransformListener): Listener for TF updates.
        marker_id (int): Incremental ID for each marker to avoid collisions in RViz.
    """
    def __init__(self):
        super().__init__('glass_marker_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'glass_markers', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_id = 0

    def publish_glass_markers(self, line_segments):
        """
        Publish a set of glass line segments as a single MarkerArray to RViz.

        Args:
            line_segments (list): List of tuples containing Point objects representing line endpoints.
        """
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.id = self.marker_id
        self.marker_id += 1
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for pt1, pt2 in line_segments:
            marker.points.append(pt1)
            marker.points.append(pt2)

        marker_array.markers.append(marker)
        self.publisher.publish(marker_array)

    def get_robot_pose(self):
        """
        Get the robot's current pose in the 'map' frame.

        Returns:
            tuple: (x, y, yaw) where x and y are in meters, and yaw is in radians.
                   Returns None if the transform is not yet available.
        """
        now_ros = rclpy.time.Time()
        try:
            if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)):
                self.get_logger().warn("Transform not yet available: map → base_link")
                return None

            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            trans = transform.transform.translation
            rot = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            return trans.x, trans.y, yaw
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

def recv_all(sock, n):
    """
    Receive exactly 'n' bytes of data from a socket.

    Args:
        sock (socket.socket): The socket object to read from.
        n (int): Number of bytes to read.

    Returns:
        bytes: Received data of length n, or None if connection is closed before full data is received.
    """
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

def recv_tag(sock, expected_tag):
    """
    Receive a fixed 4-byte tag from a socket and validate it.

    Args:
        sock (socket.socket): The socket object to read from.
        expected_tag (bytes): Expected tag (e.g., b"RGB_").

    Raises:
        ValueError: If the received tag does not match the expected tag.
    """

    tag = recv_all(sock, 4)
    if tag != expected_tag:
        raise ValueError(f"Expected tag {expected_tag}, got {tag}")

def get_relative_xy(cx, z_mm, image_width=640, hfov_deg=66.0):
    """
    Convert a horizontal pixel position and depth into relative X and Y coordinates in the robot's frame.

    Args:
        cx (int): Horizontal pixel coordinate of the detection center.
        z_mm (int): Depth value at the center in millimeters.
        image_width (int, optional): Width of the image in pixels. Default is 640.
        hfov_deg (float, optional): Horizontal field of view of the camera in degrees. Default is 66.0.

    Returns:
        tuple: Relative X and Y coordinates in meters.
    """
    angle_per_pixel = hfov_deg / image_width
    angle_deg = (cx - image_width / 2) * angle_per_pixel
    angle_rad = math.radians(angle_deg)
    z = z_mm / 1000.0
    x = z * math.cos(angle_rad)
    y = z * math.sin(angle_rad)
    return x, y

def main():
    """
    Main function to run the glass detection pipeline.

    - Initializes ROS2 node and YOLO model.
    - Waits for connection from TurtleBot and receives RGB and depth images.
    - Runs YOLO detection on RGB images to find glass surfaces.
    - Computes 3D positions using depth and robot pose.
    - Extends detected lines and publishes them as RViz markers.
    - Saves detection data to CSV file for further analysis and map projection.
    """

    
    print("🚀 Starting main()")
    rclpy.init()
    node = GlassMarkerPublisher()

    print("⏳ Waiting for TF transform: map → base_link")
    while not node.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
        rclpy.spin_once(node, timeout_sec=0.1)
    print("✅ TF transform available!")

    csv_file = open("glass_detections.csv", mode="w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "timestamp", "x1", "y1", "x2", "y2", "cx", "cy", "z_mm", "label",
        "image_width", "robot_x", "robot_y", "robot_yaw_deg",
        "orig_x1_global", "orig_y1_global", "orig_x2_global", "orig_y2_global",
        "ext_x1_global", "ext_y1_global", "ext_x2_global", "ext_y2_global"
    ])

    model = YOLO("best.pt")

    HOST = '0.0.0.0'
    PORT = 9999
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(1)
    print("Waiting for TurtleBot...")
    conn, addr = s.accept()
    print(f"Connected by {addr}")

    with conn:
        while rclpy.ok():
            try:
                recv_tag(conn, b"RGB_")
                rgb_len = struct.unpack(">I", recv_all(conn, 4))[0]
                rgb_data = recv_all(conn, rgb_len)
                rgb = cv2.imdecode(np.frombuffer(rgb_data, dtype=np.uint8), cv2.IMREAD_COLOR)

                recv_tag(conn, b"DEPT")
                h, w = struct.unpack(">II", recv_all(conn, 8))
                depth_raw = recv_all(conn, h * w * 2)
                depth = np.frombuffer(depth_raw, dtype=np.uint16).reshape((h, w))

                results = model(rgb)[0]
                glass_lines = []
                now = time.time()
                pose = node.get_robot_pose()
                if pose is None:
                    continue
                x_r, y_r, theta = pose
                image_width = rgb.shape[1]

                for box in results.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    label = model.names[int(box.cls[0])]

                    if label.lower() != "glass":
                        continue

                    if not (0 <= cx < w and 0 <= cy < h):
                        continue

                    z_mm = int(depth[cy, cx])
                    if z_mm <= 0 or z_mm > 5000:
                        continue

                    rel_x1, rel_y1 = get_relative_xy(x1, z_mm, image_width)
                    rel_x2, rel_y2 = get_relative_xy(x2, z_mm, image_width)

                    x1_global = x_r + rel_x1 * math.cos(theta) - rel_y1 * math.sin(theta)
                    y1_global = y_r + rel_x1 * math.sin(theta) + rel_y1 * math.cos(theta)

                    x2_global = x_r + rel_x2 * math.cos(theta) - rel_y2 * math.sin(theta)
                    y2_global = y_r + rel_x2 * math.sin(theta) + rel_y2 * math.cos(theta)

                    mid_x = (x1_global + x2_global) / 2
                    mid_y = (y1_global + y2_global) / 2

                    vec_x = mid_x - x_r
                    vec_y = mid_y - y_r
                    vec_len = math.sqrt(vec_x**2 + vec_y**2)
                    if vec_len == 0:
                        continue

                    scale_factor = 1.1
                    vec_x = vec_x / vec_len * vec_len * scale_factor
                    vec_y = vec_y / vec_len * vec_len * scale_factor
                    mid_x = x_r + vec_x
                    mid_y = y_r + vec_y

                    line_dx = x2_global - x1_global
                    line_dy = y2_global - y1_global
                    line_len = math.sqrt(line_dx**2 + line_dy**2)
                    if line_len == 0:
                        continue
                    line_dx /= line_len
                    line_dy /= line_len
                    line_dx *= line_len
                    line_dy *= line_len

                    new_x1 = mid_x - line_dx / 2
                    new_y1 = mid_y - line_dy / 2
                    new_x2 = mid_x + line_dx / 2
                    new_y2 = mid_y + line_dy / 2

                    new_x1, new_y1, new_x2, new_y2 = extend_line_by_percentage(
                        new_x1, new_y1, new_x2, new_y2, extension_percent=0.3
                    )

                    p1 = Point(x=new_x1, y=new_y1, z=0.0)
                    p2 = Point(x=new_x2, y=new_y2, z=0.0)
                    glass_lines.append((p1, p2))

                    csv_writer.writerow([
                        now, x1, y1, x2, y2, cx, cy, z_mm, label,
                        image_width, x_r, y_r, math.degrees(theta),
                        x1_global, y1_global, x2_global, y2_global,
                        new_x1, new_y1, new_x2, new_y2
                    ])
                    csv_file.flush()

                node.publish_glass_markers(glass_lines)
                rclpy.spin_once(node, timeout_sec=0.01)

            except Exception as e:
                print(f"❌ Error during detection: {e}")
                break

    csv_file.close()
    conn.close()
    s.close()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"❌ Startup failed: {e}")
