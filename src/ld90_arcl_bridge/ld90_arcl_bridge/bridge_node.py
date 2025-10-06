import socket, threading, time, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from builtin_interfaces.msg import Time as RosTime
from tf2_ros import TransformBroadcaster

def yaw_to_quat(yaw):
    # Z-rotation quaternion
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w

class ArclClient:
    def __init__(self, node, host, port, password, reconnect_sec=3.0):
        self.node = node
        self.host = host
        self.port = port
        self.password = password.strip() if password else ""
        self.reconnect_sec = reconnect_sec
        self.sock = None
        self.rx_thread = None
        self.connected = False
        self.stop_flag = False
        self.lock = threading.Lock()
        self.on_pose = None
        self.on_status = None

    def start(self):
        t = threading.Thread(target=self._connect_loop, daemon=True)
        t.start()

    def stop(self):
        self.stop_flag = True
        try:
            if self.sock:
                self.sock.close()
        except:
            pass

    def send_line(self, line):
        with self.lock:
            if not self.connected or not self.sock:
                return False
            try:
                self.sock.sendall((line + "\r\n").encode('utf-8'))
                return True
            except Exception as e:
                self.node.get_logger().warn(f"Send failed: {e}")
                self.connected = False
                return False

    def _connect_loop(self):
        while not self.stop_flag:
            try:
                self.node.get_logger().info(f"Connecting to ARCL {self.host}:{self.port} ...")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5.0)
                s.connect((self.host, self.port))
                s.settimeout(None)  # blocking read
                self.sock = s
                self.connected = True
                self.node.get_logger().info("Connected to ARCL")

                # Optional login
                if self.password:
                    self.send_line(f"connect {self.password}")

                # Start RX loop
                self._rx_loop()
            except Exception as e:
                self.node.get_logger().warn(f"ARCL connect error: {e}")
                self.connected = False
                try:
                    if self.sock:
                        self.sock.close()
                except:
                    pass
                time.sleep(self.reconnect_sec)

    def _rx_loop(self):
        file = self.sock.makefile('rb')
        while not self.stop_flag and self.connected:
            line = file.readline()
            if not line:
                self.node.get_logger().warn("ARCL connection closed by peer")
                self.connected = False
                break
            try:
                text = line.decode('utf-8', errors='ignore').strip()
            except:
                continue
            if not text:
                continue
            # Expected lines like:
            #   POSE x y yaw
            #   STATUS something...
            # You can extend as needed.
            if text.upper().startswith("POSE "):
                parts = text.split()
                if len(parts) >= 4:
                    try:
                        x = float(parts[1]); y = float(parts[2]); yaw = float(parts[3])
                        if self.on_pose:
                            self.on_pose(x, y, yaw)
                    except:
                        pass
            elif text.upper().startswith("STATUS"):
                if self.on_status:
                    self.on_status(text)

class LD90Bridge(Node):
    def __init__(self):
        super().__init__('ld90_bridge')
        # Params
        self.declare_parameter('arcl.host', '127.0.0.1')
        self.declare_parameter('arcl.port', 7260)
        self.declare_parameter('arcl.password', '')
        self.declare_parameter('topics.cmd_vel', '/cmd_vel')
        self.declare_parameter('topics.odom', '/odom')
        self.declare_parameter('topics.status', '/ld90/status')
        self.declare_parameter('frames.odom_frame', 'odom')
        self.declare_parameter('frames.base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('cmd_rate_hz', 10.0)
        self.declare_parameter('reconnect_sec', 3.0)

        host = self.get_parameter('arcl.host').get_parameter_value().string_value
        port = self.get_parameter('arcl.port').get_parameter_value().integer_value
        pwd  = self.get_parameter('arcl.password').get_parameter_value().string_value
        self.odom_topic   = self.get_parameter('topics.odom').get_parameter_value().string_value
        self.status_topic = self.get_parameter('topics.status').get_parameter_value().string_value
        self.cmd_topic    = self.get_parameter('topics.cmd_vel').get_parameter_value().string_value
        self.odom_frame   = self.get_parameter('frames.odom_frame').get_parameter_value().string_value
        self.base_frame   = self.get_parameter('frames.base_frame').get_parameter_value().string_value
        self.publish_tf   = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.cmd_rate_hz  = float(self.get_parameter('cmd_rate_hz').get_parameter_value().double_value)
        reconnect_sec     = float(self.get_parameter('reconnect_sec').get_parameter_value().double_value)

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # State
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.last_twist = Twist()
        self.last_cmd_time = 0.0
        self.cmd_period = 1.0 / max(1e-3, self.cmd_rate_hz)

        # ARCL client
        self.arcl = ArclClient(self, host, port, pwd, reconnect_sec=reconnect_sec)
        self.arcl.on_pose = self._on_pose
        self.arcl.on_status = self._on_status
        self.arcl.start()

        # Subscribe to cmd_vel
        self.sub_cmd = self.create_subscription(Twist, self.cmd_topic, self._on_cmd_vel, 10)

        self.get_logger().info(f"LD90Bridge running. ARCL at {host}:{port}, odom->{self.base_frame}, cmd_vel topic {self.cmd_topic}")

    def _on_status(self, text):
        msg = String(); msg.data = text
        self.pub_status.publish(msg)

    def _on_pose(self, x, y, yaw):
        self.pose_x = x; self.pose_y = y; self.pose_yaw = yaw
        self._publish_odom_and_tf()

    def _publish_odom_and_tf(self):
        now = self.get_clock().now().to_msg()

        # Odometry
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.pose_x
        od.pose.pose.position.y = self.pose_y
        q = yaw_to_quat(self.pose_yaw)
        od.pose.pose.orientation.x = q[0]
        od.pose.pose.orientation.y = q[1]
        od.pose.pose.orientation.z = q[2]
        od.pose.pose.orientation.w = q[3]
        od.twist.twist = self.last_twist
        self.pub_odom.publish(od)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.pose_x
            tf.transform.translation.y = self.pose_y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = q[0]
            tf.transform.rotation.y = q[1]
            tf.transform.rotation.z = q[2]
            tf.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(tf)

    def _on_cmd_vel(self, msg: Twist):
        self.last_twist = msg
        # Throttle sending to ARCL
        t = time.time()
        if (t - self.last_cmd_time) < self.cmd_period:
            return
        self.last_cmd_time = t
        vx = msg.linear.x
        wz = msg.angular.z
        # Simple command understood by fake server; adapt for real ARCL if needed.
        self.arcl.send_line(f"DRIVE {vx:.3f} {wz:.3f}")

def main():
    rclpy.init()
    node = LD90Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arcl.stop()
        node.destroy_node()
        rclpy.shutdown()
