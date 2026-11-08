import json, math, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, Imu, CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
rclpy.init(); n = Node("fake_drone")
st = n.create_publisher(String, "/skybeta/state", 10)
od = n.create_publisher(Odometry, "/visual_slam/tracking/odometry", 10)
dp = n.create_publisher(Image, "/camera/depth/image_raw", 10)
ci = n.create_publisher(CameraInfo, "/camera/depth/camera_info", 10)
lf = n.create_publisher(Image, "/oak/left/image_raw", 10)
im = n.create_publisher(Imu, "/oak/imu/data", 10)
pa = n.create_publisher(Path, "/visual_slam/tracking/slam_path", 10)
tfs = n.create_publisher(TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t = TransformStamped(); t.header.frame_id = "base_link"; t.child_frame_id = "oak_parent_frame"; t.transform.translation.x = .1053; t.transform.rotation.w = 1.0
tfs.publish(TFMessage(transforms=[t]))
depth = Image(height=400, width=640, encoding="16UC1", step=1280, data=bytes(640 * 400 * 2))
left = Image(height=800, width=1280, encoding="bgr8", step=3840, data=bytes(1280 * 800 * 3))
k = [0]
def fast():
    k[0] += 1
    m = Imu(); m.linear_acceleration.z = 9.8; im.publish(m)
def cam():
    o = Odometry(); o.pose.pose.position.x = 0.01 * k[0]; o.pose.pose.orientation.w = 1.0; od.publish(o)
    dp.publish(depth); ci.publish(CameraInfo(width=640, height=400)); lf.publish(left); pa.publish(Path())
def slow():
    st.publish(String(data=json.dumps({"px4_link": True, "armed": False, "landed": True, "offboard_active": False})))
n.create_timer(1/200, fast); n.create_timer(1/30, cam); n.create_timer(1/10, slow)
rclpy.spin(n)
