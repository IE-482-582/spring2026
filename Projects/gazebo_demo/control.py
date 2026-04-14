import time
import math
from gz.transport13 import Node
from gz.msgs.double_pb2 import Double
from gz.msgs.pose_pb2 import Pose
from gz.msgs.vector3d_pb2 import Vector3d

WORLD = "default"
MODEL = "pantilt"

node = Node()

# Publishers
pan_pub = node.advertise(
    f"/model/{MODEL}/joint/pan_joint/cmd_pos",
    Double
)

tilt_pub = node.advertise(
    f"/model/{MODEL}/joint/tilt_joint/cmd_pos",
    Double
)

pose_pub = node.advertise(
    f"/world/{WORLD}/set_pose",
    Pose
)

def set_pan(angle):
    msg = Double()
    msg.data = angle
    pan_pub.publish(msg)

def set_tilt(angle):
    msg = Double()
    msg.data = angle
    tilt_pub.publish(msg)

def move_image(x, y, z):
    pose = Pose()
    pose.name = "floating_image"
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose_pub.publish(pose)

# Simple sweep demo
t = 0
while True:
    pan = math.sin(t) * 1.0
    tilt = math.cos(t) * 0.5

    set_pan(pan)
    set_tilt(tilt)

    move_image(2, 0, 1 + 0.5 * math.sin(t))

    t += 0.05
    time.sleep(0.02)
