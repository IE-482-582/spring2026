import time
import math
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose

'''
Gazebo intentionally versions every Python ABI to avoid breakage when multiple Gazebo releases coexist:
	Component 	Python module
	Transport 	gz.transport13
	Messages	gz.msgs10
	Math		gz.math7
	Sim			gz.sim8
There is no unversioned alias like gz.msgs on Ubuntu Noble / Harmonic
'''

WORLD = "default"
MODEL = "pantilt"
FLOATING_IMAGE = "floating_image"

node = Node()

# Publishers
pan_pub = node.advertise(
    f"/model/{MODEL}/joint/pan_joint/0/cmd_pos",
    Double
)

tilt_pub = node.advertise(
    f"/model/{MODEL}/joint/tilt_joint/0/cmd_pos",
    Double
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
    pose.name = FLOATING_IMAGE
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    ok, response = node.request(
        f"/world/{WORLD}/set_pose",
        pose,
        Pose,
        Boolean,
        1000,
    )
    if not ok or not response.data:
        raise RuntimeError(
            "set_pose request failed. Confirm the world includes "
            "gz::sim::systems::UserCommands and restart Gazebo."
        )

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
