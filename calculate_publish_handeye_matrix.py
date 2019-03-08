#!/usr/bin/env python2

import rospy
from tf import TransformBroadcaster, TransformerROS, transformations as tfs
import tf
from geometry_msgs.msg import Transform
import numpy as np

rospy.init_node('handeye_calibration_publisher')
print("Publishing handeye matrix!")
while rospy.get_time() == 0.0:
    pass

d = np.load("calibration_data.npz")
observed_pts = d['arr_0']
measured_pts = d['arr_1']

# TODO: define function get_rigid_transform to calculate the transformation between two sets of point A and B
def get_rigid_transform(A, B):


R, t = get_rigid_transform(observed_pts, measured_pts)

# TODO: calculate the quaternion q from rotation matrix

# broadcast the hand eye transformation to topic /tf
broad = TransformBroadcaster()
rate = rospy.Rate(50)
while not rospy.is_shutdown():
    broad.sendTransform((t[0],t[1],t[2]), (q[0],q[1],q[2],q[3]), rospy.Time.now(), "camera_color_optical_frame", "base_link")  # takes ..., child, parent
    rate.sleep()
