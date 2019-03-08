#!/usr/bin/env python
import roslib
import rospy

import tf
from geometry_msgs.msg import TransformStamped

def handle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z),
                     (msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w),
                     msg.header.stamp,
                     "aruco_marker_frame",
                     "camera_color_optical_frame")

if __name__ == '__main__':
    rospy.init_node('marker_tf_broadcaster')
    rospy.Subscriber('/aruco_single/transform',
                     TransformStamped,
                     handle_pose)
    rospy.spin()
