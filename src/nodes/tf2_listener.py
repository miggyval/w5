#!/usr/bin/python3

import rospy
import numpy as np
import tf2_ros
from tf_conversions import transformations as tfct
import modern_robotics as mr

if __name__ == '__main__':

    rospy.init_node('tf2_listener')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        try:

            # Get Tsb from robot

            trans_sb = tfBuffer.lookup_transform('world', 'gripper_tip', rospy.Time()).transform
            q = trans_sb.rotation
            t = trans_sb.translation
            Rsb = tfct.quaternion_matrix(np.array([q.x, q.y, q.z, q.w]))[:3, :3]
            psb = np.array([t.x, t.y, t.z])
            Tsb = mr.RpToTrans(Rsb, psb)

            # Get Tsb from camera

            trans_cb = tfBuffer.lookup_transform('camera', 'gripper_camera', rospy.Time()).transform
            q = trans_cb.rotation
            t = trans_cb.translation
            Rcb = tfct.quaternion_matrix(np.array([q.x, q.y, q.z, q.w]))[:3, :3]
            pcb = np.array([t.x, t.y, t.z])
            Tcb = mr.RpToTrans(Rcb, pcb)
            
            print("Tsb =")
            print(Tsb)
            print("Tcb =")
            print(Tcb)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue