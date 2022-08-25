#!/usr/bin/python3

import rospy
import numpy as np
import tf2_ros
from tf_conversions import transformations as tfct
import modern_robotics as mr
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3

if __name__ == '__main__':

    rospy.init_node('tf2_camera')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
                     
    while not rospy.is_shutdown():

        try:

            # Get Tsb from robot

            # TransformStamped
            trans_sb = tfBuffer.lookup_transform('world', 'gripper_tip', rospy.Time()).transform
            
            qsb = trans_sb.rotation
            tsb = trans_sb.translation

            Rsb = tfct.quaternion_matrix(np.array([qsb.x, qsb.y, qsb.z, qsb.w]))[:3, :3]
            
            psb = np.array([tsb.x, tsb.y, tsb.z])
            
            Tsb = mr.RpToTrans(Rsb, psb)

            Tsc = np.eye(4)
            Tsc[:3, 3:] = np.array([[0.0, 0.0, 2.0]]).T
            Tsc[:3, :3] = mr.MatrixExp3(np.pi * mr.VecToso3(np.array([1.0, 0.0, 0.0])))
            
            Tcb = mr.TransInv(Tsc) @ Tsb

            # Numpy
            quat_cb = tfct.quaternion_from_matrix(Tcb)
            pos_cb = tfct.translation_from_matrix(Tcb)
            
            # ROS geometry_msgs
            qcb = Quaternion(quat_cb[0], quat_cb[1], quat_cb[2], quat_cb[3])
            tcb = Vector3(pos_cb[0], pos_cb[1], pos_cb[2])
            

            tf_stamp = TransformStamped()
            tf_stamp.child_frame_id = 'gripper_camera'
            tf_stamp.header.frame_id = 'camera'

            tf_stamp.header.stamp = rospy.Time.now()

            tf_stamp.transform.rotation = qcb
            tf_stamp.transform.translation = tcb
    
            br = tf2_ros.TransformBroadcaster()
            br.sendTransform(tf_stamp)

            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue