#!/usr/bin/env python

import rospy
import numpy
import tf
import tf2_ros
import geometry_msgs.msg

from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseStamped


def message_from_transform(T):
    msg = Transform()
    quaternion_part = tf.transformations.quaternion_from_matrix(T)
    translation_part = tf.transformations.translation_from_matrix(T)
    msg.translation.x = translation_part[0]
    msg.translation.y = translation_part[1]
    msg.translation.z = translation_part[2]
    msg.rotation.x = quaternion_part[0]
    msg.rotation.y = quaternion_part[1]
    msg.rotation.z = quaternion_part[2]
    msg.rotation.w = quaternion_part[3]
    return msg

def publish_transforms():
    
    T1_translation = tf.transformations.translation_matrix((0.0, 1.0, 1.0))
    q1 = tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)
    T1_rotation = tf.transformations.quaternion_matrix(q1)
    T1 = numpy.dot(T1_rotation, T1_translation)

    tf_base_to_obj = TransformStamped()
    tf_base_to_obj.header.stamp = rospy.Time.now()
    tf_base_to_obj.header.frame_id = "base_frame"
    tf_base_to_obj.child_frame_id = "object_frame"
    tf_base_to_obj.transform = message_from_transform(T1)
    br.sendTransform(tf_base_to_obj)
###########################################################
    
    T2_translation = tf.transformations.translation_matrix((0.0, -1.0, 0.0))
    q2 = tf.transformations.quaternion_from_euler(0.0, 0.0, 1.5)
    T2_rotation = tf.transformations.quaternion_matrix(q2)
    T2 = numpy.dot(T2_rotation, T2_translation)

    tf_base_to_robot = TransformStamped()
    tf_base_to_robot.header.stamp = rospy.Time.now()
    tf_base_to_robot.header.frame_id = "base_frame"
    tf_base_to_robot.child_frame_id = "robot_frame"
    tf_base_to_robot.transform = message_from_transform(T2)
    br.sendTransform(tf_base_to_robot)
############################################################
    
    T2_inverse = tf.transformations.inverse_matrix(T2)
    T_Robot_obj = numpy.dot(T2_inverse, T1)
    
    P_object = numpy.array([[0], [0], [0], [1]])
    P_Robot_frame = numpy.dot(T_Robot_obj, P_object)
    P_Robot_frame[0] = P_Robot_frame[0] 
    P_Robot_frame[1] = P_Robot_frame[1] 
    P_Robot_frame[2] = P_Robot_frame[2]

    a = numpy.array([[1, 0, 0]])
    b = numpy.array([P_Robot_frame[0][0],P_Robot_frame[1][0]-0.1, P_Robot_frame[2][0]-0.1])
    a_u = numpy.linalg.norm(a)
    b_u = numpy.linalg.norm(b)

    dot_prod = numpy.dot(a,b.transpose())
    cross_prod = numpy.cross(a,b)
    cos_theta = dot_prod/(a_u*b_u)
    theta_x = numpy.arccos(cos_theta)

    T3_translation = tf.transformations.translation_matrix((0.0, 0.1, 0.1))
    q3 = tf.transformations.quaternion_about_axis(theta_x,cross_prod)
    
    euler_ang = tf.transformations.euler_from_quaternion(q3)
    print euler_ang
    q3 = tf.transformations.quaternion_from_euler(euler_ang[0], euler_ang[1], euler_ang[2])
    
    T3_rotation = tf.transformations.quaternion_matrix(q3)
    T3 = numpy.dot(T3_translation, T3_rotation)
    
    tf_robot_to_camera = TransformStamped()
    tf_robot_to_camera.header.stamp = rospy.Time.now()
    tf_robot_to_camera.header.frame_id = "robot_frame"
    tf_robot_to_camera.child_frame_id = "camera_frame"
    tf_robot_to_camera.transform = message_from_transform(T3)
    br.sendTransform(tf_robot_to_camera)
    
############################################################
    
if __name__ == '__main__':
    rospy.init_node('learning_tf')
    rate = rospy.Rate(20)
    br = tf2_ros.TransformBroadcaster()
    pose_pub_obj = rospy.Publisher("/position_obj", PoseStamped, queue_size=20)
    pose_pub_robot = rospy.Publisher("/position_robot", PoseStamped, queue_size=20)
    while not rospy.is_shutdown():
        publish_transforms()
        rate.sleep()
    
    
    
