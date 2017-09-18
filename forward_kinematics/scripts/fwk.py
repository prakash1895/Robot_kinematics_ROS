#!/usr/bin/env python

import numpy

import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.msg
from urdf_parser_py.urdf import URDF

def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]        
    return t
    
class ForwardKinematics(object):
    
    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)
        
        self.robot = URDF.from_parameter_server()
        
        rospy.Subscriber("joint_states", JointState, self.callback)


    def callback(self, joint_values):
        link_name = self.robot.get_root()
        link_names = []
        joints = []
        while True:
            if link_name not in self.robot.child_map:
                break
          
            if len(self.robot.child_map[link_name]) != 1:
                rospy.logerror("Forked kinematic chain!");
                break
           
            (joint_name, next_link_name) = self.robot.child_map[link_name][0]
         
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                break;
            joints.append(self.robot.joint_map[joint_name])
            link_names.append(next_link_name)     
            link_name = next_link_name
            
        all_transforms = self.compute_transforms(link_names, joints, joint_values)
        self.pub_tf.publish(all_transforms)

    def compute_transforms(self, link_names, joints, joint_values):
        
        all_transforms = tf.msg.tfMessage()
        tf_msg = geometry_msgs.msg.TransformStamped()
        
        I = tf.transformations.identity_matrix()
        T_prev = I 
    
        for i in range(len(link_names)):
            
            x = joints[i].origin.xyz[0]
            y = joints[i].origin.xyz[1]
            z = joints[i].origin.xyz[2]
            
            roll = joints[i].origin.rpy[0]
            pitch = joints[i].origin.rpy[1]
            yaw = joints[i].origin.rpy[2]
            
            T_translation = tf.transformations.translation_matrix((x, y, z))
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            T_rotation = tf.transformations.quaternion_matrix(quaternion)
            T_matrix_link = numpy.dot(T_translation, T_rotation)
            T_intermediate = numpy.dot(T_prev, T_matrix_link)
  
            if joints[i].type == 'revolute':
                axis = joints[i].axis
                for index,name in enumerate(joint_values.name):
                    if name == joints[i].name:
                        theta = joint_values.position[index]        
                quaternion = tf.transformations.quaternion_about_axis(theta,axis)
                T_rotation = tf.transformations.quaternion_matrix(quaternion)
                T_matrix = numpy.dot(T_intermediate, T_rotation)

            elif joints[i].type == 'fixed':
                T_matrix = numpy.dot(T_intermediate, I)
            
            tf_msg = convert_to_message(T_matrix, link_names[i], 'world_link')
            all_transforms.transforms.insert(i,tf_msg)
            T_prev = T_matrix
                        
        return all_transforms
           
if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()

