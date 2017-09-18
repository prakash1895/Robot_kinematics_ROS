#!/usr/bin/env python

import math
import numpy
import time
from threading import Thread, Lock

import rospy
import tf
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from urdf_parser_py.urdf import URDF

def S_matrix(w):
    S = numpy.zeros((3,3))
    S[0,1] = -w[2]
    S[0,2] =  w[1]
    S[1,0] =  w[2]
    S[1,2] = -w[0]
    S[2,0] = -w[1]
    S[2,1] =  w[0]
    return S

def cartesian_control(joint_transforms, b_T_ee_current, b_T_ee_desired, red_control, q_current, q0_desired):
    
    num_joints = len(joint_transforms)
    dq = numpy.zeros(num_joints)

    b_T_ee_inv = tf.transformations.inverse_matrix(b_T_ee_current)
    delta_T = numpy.dot(b_T_ee_inv,b_T_ee_desired)
    delta_pos = tf.transformations.translation_from_matrix(delta_T)
    delta_rot = submatrix(delta_T,0,0,3)
    angle,axis = rotation_from_matrix(delta_rot)
    delta_ang = numpy.dot(angle,axis)
   
    delta_X = numpy.zeros(6)
    delta_X[0] = delta_pos[0]
    delta_X[1] = delta_pos[1]
    delta_X[2] = delta_pos[2]
    delta_X[3] = delta_ang[0]
    delta_X[4] = delta_ang[1]
    delta_X[5] = delta_ang[2]

    X_dot = 1*delta_X
    b_R_ee = submatrix(b_T_ee_current,0,0,3)
    ee_R_b = numpy.transpose(b_R_ee)

    K = numpy.zeros((6,6))
    V_ee = numpy.zeros((6,1))
    
    for i in range(6):
        for j in range(6):
            if i<3 and j<3:
                K[i][j] = ee_R_b[i][j]
            elif i>=3 and j>=3:
                K[i][j] = ee_R_b[i-3][j-3]
            else:
                K[i][j] = 0

    V_ee = numpy.dot(K,X_dot)

    max_vel = 0
    for i in range(3):
        if abs(V_ee[i]) > max_vel:
            max_vel = abs(V_ee[i])
    if max_vel > 0.1:
        for i in range(3):
            V_ee[i] = (V_ee[i]/max_vel)*0.1

    max_ang_vel = 0
    for i in range(3):
        if abs(V_ee[i+3]) > max_ang_vel:
            max_ang_vel = abs(V_ee[i+3])
    if max_ang_vel > 1:
        for i in range(3):
            V_ee[i+3] = (V_ee[i+3]/max_ang_vel)*1

    j_T_ee = []
    ee_R_j = []
    RS_j = []
    
    for i in range(num_joints):
        b_T_j = joint_transforms[i]
        b_T_j_inv = tf.transformations.inverse_matrix(b_T_j)
        j_T_ee.append(numpy.dot(b_T_j_inv,b_T_ee_current))

        Rot_temp = submatrix(j_T_ee[i],0,0,3)
        ee_R_j.append(numpy.transpose(Rot_temp))
        
        trans_temp = tf.transformations.translation_from_matrix(j_T_ee[i])
        S = S_matrix(trans_temp)
        S = (-1)*S
        RS_j.append(numpy.dot(ee_R_j[i],S))

    J = numpy.zeros((6,num_joints))

    Vj = []
    for i in range(num_joints):
        Vj_temp = Vj_creator(ee_R_j[i],RS_j[i])
        Vj.append(Vj_temp)

    for i in range(num_joints):
        Vj_col = Vj[i][:,5]
        J[:,i] = Vj_col
        
    J_plus = numpy.linalg.pinv(J,0.01)

    dq = numpy.dot(J_plus,V_ee)

    max_ang_vel = 0
    for i in range(7):
        if abs(dq[i]) > max_ang_vel:
            max_ang_vel = abs(dq[i])
    if max_ang_vel > 1:
        for i in range(7):
            dq[i] = (dq[i]/max_ang_vel)*1
            
    return dq
    
def Vj_creator(A,B):

    C = numpy.zeros((6,6))
    for i in range(6):
        for j in range(6):
            if i<3 and j<3:
                C[i][j] = A[i][j]
            elif i<3 and j>=3:
                C[i][j] = B[i][j-3]
            elif i>=3 and j<3:
                C[i][j] = 0
            elif i>=3 and j>=3:
                C[i][j] = A[i-3][j-3]
    return C

def submatrix(matrix,row,col,size):
    return matrix[row:row+size,col:col+size]
          
def convert_from_message(t):
    trans_x = t.translation.x
    trans_y = t.translation.y
    trans_z = t.translation.z

    rot_x = t.rotation.x
    rot_y = t.rotation.y
    rot_z = t.rotation.z
    rot_w = t.rotation.w
    
    trans = tf.transformations.translation_matrix((trans_x,trans_y,trans_z))
    rot = tf.transformations.quaternion_matrix((t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))                                               
    T = numpy.dot(trans,rot)
    return T

# Returns the angle-axis representation of the rotation contained in the input matrix
# Use like this:
# angle, axis = rotation_from_matrix(R)
def rotation_from_matrix(matrix):
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    R33 = R[:3, :3]
    # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, W = numpy.linalg.eig(R33.T)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    axis = numpy.real(W[:, i[-1]]).squeeze()
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = numpy.linalg.eig(R)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    # rotation angle depending on axis
    cosa = (numpy.trace(R33) - 1.0) / 2.0
    if abs(axis[2]) > 1e-8:
        sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
    elif abs(axis[1]) > 1e-8:
        sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
    else:
        sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
    angle = math.atan2(sina, cosa)
    return angle, axis

class CartesianControl(object):

    #Initialization
    def __init__(self):
        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        #Subscribes to command for end-effector pose
        rospy.Subscriber("/cartesian_command", Transform, self.command_callback)

        #Subscribes to command for redundant dof
        rospy.Subscriber("/redundancy_command", Float32, self.redundancy_callback)

        # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

        #This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.q_current = []
        self.x_current = tf.transformations.identity_matrix()
        self.R_base = tf.transformations.identity_matrix()
        self.x_target = tf.transformations.identity_matrix()
        self.q0_desired = 0
        self.last_command_time = 0
        self.last_red_command_time = 0

        # Initialize timer that will trigger callbacks
        self.mutex = Lock()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def command_callback(self, command):
        self.mutex.acquire()
        self.x_target = convert_from_message(command)
        self.last_command_time = time.time()
        self.mutex.release()

    def redundancy_callback(self, command):
        self.mutex.acquire()
        self.q0_desired = command.data
        self.last_red_command_time = time.time()
        self.mutex.release()        
        
    def timer_callback(self, event):
        msg = JointState()
        self.mutex.acquire()
        if time.time() - self.last_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_target,
                                   False, self.q_current, self.q0_desired)
            msg.velocity = dq
        elif time.time() - self.last_red_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_current,
                                   True, self.q_current, self.q0_desired)
            msg.velocity = dq
        else:            
            msg.velocity = numpy.zeros(7)
        self.mutex.release()
        self.pub_vel.publish(msg)
        
    def joint_callback(self, joint_values):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.mutex.acquire()
        self.joint_transforms = []
        self.q_current = joint_values.position
        self.process_link_recursive(root, T, joint_values)
        self.mutex.release()

    def align_with_z(self, axis):
        T = tf.transformations.identity_matrix()
        z = numpy.array([0,0,1])
        x = numpy.array([1,0,0])
        dot = numpy.dot(z,axis)
        if dot == 1: return T
        if dot == -1: return tf.transformation.rotation_matrix(math.pi, x)
        rot_axis = numpy.cross(z, axis)
        angle = math.acos(dot)
        return tf.transformations.rotation_matrix(angle, rot_axis)

    def process_link_recursive(self, link, T, joint_values):
        if link not in self.robot.child_map: 
            self.x_current = T
            return
        for i in range(0,len(self.robot.child_map[link])):
            (joint_name, next_link) = self.robot.child_map[link][i]
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                continue
            current_joint = self.robot.joint_map[joint_name]        

            trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0], 
                                                                  current_joint.origin.xyz[1],
                                                                  current_joint.origin.xyz[2]))
            rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0], 
                                                         current_joint.origin.rpy[1],
                                                         current_joint.origin.rpy[2], 'rxyz')
            origin_T = numpy.dot(trans_matrix, rot_matrix)
            current_joint_T = numpy.dot(T, origin_T)
            if current_joint.type != 'fixed':
                if current_joint.name not in joint_values.name:
                    rospy.logerror("Joint not found in list")
                    continue
                # compute transform that aligns rotation axis with z
                aligned_joint_T = numpy.dot(current_joint_T, self.align_with_z(current_joint.axis))
                self.joint_transforms.append(aligned_joint_T)
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_T = tf.transformations.rotation_matrix(angle, 
                                                                 numpy.asarray(current_joint.axis))
                next_link_T = numpy.dot(current_joint_T, joint_rot_T) 
            else:
                next_link_T = current_joint_T

            self.process_link_recursive(next_link, next_link_T, joint_values)
        
if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    cc = CartesianControl()
    rospy.spin()
