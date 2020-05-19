#!/usr/bin/env python  
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg

def parse_transform(Tr):
    t = geometry_msgs.msg.Transform()
    
    translation = tf.transformations.translation_from_matrix(Tr)
    t.translation.x = translation[0]
    t.translation.y = translation[1]
    t.translation.z = translation[2]
    
    q = tf.transformations.quaternion_from_matrix(Tr)
    t.rotation.x = q[0]
    t.rotation.y = q[1]
    t.rotation.z = q[2]
    t.rotation.w = q[3]
    
    return t


def publish_transforms():
        
    # quaternion for B -> O transform
    quat1 = tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)

    # B -> O transform
    Tr1 = numpy.dot( tf.transformations.quaternion_matrix(quat1), tf.transformations.translation_matrix((0.0,1.0,1.0)))
    #Tr1 = tf.transformations.concatenate_matrices(tf.transformations.quaternion_matrix(quat1), tf.transformations.translation_matrix(0.0,1.0,1.0))

    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"

    # parse transform
    object_transform.transform = parse_transform(Tr1)

    # Broacast
    br.sendTransform(object_transform)

    # quaternion for B -> R transform
    quat2 = tf.transformations.quaternion_about_axis(1.5, (0,0,1))

    # B -> R transform
    Tr2 = numpy.dot(tf.transformations.quaternion_matrix(quat2), tf.transformations.translation_matrix((0.0,-1.0,0.0)))
    #Tr2 = tf.transformations.concatenate_matrices(tf.transformations.quaternion_matrix(quat2), tf.transformations.translation_matrixranslation_matrix(0.0,-1.0,0.0),)

    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"

    # parse transform
    robot_transform.transform = parse_transform(Tr2)
    
    # Broadcast
    br.sendTransform(robot_transform)

    # calculations for R -> C transforms
    Tr_b_o = Tr1
    Tr_b_r = Tr2
    Tr_r_c = tf.transformations.translation_matrix((0.0,0.1,0.1))
    Tr_b_c = numpy.dot(Tr_b_r,Tr_r_c)
    Tr_c_o = numpy.dot(tf.transformations.inverse_matrix(Tr_b_c),Tr_b_o)

    # After Finding T_C_O HINT FROM Ping Fang
    # Next step is to find a rotation axis and angle for our camera frame to have 
    # its x-axis coincide with this translation vector. 
    # The way to do that is, first normalize your translation vector, 
    # second, apply outer product to it with [1,0,0] to find the axis. 
    # Third, use arccos of the vector and [1,0,0] to find the angle.


    # Normalizing translations from T_c_o

    normalized_vector = tf.transformations.translation_from_matrix(Tr_c_o)/numpy.linalg.norm(tf.transformations.translation_from_matrix(Tr_c_o))
    
    # apply outer product to it with [1,0,0] to find the axis
    axis_for_tr = numpy.cross([1,0,0], normalized_vector)

    # Third, use arccos of the vector and [1,0,0] to find the angle
    angle_for_tr = numpy.arccos(numpy.dot([1,0,0],normalized_vector))

    # quaternion from axis and angle
    quat3 = tf.transformations.quaternion_about_axis(angle_for_tr, axis_for_tr)

    # Concatenate matrix
    Tr3 = tf.transformations.concatenate_matrices(Tr_r_c, tf.transformations.quaternion_matrix(quat3))

    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"

    # parse transform
    camera_transform.transform = parse_transform(Tr3)
    
    # Broadcast
    br.sendTransform(camera_transform)
    
if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
