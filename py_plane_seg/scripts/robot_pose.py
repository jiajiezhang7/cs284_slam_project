#!/usr/bin/env python

# TODO Remain Edited 6.10 21:24
import rospy
import tf
from geometry_msgs.msg import PoseStamped

def publish_pose():
    rospy.init_node('pose_publisher')
    pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
    tf_listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            pose_pub.publish(pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
