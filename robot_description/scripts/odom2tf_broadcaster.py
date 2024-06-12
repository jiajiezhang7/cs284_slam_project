#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf

def odom_callback(msg):
    br = tf.TransformBroadcaster()
    # 提取位置
    position = msg.pose.pose.position
    # 提取方向（四元数）
    orientation = msg.pose.pose.orientation
    
    # 发布TF变换
    br.sendTransform((position.x, position.y, position.z),
                     (orientation.x, orientation.y, orientation.z, orientation.w),
                     rospy.Time.now(),
                     "base_link",  # 子坐标系
                     "odom")       # 父坐标系

if __name__ == '__main__':
    rospy.init_node('odom_to_tf_broadcaster')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()
