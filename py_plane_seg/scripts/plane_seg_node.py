#!/usr/bin/env python

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from scipy.spatial.transform import Rotation as R
import message_filters
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros

# TODO This Script use RANSAC and Iteration, toooooo slow if pcd is too dense(e.g. voxel_size = 0.05)

def point_cloud_callback(msg):
    # 将ROS PointCloud2消息转换为Open3D点云
    cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(cloud))
    if points.shape[0] == 0:
        return  # 无点云数据时返回

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 对点云进行下采样，减少噪声
    pcd = pcd.voxel_down_sample(voxel_size=0.07)
    
    # 设置平面检测参数
    distance_threshold = 0.05  # 设置为适合你的点云的值
    ransac_n = 3
    num_iterations = 500
    planes_detected = 0
    marker_array = MarkerArray()
    combined_wall_points = []

    # 创建一个删除所有旧Marker的Marker
    delete_marker = Marker()
    delete_marker.header.frame_id = msg.header.frame_id
    delete_marker.header.stamp = rospy.Time.now()
    delete_marker.ns = "plane"
    delete_marker.id = 0
    delete_marker.action = Marker.DELETEALL
    marker_array.markers.append(delete_marker)

    # marker_pub.publish(marker_array)
    # marker_array.markers = []  # 清空marker_array，准备添加新的marker

    while True:
        # 检测平面
        plane_model, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
        if len(inliers) < 200:  # 阈值，根据点云的密度进行调整
            break

        planes_detected += 1

        # 平面模型的系数a, b, c, d
        [a, b, c, d] = plane_model
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "plane"
        marker.id = planes_detected
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # 获取平面的中心
        inlier_cloud = pcd.select_by_index(inliers)
        center = inlier_cloud.get_center()

        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]

        # 将平面的法线方向转换为四元数
        normal = np.array([a, b, c])
        # Ensure the normal is a unit vector
        normal = normal / np.linalg.norm(normal)
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(normal * np.pi)
        r = R.from_matrix(rotation_matrix)
        quaternion = r.as_quat()
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]

        # 设置marker的大小
        extent = inlier_cloud.get_max_bound() - inlier_cloud.get_min_bound()
        marker.scale.x = extent[0]
        marker.scale.y = extent[1]
        marker.scale.z = extent[2]

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        marker_array.markers.append(marker)

        # 收集属于平面的点云
        combined_wall_points.extend(np.asarray(inlier_cloud.points))

        # 从点云中移除检测到的平面
        pcd = pcd.select_by_index(inliers, invert=True)

    rospy.loginfo(f"检测到的平面数量: {planes_detected}")
    marker_pub.publish(marker_array)

    # 发布合并的属于平面的点云
    if combined_wall_points:
        combined_wall_cloud = pc2.create_cloud_xyz32(msg.header, np.array(combined_wall_points))
        wall_pc_pub.publish(combined_wall_cloud)

def main():
    rospy.init_node('open3d_plane_detection')

    # 创建TF缓存
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 创建订阅器和同步器
    point_cloud_sub = message_filters.Subscriber('/velodyne_points', PointCloud2)
    ts = message_filters.ApproximateTimeSynchronizer([point_cloud_sub], queue_size=10, slop=0.1)
    ts.registerCallback(point_cloud_callback)

    global marker_pub, wall_pc_pub
    marker_pub = rospy.Publisher('/plane_markers', MarkerArray, queue_size=10)
    wall_pc_pub = rospy.Publisher('/wall_pc', PointCloud2, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
