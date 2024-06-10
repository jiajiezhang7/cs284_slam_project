#!/usr/bin/env python

# this is a method using plane segmentation from paper 2020 Robust....
import rospy
import open3d as o3d # type: ignore
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def point_cloud_callback(msg):
    # 将ROS PointCloud2消息转换为Open3D点云
    cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(cloud))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 设置平面检测参数
    normal_variance_threshold_deg = 60
    coplanarity_deg = 75
    outlier_ratio = 0.75
    min_plane_edge_length = 0.0  # 使用默认值
    min_num_points = 0  # 使用默认值
    search_param = o3d.geometry.KDTreeSearchParamKNN(knn=30)

    # 检测平面
    plane_patches = pcd.detect_planar_patches(
        normal_variance_threshold_deg,
        coplanarity_deg,
        outlier_ratio,
        min_plane_edge_length,
        min_num_points,
        search_param
    )

    rospy.loginfo(f"检测到的平面数量: {len(plane_patches)}")

    marker_array = MarkerArray()

    for i, patch in enumerate(plane_patches):
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "plane"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        obb = o3d.geometry.OrientedBoundingBox(patch.center, patch.R, patch.extent)
        marker.pose.position.x = obb.center[0]
        marker.pose.position.y = obb.center[1]
        marker.pose.position.z = obb.center[2]

        # 由于OrientedBoundingBox不提供直接的四元数，我们需要转换R矩阵
        q = o3d.geometry.get_rotation_matrix_from_xyz(patch.R)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.scale.x = obb.extent[0]
        marker.scale.y = obb.extent[1]
        marker.scale.z = obb.extent[2]

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        marker_array.markers.append(marker)

    marker_pub.publish(marker_array)

def main():
    rospy.init_node('open3d_plane_detection')
    rospy.Subscriber('/velodyne_points', PointCloud2, point_cloud_callback)
    global marker_pub
    marker_pub = rospy.Publisher('/plane_markers', MarkerArray, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
