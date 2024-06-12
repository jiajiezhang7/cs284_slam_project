#!/usr/bin/env python
# TODO make this node faster
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

def point_cloud_callback(msg):
    # 将ROS PointCloud2消息转换为Open3D点云
    cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(cloud))
    if points.shape[0] == 0:
        return  # 无点云数据时返回

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 对点云进行下采样，减少噪声
    #TODO Simulation：不降采样 Realworld-hesai: 0.20
    pcd = pcd.voxel_down_sample(voxel_size=0.05)
    
    # 设置平面检测参数

    distance_threshold = 0.10  # 设置为适合你的点云的值
    ransac_n = 3
    num_iterations = 500
    planes_detected = 0
    combined_wall_points = []

    while True:
        # 检测平面
        plane_model, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)

        # TODO Simulation: 200; Realworld-hesai: 1000
        if len(inliers) < 500:  # 阈值，根据点云的密度进行调整
            break

        # 平面模型的系数a, b, c, d
        [a, b, c] = plane_model[:3]

        # 判断平面是否是水平面（地面）
        horizontal_threshold = 0.1  # This can be tuned
        if abs(a) < horizontal_threshold and abs(b) < horizontal_threshold and abs(c - 1) < horizontal_threshold:

        # 如果是地面则跳过
            pcd = pcd.select_by_index(inliers, invert=True)
            continue

        planes_detected += 1
        inlier_cloud = pcd.select_by_index(inliers)

        
        # 收集属于垂直平面的点云
        combined_wall_points.extend(np.asarray(inlier_cloud.points))

        # 从点云中移除检测到的平面
        pcd = pcd.select_by_index(inliers, invert=True)

    rospy.loginfo(f"Detected Vertical Planes: {planes_detected}")

    # 发布合并的属于垂直平面的点云
    if combined_wall_points:
        combined_wall_cloud = pc2.create_cloud_xyz32(msg.header, np.array(combined_wall_points))
        wall_pc_pub.publish(combined_wall_cloud)

def main():
    rospy.init_node('plane_detection')

    # 创建TF缓存
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 创建订阅器和同步器
    point_cloud_sub = message_filters.Subscriber('/hesai/pandar', PointCloud2)
    ts = message_filters.ApproximateTimeSynchronizer([point_cloud_sub], queue_size=10, slop=0.1)
    ts.registerCallback(point_cloud_callback)

    global wall_pc_pub
    wall_pc_pub = rospy.Publisher('/wall_pc', PointCloud2, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
