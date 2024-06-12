import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def point_cloud_callback(msg):
    # 将ROS PointCloud2消息转换为Open3D点云
    cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(cloud))
    if points.shape[0] == 0:
        return  # 无点云数据时返回

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 估计法线
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # DBSCAN 聚类
    labels = np.array(pcd.cluster_dbscan(eps=0.05, min_points=10))

    # 提取主要平面
    max_label = labels.max()
    if max_label < 0:
        rospy.loginfo("No planes found")
        return

    largest_cluster_idx = np.argmax(np.bincount(labels[labels >= 0]))
    largest_cluster = pcd.select_by_index(np.where(labels == largest_cluster_idx)[0])

    # 使用 RANSAC 检测平面，但减少迭代次数
    plane_model, inliers = largest_cluster.segment_plane(distance_threshold=0.05, ransac_n=3, num_iterations=100)
    inlier_cloud = largest_cluster.select_by_index(inliers)

    # 发布检测到的平面点云
    if len(inliers) > 0:
        combined_wall_points = np.asarray(inlier_cloud.points)
        header = msg.header
        ros_cloud = pc2.create_cloud_xyz32(header, combined_wall_points)
        wall_pc_pub.publish(ros_cloud)

def main():
    rospy.init_node('plane_detection')
    rospy.Subscriber('/hesai/pandar', PointCloud2, point_cloud_callback)

    global wall_pc_pub
    wall_pc_pub = rospy.Publisher('/wall_pc', PointCloud2, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()
