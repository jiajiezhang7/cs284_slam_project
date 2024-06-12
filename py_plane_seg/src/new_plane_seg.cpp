// 用于Real World，可以实现Realtime

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher wall_pc_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 将ROS PointCloud2消息转换为PCL点云
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->points.empty()) {
        return;  // 无点云数据时返回
    }

    // 对点云进行下采样，减少噪声
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel_filter.filter(*cloud);

    // 设置平面检测参数
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.10);
    seg.setMaxIterations(500);

    int planes_detected = 0;
    PointCloud::Ptr combined_wall_points(new PointCloud);

    while (true) {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < 500) {
            break;
        }

        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];
        float horizontal_threshold = 0.1;

        if (std::abs(a) < horizontal_threshold && std::abs(b) < horizontal_threshold && std::abs(c - 1.0) < horizontal_threshold) {
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloud);
            continue;
        }

        planes_detected++;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        PointCloud::Ptr inlier_cloud(new PointCloud);
        extract.filter(*inlier_cloud);

        *combined_wall_points += *inlier_cloud;

        extract.setNegative(true);
        extract.filter(*cloud);
    }

    ROS_INFO("Detected Vertical Planes: %d", planes_detected);

    if (!combined_wall_points->points.empty()) {
        sensor_msgs::PointCloud2 combined_wall_cloud;
        pcl::toROSMsg(*combined_wall_points, combined_wall_cloud);
        combined_wall_cloud.header = msg->header;
        wall_pc_pub.publish(combined_wall_cloud);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_detection");
    ros::NodeHandle nh;

    ros::Subscriber point_cloud_sub = nh.subscribe("/hesai/pandar", 10, pointCloudCallback);
    wall_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/wall_pc", 10);

    ros::spin();

    return 0;
}