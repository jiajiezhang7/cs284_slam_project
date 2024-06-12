#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 将ROS 点云消息转换为PCL 点云
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // 对点云进行下采样
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(cloud);
    vox_grid.setLeafSize(0.05f, 0.05f, 0.05f);
    vox_grid.filter(*cloud);

    // 检测平面
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.10);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // 判断平面是否为水平面（地面）
    if (fabs(coefficients->values[0]) < 0.1 && fabs(coefficients->values[1]) < 0.1 && fabs(coefficients->values[2] - 1.0) < 0.1) {
        return; // 跳过地面平面
    }

    // 从点云中移除检测到的平面
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setNegative(true);
    extract.setIndices(inliers);
    extract.filter(*cloud);

    // 收集属于垂直平面的点云
    wall_points.insert(wall_points.end(), inliers->indices.begin(), inliers->indices.end());

    // 发布合并的属于垂直平面的点云
    if (!wall_points.empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, wall_points, *wall_cloud);
        pcl::toROSMsg(*wall_cloud, wall_msg);
        wall_msg.header = msg->header;
        wall_pub.publish(wall_msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_detection");
    ros::NodeHandle nh;

    // 创建TF 缓存和监听器
    tf2_ros::Buffer tf_buffer(nh);
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // 创建订阅器和同步器
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub(nh, "/hesai/pandar", 10);
    message_filters::ApproximateTimeSynchronizer<sensor_msgs::PointCloud2> sync(
        {sub}, 10, 0.1);
    sync.registerCallback(boost::bind(&cloudCallback, _1));

    // 创建发布器
    ros::Publisher wall_pub = nh.advertise<sensor_msgs::PointCloud2>("/wall_pc", 10);

    ros::spin();
    return 0;
}
