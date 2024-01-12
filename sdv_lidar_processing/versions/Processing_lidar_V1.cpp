#include <rclcpp/rclcpp.hpp>
// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// C++
#include <iostream>
#include <vector>
#include <algorithm>

// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/opencv.hpp>

// OpenCV and ROS
#include <image_geometry/pinhole_camera_model.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>




class Processing_lidar : public rclcpp::Node
{
private:



    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_segmented_cloud_;


public:
    Processing_lidar(/* args */);
    ~Processing_lidar();
};

Processing_lidar::Processing_lidar(/* args */) : Node("Lidar_Processing_node")

{
    sub_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points_roi", 10, std::bind(&Processing_lidar::pointCloudCallback, this, std::placeholders::_1));

    pub_segmented_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_cloud", 10);


    RCLCPP_INFO(this->get_logger(), "Lidar_Processing_node initialized");
}

Processing_lidar::~Processing_lidar()
{
}

void Processing_lidar::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *input_cloud);

    // RCLCPP_INFO(this->get_logger(), "working");

    // ********************************     Planner Segmentation

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_segmented_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::SACSegmentation<pcl::PointXYZI> plane_segmentor;
    pcl::ExtractIndices<pcl::PointXYZI> indices_extractor;

    plane_segmentor.setInputCloud(input_cloud);
    plane_segmentor.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentor.setMethodType(pcl::SAC_RANSAC);
    plane_segmentor.setDistanceThreshold(0.9);
    plane_segmentor.segment(*inliers, *coefficients);

    indices_extractor.setInputCloud(input_cloud);
    indices_extractor.setIndices(inliers);
    indices_extractor.setNegative(true);
    indices_extractor.filter(*plane_segmented_cloud);

    // After processing and segmenting the point cloud
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*plane_segmented_cloud, output_msg);
    output_msg.header.frame_id = msg->header.frame_id;
    output_msg.header.stamp = this->get_clock()->now();

    pub_segmented_cloud_->publish(output_msg);


}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Processing_lidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
