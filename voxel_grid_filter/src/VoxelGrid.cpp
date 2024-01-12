// #pragma once

#include <rclcpp/rclcpp.hpp>

// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>

// C++
#include <iostream>
#include <vector>
#include <algorithm>

// Eigen
#include <Eigen/Dense>

using namespace std;

class VoxelGrid : public rclcpp::Node
{
private:

    // voxelgrid resolution
    float voxel_leaf_size_x_ = 0.2;
    float voxel_leaf_size_y_ = 0.05;
    float voxel_leaf_size_z_ = 0.1; 
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using PointCloudMsg2 = sensor_msgs::msg::PointCloud2;

    
    // ROI boundaries
    // The velodyne is mounted on the car with a bad orientation, so the x is the y axis. 
    double roi_max_x_ = 0.1; //FRONT THE CAR
    double roi_max_y_ = 1.0;  //LEFT THE CAR
    double roi_max_z_ = 0.1; //UP THE VELODYNE

    double roi_min_x_ = -10.0; //RIGHT THE CAR 
    double roi_min_y_ = -1.0; //BACK THE CAR
    double roi_min_z_ = -2.0; //DOWN THE VELODYNE

    Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr down_sampling_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_sampling_pub_;


    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);


public:
    VoxelGrid(/* args */);
    ~VoxelGrid();
};

VoxelGrid::VoxelGrid(/* args */): Node("voxel_grid_node")
{
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_raw", 10, std::bind(&VoxelGrid::pointCloudCallback, this, std::placeholders::_1));

    down_sampling_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_down_sampling", 10);
    roi_sampling_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_roi", 10);

    ROI_MAX_POINT = Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1);
    ROI_MIN_POINT = Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1);

    RCLCPP_INFO(this->get_logger(), "waypoints_creator_node initialized");

}

VoxelGrid::~VoxelGrid()
{
}

void VoxelGrid::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    // =========================== MAKE A DOWN SAMPLYNG VOXEL GRID

    // pcl::PCLPointCloud2 pcl_pc2;
    // pcl_conversions::toPCL(*msg, pcl_pc2);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    // // Perform the voxel grid down sampling
    // pcl::VoxelGrid<pcl::PointXYZ> sor;
    // sor.setInputCloud(temp_cloud);
    // sor.setLeafSize(voxel_leaf_size_x_, voxel_leaf_size_y_, voxel_leaf_size_z_);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // sor.filter(*cloud_filtered);
    // // Convert the pcl/PointCloud data back to sensor_msgs/PointCloud2
    // sensor_msgs::msg::PointCloud2 output;
    // pcl::toROSMsg(*cloud_filtered, output);
    // output.header.frame_id = msg->header.frame_id;  

    // // Publish the down sampled data
    // down_sampling_pub_->publish(output);

    // =========================== MAKE A DOWN SAMPLYNG FIST AND THE ROI FILTERING

    // pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
    //     new pcl::PointCloud<pcl::PointXYZI>);

    // pcl::fromROSMsg(*msg, *input_cloud);

    // // create voxel grid object
    // pcl::VoxelGrid<pcl::PointXYZI> vg;
    // vg.setInputCloud(input_cloud);
    // vg.setLeafSize(voxel_leaf_size_x_, voxel_leaf_size_y_, voxel_leaf_size_z_);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // vg.filter(*filtered_cloud);

    // // convert back to ROS datatype
    // VoxelGrid::PointCloudMsg downsampled_cloud_msg;
    // pcl::toROSMsg(*filtered_cloud, downsampled_cloud_msg);

    // down_sampling_pub_->publish(downsampled_cloud_msg);

    // // Apply ROI filtering
    // pcl::CropBox<pcl::PointXYZI> roi_filter;
    // roi_filter.setInputCloud(filtered_cloud);
    // roi_filter.setMax(ROI_MAX_POINT);
    // roi_filter.setMin(ROI_MIN_POINT);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);
    // roi_filter.filter(*cloud_roi);

    // VoxelGrid::PointCloudMsg2 downsampled_cloud_msg_rio;
    // pcl::toROSMsg(*cloud_roi, downsampled_cloud_msg_rio);

    // roi_sampling_pub_->publish(downsampled_cloud_msg_rio);

    // ============================= MAKE A ROY FILTER AND THEN THE DOWNSAMPLING OF VOXEL GRID

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *input_cloud);

    // Apply ROI filtering
    pcl::CropBox<pcl::PointXYZI> roi_filter;
    roi_filter.setInputCloud(input_cloud);
    roi_filter.setMax(ROI_MAX_POINT);
    roi_filter.setMin(ROI_MIN_POINT);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);
    roi_filter.filter(*cloud_roi);

    VoxelGrid::PointCloudMsg2 downsampled_cloud_msg_rio;
    pcl::toROSMsg(*cloud_roi, downsampled_cloud_msg_rio);

    roi_sampling_pub_->publish(downsampled_cloud_msg_rio);

    // create voxel grid object
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(input_cloud);
    vg.setLeafSize(voxel_leaf_size_x_, voxel_leaf_size_y_, voxel_leaf_size_z_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    vg.filter(*filtered_cloud);

    // convert back to ROS datatype
    VoxelGrid::PointCloudMsg downsampled_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, downsampled_cloud_msg);

    down_sampling_pub_->publish(downsampled_cloud_msg);

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoxelGrid>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
