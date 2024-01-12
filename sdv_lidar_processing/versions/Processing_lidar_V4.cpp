#include <rclcpp/rclcpp.hpp>
// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


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


#include "obstacle_detector.hpp"


class Processing_lidar : public rclcpp::Node
{
private:
    float GROUND_THRESHOLD = 0.03;
    float CLUSTER_THRESH = 0.25;
    int CLUSTER_MAX_SIZE = 5000; 
    int CLUSTER_MIN_SIZE = 10;
    size_t obstacle_id_;


    bool USE_PCA_BOX = 0;
    float DISPLACEMENT_THRESH = 1.0;
    float IOU_THRESH = 1.0;
    bool USE_TRACKING = 1;


    std::string bbox_target_frame_;
    std::string bbox_source_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<Box> curr_boxes_;   // You need to define the 'Box' type
    std::vector<Box> prev_boxes_;


    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    std::shared_ptr<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>> obstacle_detector;

    void publishDetectedObjects(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::msg::Header& header);


    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_seg_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_detected_objects_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

public:
    Processing_lidar(/* args */);
    ~Processing_lidar();
    void initialize();
};


Processing_lidar::Processing_lidar(/* args */) : Node("Lidar_Processing_node")

{
    sub_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points_roi", 10, std::bind(&Processing_lidar::pointCloudCallback, this, std::placeholders::_1));
    ground_seg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points", 10);
    marker_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    // Create point processor
    obstacle_detector = std::make_shared<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>>();
    pub_detected_objects_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("/detected_objects", 10);
    
    obstacle_id_ = 0;
    RCLCPP_INFO(this->get_logger(), "Lidar_Processing_node initialized");
}

Processing_lidar::~Processing_lidar()
{
}

void Processing_lidar::initialize()
{
    tf2_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer, shared_from_this(), false);
}


void Processing_lidar::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *input_cloud);


    // Convert your PointXYZI cloud to PointXYZ cloud if necessary.
    pcl::PointCloud<pcl::PointXYZ>::Ptr converted_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*input_cloud, *converted_cloud);

    // Pass a const pointer (ConstPtr) to segmentPlane.
    auto segmented_clouds = obstacle_detector->segmentPlane(converted_cloud, 100, GROUND_THRESHOLD);
    auto cloud_clusters = obstacle_detector->clustering(segmented_clouds.first, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);

    // Publish the detected objects
    publishDetectedObjects(std::move(cloud_clusters), msg->header);

    std::cout << "Number of clusters: " << cloud_clusters.size() << std::endl;


    Processing_lidar::PointCloudMsg downsampled_cloud_msg;
    pcl::toROSMsg(*(segmented_clouds.first), downsampled_cloud_msg);

    ground_seg_pub_->publish(downsampled_cloud_msg);
}


void Processing_lidar::publishDetectedObjects(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::msg::Header& header)
{
    curr_boxes_.clear(); // Clear the current boxes at the beginning
    for (auto& cluster : cloud_clusters)
    {
        // Create Bounding Boxes
        Box box = USE_PCA_BOX ?
          obstacle_detector->pcaBoundingBox(cluster, obstacle_id_) :
          obstacle_detector->axisAlignedBoundingBox(cluster, obstacle_id_);

        if (obstacle_id_ < SIZE_MAX) {
            ++obstacle_id_;
        } else {
            obstacle_id_ = 0;
        }

        curr_boxes_.emplace_back(box);
    }

    // Additional code for Box ID re-assignment and frame transform...
    if (USE_TRACKING)
        obstacle_detector->obstacleTracking(prev_boxes_, curr_boxes_, DISPLACEMENT_THRESH, IOU_THRESH);

    //==================================== Drawing Boxes  ====================================

    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    const std_msgs::msg::Header& inp_header = header;


    std::cout << "Number of boxes: " << curr_boxes_.size() << std::endl;

    for (const auto& box : curr_boxes_)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = inp_header.frame_id;
        marker.header.stamp = inp_header.stamp;
        marker.ns = "obstacle_boxes";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = box.position(0);
        marker.pose.position.y = box.position(1);
        marker.pose.position.z = box.position(2);
        marker.pose.orientation.w = box.quaternion.w();
        marker.pose.orientation.x = box.quaternion.x();
        marker.pose.orientation.y = box.quaternion.y();
        marker.pose.orientation.z = box.quaternion.z();

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0; // You should set these values based on the actual size of the boxes
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;

        marker_array.markers.push_back(marker);
    }

    marker_pub->publish(marker_array);




}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Processing_lidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}