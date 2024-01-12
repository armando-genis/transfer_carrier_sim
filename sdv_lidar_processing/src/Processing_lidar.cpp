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
    void publishDetectedObjectsversion2(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::msg::Header& header);


    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_seg_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_detected_objects_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_next;


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

    marker_pub_next =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array_next", 10);
        
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

    publishDetectedObjectsversion2(std::move(cloud_clusters), msg->header);


    std::cout << "Number of clustersasdasd: " << cloud_clusters.size() << std::endl;


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



void Processing_lidar::publishDetectedObjectsversion2(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::msg::Header& header)
{
    struct BBox
    {
        float x_min;
        float x_max;
        float y_min;
        float y_max;
        float z_min;
        float z_max;
        double r = 1.0;
        double g = 0.0;
        double b = 0.0;
    };
    
    std::vector<BBox> bboxes;
    
    size_t min_reasonable_size = 50;
    size_t max_reasonable_size = 1900;
    int num_reasonable_clusters = 0;
    
    for (auto& cluster : cloud_clusters)
    {
        if (cluster->size() > min_reasonable_size && cluster->size() < max_reasonable_size)
        {
            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D<pcl::PointXYZ>(*cluster, min_pt, max_pt);

            BBox bbox;
            bbox.x_min = min_pt[0];
            bbox.y_min = min_pt[1];
            bbox.z_min = min_pt[2];
            bbox.x_max = max_pt[0];
            bbox.y_max = max_pt[1];
            bbox.z_max = max_pt[2];

            bboxes.push_back(bbox);
            num_reasonable_clusters++;
        }
    }

    std::cout << "Number of reasonable clusters: " << num_reasonable_clusters << std::endl;

    //==================================== Drawing Boxes  ====================================

    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    const std_msgs::msg::Header& inp_header = header;
    // Create a marker for each bounding box
    for (const auto& bbox : bboxes)
    {
        // Create the marker for the top square
        visualization_msgs::msg::Marker top_square_marker;
        top_square_marker.header = inp_header;
        top_square_marker.ns = "bounding_boxes";
        top_square_marker.id = id++;
        top_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        top_square_marker.action = visualization_msgs::msg::Marker::ADD;
        top_square_marker.pose.orientation.w = 1.0;
        top_square_marker.scale.x = 0.06;
        top_square_marker.color.r = bbox.r;
        top_square_marker.color.g = bbox.g;
        top_square_marker.color.b = bbox.b;
        top_square_marker.color.a = 1.0;

        // Add the points to the top square marker
        geometry_msgs::msg::Point p1, p2, p3, p4;
        p1.x = bbox.x_max; p1.y = bbox.y_max; p1.z = bbox.z_max;
        p2.x = bbox.x_min; p2.y = bbox.y_max; p2.z = bbox.z_max;
        p3.x = bbox.x_min; p3.y = bbox.y_min; p3.z = bbox.z_max;
        p4.x = bbox.x_max; p4.y = bbox.y_min; p4.z = bbox.z_max;
        top_square_marker.points.push_back(p1);
        top_square_marker.points.push_back(p2);
        top_square_marker.points.push_back(p3);
        top_square_marker.points.push_back(p4);
        top_square_marker.points.push_back(p1);

        // Add the top square marker to the array
        marker_array.markers.push_back(top_square_marker);

        // Create the marker for the bottom square
        visualization_msgs::msg::Marker bottom_square_marker;
        bottom_square_marker.header = inp_header;
        bottom_square_marker.ns = "bounding_boxes";
        bottom_square_marker.id = id++;
        bottom_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        bottom_square_marker.action = visualization_msgs::msg::Marker::ADD;
        bottom_square_marker.pose.orientation.w = 1.0;
        bottom_square_marker.scale.x = 0.04;
        bottom_square_marker.color.r = bbox.r;
        bottom_square_marker.color.g = bbox.g;
        bottom_square_marker.color.b = bbox.b;
        bottom_square_marker.color.a = 1.0;

        // Add the points to the bottom square marker
        geometry_msgs::msg::Point p5, p6, p7, p8;
        p5.x = bbox.x_max; p5.y = bbox.y_max; p5.z = bbox.z_min;
        p6.x = bbox.x_min; p6.y = bbox.y_max; p6.z = bbox.z_min;
        p7.x = bbox.x_min; p7.y = bbox.y_min; p7.z = bbox.z_min;
        p8.x = bbox.x_max; p8.y = bbox.y_min; p8.z = bbox.z_min;

        bottom_square_marker.points.push_back(p5);
        bottom_square_marker.points.push_back(p6);
        bottom_square_marker.points.push_back(p7);
        bottom_square_marker.points.push_back(p8);
        bottom_square_marker.points.push_back(p5); // connect the last point to the first point to close the square

        // Add the bottom square marker to the marker array
        marker_array.markers.push_back(bottom_square_marker);


        // Create the marker for the lines connecting the top and bottom squares
        visualization_msgs::msg::Marker connecting_lines_marker;
        connecting_lines_marker.header = inp_header;
        connecting_lines_marker.ns = "bounding_boxes";
        connecting_lines_marker.id = id++;
        connecting_lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        connecting_lines_marker.action = visualization_msgs::msg::Marker::ADD;
        connecting_lines_marker.pose.orientation.w = 1.0;
        connecting_lines_marker.scale.x = 0.04;
        connecting_lines_marker.color.r = 0.0;
        connecting_lines_marker.color.g = 1.0;
        connecting_lines_marker.color.b = 0.0;
        connecting_lines_marker.color.a = 1.0;

        // Add the points to the connecting lines marker
        connecting_lines_marker.points.push_back(p1);
        connecting_lines_marker.points.push_back(p5);

        connecting_lines_marker.points.push_back(p2);
        connecting_lines_marker.points.push_back(p6);

        connecting_lines_marker.points.push_back(p3);
        connecting_lines_marker.points.push_back(p7);

        connecting_lines_marker.points.push_back(p4);
        connecting_lines_marker.points.push_back(p8);

        // Add the connecting lines marker to the marker array
        marker_array.markers.push_back(connecting_lines_marker);


        // Create a marker for the corners
        visualization_msgs::msg::Marker corner_marker;
        corner_marker.header = inp_header;
        corner_marker.ns = "bounding_boxes";
        corner_marker.id = id++;
        corner_marker.type = visualization_msgs::msg::Marker::SPHERE;
        corner_marker.action = visualization_msgs::msg::Marker::ADD;
        corner_marker.pose.orientation.w = 1.0;
        corner_marker.scale.x = 0.4;
        corner_marker.scale.y = 0.4;
        corner_marker.scale.z = 0.4;
        corner_marker.color.r = bbox.r;
        corner_marker.color.g = 0.2;
        corner_marker.color.b = 0.5;
        corner_marker.color.a = 0.64;

        // Create a sphere for each corner and add it to the marker array

        corner_marker.pose.position = p1;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p2;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p3;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p4;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p5;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p6;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p7;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p8;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        marker_pub_next->publish(marker_array);
    }



}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Processing_lidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}