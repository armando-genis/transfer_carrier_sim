#include <rclcpp/rclcpp.hpp>


#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <laser_geometry/laser_geometry.hpp>
#include <geometry_msgs/msg/point.hpp>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

// C++
#include <iostream>
#include <vector>
#include <algorithm>

// Eigen
#include <Eigen/Dense>

using namespace std;

class TransferDetector : public rclcpp::Node
{
private:
    /* data */

    // Laser Scan variables
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    vector<float> ranges;
    vector<std::vector<geometry_msgs::msg::Point>> clusters_points;

    // ROI boundaries
    double roi_min_x_ = 0.1;
    double roi_max_x_ = 10.0;
    double roi_min_y_ = -1.0;
    double roi_max_y_ = 1.0;

    // Clustering threshold
    float thresh_dist_points = 0.3; // Distance threshold for clustering
    // std::map<int, float> prev_closest_distances;



    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void process_clusters(const vector<std::vector<geometry_msgs::msg::Point>>& clusters);


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr roi_scan_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;


public:
    TransferDetector(/* args */);
    ~TransferDetector();
};

TransferDetector::TransferDetector(/* args */): Node("Obstacle_avoidance_node")
{
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&TransferDetector::scan_callback, this, std::placeholders::_1));
    roi_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/roi_scan", 10);
    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_markers", 10);


    RCLCPP_INFO(this->get_logger(), "obstacle_avoidance_node initialized");
}

TransferDetector::~TransferDetector()
{
}


void TransferDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{

    // sensor_msgs::msg::LaserScan roi_scan = *msg; 
    // roi_scan.ranges.clear(); 

    // Extracting LaserScan information
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;
    range_min = msg->range_min;
    range_max = msg->range_max;
    ranges = msg->ranges;

    // Clustering logic
    vector<std::vector<geometry_msgs::msg::Point>> clusters;
    vector<geometry_msgs::msg::Point> current_cluster;
    float x_prev = 0, y_prev = 0;
    bool prev_inf = true;
    int numOfClusters = 0; 


    for (size_t i = 0; i < ranges.size(); ++i) {
        float range = ranges[i];
        float theta = angle_min + (i * angle_increment);
        float x = range * cos(theta);
        float y = range * sin(theta);

        // Check if point is within ROI
        if (x >= roi_min_x_ && x <= roi_max_x_ && y >= roi_min_y_ && y <= roi_max_y_) {

            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.0;



            if (isfinite(range)) {
                float dist_to_prev = sqrt(pow(x - x_prev, 2) + pow(y - y_prev, 2));
                if (dist_to_prev < thresh_dist_points) {
                    current_cluster.push_back(p);
                } else {
                    if (!current_cluster.empty()) {
                        clusters.push_back(current_cluster);
                        current_cluster.clear();
                        numOfClusters++; 

                    }
                    current_cluster.push_back(p);
                }
                x_prev = x;
                y_prev = y;
                prev_inf = false;
            } else {
                if (!current_cluster.empty()) {
                    clusters.push_back(current_cluster);
                    current_cluster.clear();
                    numOfClusters++;
                }
                prev_inf = true;
            }
        }
    }

    if (!current_cluster.empty()) {
        clusters.push_back(current_cluster);
        numOfClusters++; // Increment for the last cluster if not empty
    }


    // Calculate closest point distance for each cluster

    float thresh_dist = 1.0; // Threshold for very close obstacles
    float thresh_dist_2 = 4.5; // Threshold for moderately close obstacles
    float thresh_dist_3 = 7.0; // Threshold for distant obstacles
    int highest_warning_code = 4; // To store the highest severity warning

    for (size_t i = 0; i < clusters.size(); ++i) {
        float min_distance = std::numeric_limits<float>::max();
        int warning_code = 4;

        for (const auto& point : clusters[i]) {
            float distance = sqrt(point.x * point.x + point.y * point.y);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }

        if (min_distance < thresh_dist) {
            warning_code = 1; // Very close obstacle
        } else if (min_distance >= thresh_dist && min_distance < thresh_dist_2) {
            warning_code = 2; // Moderately close obstacle
        } else if (min_distance >= thresh_dist_2 && min_distance < thresh_dist_3) {
            warning_code = 3; // Distant obstacle
        }

        if (warning_code < highest_warning_code) {
            highest_warning_code = warning_code;
        }

        // RCLCPP_INFO(this->get_logger(), "Cluster %ld closest distance: %f, Warning Code: %d", i, min_distance, warning_code);


        // RCLCPP_INFO(this->get_logger(), "Cluster %ld closest distance: %f", i, min_distance);
    }

    // Now, log the message based on the closest obstacle detected
    if (highest_warning_code == 1) {
        RCLCPP_INFO(this->get_logger(), "[WARNING 220] Obstacle detected in less than 1.0m radius");
    } else if (highest_warning_code == 2) {
        RCLCPP_INFO(this->get_logger(), "[WARNING 330] Obstacle detected in 1.0 to 4.5m radius");
    } else if (highest_warning_code == 3) {
        RCLCPP_INFO(this->get_logger(), "[WARNING 550] Obstacle detected in 4.5 to 7.0m radius");
    } else {
        RCLCPP_INFO(this->get_logger(), "[INFO 000] No obstacle detected");
    }



    // RCLCPP_INFO(this->get_logger(), "Number of clusters detected: %d", numOfClusters);

    // Process the clusters for obstacle detection and visualization
    process_clusters(clusters);
}



void TransferDetector::process_clusters(const vector<std::vector<geometry_msgs::msg::Point>>& clusters)
{
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < clusters.size(); i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "laser";
        marker.header.stamp = this->now();
        marker.ns = "clusters";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.r = static_cast<float>(rand()) / RAND_MAX; 
        marker.color.g = static_cast<float>(rand()) / RAND_MAX;
        marker.color.b = static_cast<float>(rand()) / RAND_MAX;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration(0, 500000000);

        for (const auto& point : clusters[i]) {
            marker.points.push_back(point);
        }

        marker_array.markers.push_back(marker);
    }

    marker_array_publisher_->publish(marker_array);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransferDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// // Add a member to store the closest point distances of clusters from the previous scan
// std::map<int, float> prev_closest_distances;

// void TransferDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//     // Existing LaserScan processing logic ...

//     // Calculate closest point distance for each cluster
//     std::map<int, float> current_closest_distances;
//     for (size_t i = 0; i < clusters.size(); ++i) {
//         float min_distance = std::numeric_limits<float>::max();
//         for (const auto& point : clusters[i]) {
//             float distance = sqrt(point.x * point.x + point.y * point.y);
//             if (distance < min_distance) {
//                 min_distance = distance;
//             }
//         }
//         current_closest_distances[i] = min_distance;

//         // Check if the cluster is getting closer
//         if (prev_closest_distances.find(i) != prev_closest_distances.end() &&
//             min_distance < prev_closest_distances[i]) {
//             RCLCPP_WARN(this->get_logger(), "Part of Cluster %d is getting closer!", i);
//             // Implement your response logic here
//         }
//     }

//     // Update previous closest distances for the next cycle
//     prev_closest_distances = current_closest_distances;

//     // Process the clusters...
// }



// float thresh_dist = 1.0; // Threshold for very close obstacles
// float thresh_dist_2 = 4.5; // Threshold for moderately close obstacles
// float thresh_dist_3 = 7.0; // Threshold for distant obstacles

// int highest_warning_code = 0; // To store the highest severity warning

// for (size_t i = 0; i < clusters.size(); ++i) {
//     float min_distance = std::numeric_limits<float>::max();
//     int warning_code = 0;
//     for (const auto& point : clusters[i]) {
//         float distance = sqrt(point.x * point.x + point.y * point.y);
//         if (distance < min_distance) {
//             min_distance = distance;
//         }
//     }

//     if (min_distance < thresh_dist) {
//         warning_code = 1; // Very close obstacle
//     } else if (min_distance >= thresh_dist && min_distance < thresh_dist_2) {
//         warning_code = 2; // Moderately close obstacle
//     } else if (min_distance >= thresh_dist_2 && min_distance < thresh_dist_3) {
//         warning_code = 3; // Distant obstacle
//     }

//     if (warning_code < highest_warning_code) {
//         highest_warning_code = warning_code;
//     }

//     RCLCPP_INFO(this->get_logger(), "Cluster %ld closest distance: %f, Warning Code: %d", i, min_distance, warning_code);
// }

// // Now, log the highest severity warning
// if (highest_warning_code == 1) {
//     RCLCPP_INFO(this->get_logger(), "[WARNING 220] Obstacle detected in less than 1.0m radius");
// } else if (highest_warning_code == 2) {
//     RCLCPP_INFO(this->get_logger(), "[WARNING 330] Obstacle detected in 1.0 to 4.5m radius");
// } else if (highest_warning_code == 3) {
//     RCLCPP_INFO(this->get_logger(), "[WARNING 550] Obstacle detected in 4.5 to 7.0m radius");
// }
