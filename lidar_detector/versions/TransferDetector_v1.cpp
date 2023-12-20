#include <rclcpp/rclcpp.hpp>


#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

    // Define ROI boundaries
    double roi_min_x_ = 0.1; // minimum x in meters to the back of the carrier
    double roi_max_x_ = 10.0;  // maximum x in meters to the front of the carrier
    double roi_min_y_ = -1.0; // minimum y  in meters to the left of the carrier
    double roi_max_y_ = 1.0;  // maximum y in meters to the right of the carrier

    // Detection 
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;

    vector<float> ranges;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr roi_scan_publisher_;
    laser_geometry::LaserProjection projector_;

public:
    TransferDetector(/* args */);
    ~TransferDetector();
};

TransferDetector::TransferDetector(/* args */): Node("Obstacle_avoidance_node")
{
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&TransferDetector::scan_callback, this, std::placeholders::_1));
    roi_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/roi_scan", 10);

    RCLCPP_INFO(this->get_logger(), "obstacle_avoidance_node initialized");
}

TransferDetector::~TransferDetector()
{
}


void TransferDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Scan callback working");



    sensor_msgs::msg::LaserScan roi_scan = *msg; 
    roi_scan.ranges.clear(); 

    vector<float> ranges = msg->ranges;
    size_t num_ranges = ranges.size();
    double angle = msg->angle_min;

    for (size_t i = 0; i < num_ranges; ++i, angle += msg->angle_increment) {
        float range = ranges[i];

        double x = range * cos(angle);
        double y = range * sin(angle);


        if (x >= roi_min_x_ && x <= roi_max_x_ && y >= roi_min_y_ && y <= roi_max_y_) {
            roi_scan.ranges.push_back(range);
        } else {
            roi_scan.ranges.push_back(std::numeric_limits<float>::infinity()); //inf if out of ROI
        }
    }

    // Extracting LaserScan information:
    angle_min = roi_scan.angle_min;
    angle_max = roi_scan.angle_max;
    angle_increment = roi_scan.angle_increment;
    range_min = roi_scan.range_min;
    range_max = roi_scan.range_max;
    ranges = roi_scan.ranges;

    bool prev_inf = true;
    float thresh_dist = 1.0;
    float thresh_dist_2 = 4.5;
    float thresh_dist_3 = 8.0;

    int warning_code = 0;
    for (size_t i = 0; i < ranges.size(); ++i) {
        float distance = ranges[i];

        // a obstacle is detected if the distance is less than 0.9m
        if (distance < thresh_dist) {
            if (isfinite(ranges[i])) {
                if (prev_inf == false) {
                    warning_code = 1;
                }
                prev_inf = false;
            }
        }
        // a obstacle is detected if the distance is more than 0.9 and less than 1.5m
        else if (distance < thresh_dist_2 && distance > thresh_dist && warning_code < 2) {
            if (isfinite(ranges[i]) && warning_code == 0) {
                if (prev_inf == false) {
                    warning_code = 2;
                }
                prev_inf = false;
            }
        }
        // a obstacle is detected if the distance is more than 1.5 and less than 3m
        else if (distance < thresh_dist_3 && distance > thresh_dist_2 && warning_code < 3) {
            if (isfinite(ranges[i]) && warning_code == 0) {
                if (prev_inf == false) {
                    warning_code = 3; 
                }
                prev_inf = false;
            }
        }
    }

    // Now, log the message based on the closest obstacle detected
    if (warning_code == 1) {
        RCLCPP_INFO(this->get_logger(), "[WARNING 220] Obstacle detected in less than 1.0m radius");
    } else if (warning_code == 2) {
        RCLCPP_INFO(this->get_logger(), "[WARNING 330] Obstacle detected in 1.0 to 2.5m radius");
    } else if (warning_code == 3) {
        RCLCPP_INFO(this->get_logger(), "[WARNING 550] Obstacle detected in 2.5 to 6.0m radius");
    }

    roi_scan_publisher_->publish(roi_scan); // Publish the ROI LaserScan

}



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransferDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


    // double roi_min_x_ = -2.0; // minimum x
    // double roi_max_x_ = 2.0;  // maximum x
    // double roi_min_y_ = -2.0; // minimum y
    // double roi_max_y_ = 2.0;  // maximum y
    // float angle_min;
    // float angle_max;
    // float angle_increment;
    // float range_min;
    // float range_max;

    // vector<float> ranges;


    // // Extracting LaserScan information:
    // angle_min = msg->angle_min;
    // angle_max = msg->angle_max;
    // angle_increment = msg->angle_increment;
    // range_min = msg->range_min;
    // range_max = msg->range_max;
    // ranges = msg->ranges;

    // // http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html
    // // make a cluster of points when a obstacle is detected
    // vector<std::vector<geometry_msgs::msg::Point>> clusters;
    // vector<geometry_msgs::msg::Point> current_cluster;

    // float x_prev = 0, y_prev = 0;
    // bool prev_inf = true;
    // float thresh_dist_points = 0.5;
    // float thresh_dist = 0.9;
    // int numOfClusters = 0;

    // for (size_t i = 0; i < ranges.size(); ++i) {
    //     float distance = ranges[i];
    //     float theta = angle_min + (i * angle_increment);
    //     float x_curr = ranges[i] * cos(theta);
    //     float y_curr = ranges[i] * sin(theta);
    //     geometry_msgs::msg::Point p;
    //     p.x = x_curr;
    //     p.y = y_curr;
    //     p.z = 0.0;

    //     // a obstacle is detected if the distance is less than 0.9m
    //     if (distance < thresh_dist) {
    //         if (isfinite(ranges[i])) {
    //             if (prev_inf == false) {
    //                 RCLCPP_INFO(this->get_logger(), "Obstacle detected");
    //             }
    //             x_prev = x_curr;
    //             y_prev = y_curr;
    //             prev_inf = false;

    //         }
    //     }
    // }



// Other form to cut the laser scan 

        // // Convert LaserScan to PointCloud2
    // sensor_msgs::msg::PointCloud2 cloud;
    // projector_.projectLaser(*msg, cloud);

    // // Convert PointCloud2 to PCL PointCloud
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(cloud, *pcl_cloud);

    // // Apply ROI filtering using PCL
    // pcl::CropBox<pcl::PointXYZ> roi_filter;
    // roi_filter.setMin(Eigen::Vector4f(roi_min_x_, roi_min_y_, -1.0, 1.0));
    // roi_filter.setMax(Eigen::Vector4f(roi_max_x_, roi_max_y_, 1.0, 1.0));
    // roi_filter.setInputCloud(pcl_cloud);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
    // roi_filter.filter(*cloud_roi);

    // // Convert back to PointCloud2
    // sensor_msgs::msg::PointCloud2 roi_cloud;
    // pcl::toROSMsg(*cloud_roi, roi_cloud);


    // // Initialize the new LaserScan message
    // sensor_msgs::msg::LaserScan filtered_scan;
    // filtered_scan.header = msg->header;
    // filtered_scan.angle_min = msg->angle_min;
    // filtered_scan.angle_max = msg->angle_max;
    // filtered_scan.angle_increment = msg->angle_increment;
    // filtered_scan.time_increment = msg->time_increment;
    // filtered_scan.scan_time = msg->scan_time;
    // filtered_scan.range_min = msg->range_min;
    // filtered_scan.range_max = msg->range_max;

    // // Initialize ranges with 'inf' (no detection)
    // int num_ranges = round((filtered_scan.angle_max - filtered_scan.angle_min) / filtered_scan.angle_increment);
    // filtered_scan.ranges.assign(num_ranges, std::numeric_limits<float>::infinity());

    // // Iterate over the filtered PointCloud
    // for (const auto& point : cloud_roi->points)
    // {
    //     // Convert to polar coordinates
    //     double range = sqrt(point.x * point.x + point.y * point.y);
    //     double angle = atan2(point.y, point.x);

    //     // Find the corresponding index in the LaserScan range array
    //     int index = static_cast<int>((angle - filtered_scan.angle_min) / filtered_scan.angle_increment);
        
    //     // Update the range if the point is closer
    //     if (index >= 0 && index < num_ranges && range < filtered_scan.ranges[index])
    //     {
    //         filtered_scan.ranges[index] = range;
    //     }
    // }

    // // Publish the filtered LaserScan
    // roi_scan_publisher_->publish(filtered_scan);

// This is the base 

    //     // Extracting LaserScan information:
    // angle_min = roi_scan.angle_min;
    // angle_max = roi_scan.angle_max;
    // angle_increment = roi_scan.angle_increment;
    // range_min = roi_scan.range_min;
    // range_max = roi_scan.range_max;
    // ranges = roi_scan.ranges;

    // bool prev_inf = true;
    // float thresh_dist = 1.0;
    // float thresh_dist_2 = 2.5;
    // float thresh_dist_3 = 6.0;

    // int warning_code = 0;
    // for (size_t i = 0; i < ranges.size(); ++i) {
    //     float distance = ranges[i];

    //     // a obstacle is detected if the distance is less than 0.9m
    //     if (distance < thresh_dist) {
    //         if (isfinite(ranges[i])) {
    //             if (prev_inf == false) {
    //                 warning_code = 1;
    //             }
    //             prev_inf = false;
    //         }
    //     }
    //     // a obstacle is detected if the distance is more than 0.9 and less than 1.5m
    //     else if (distance < thresh_dist_2 && distance > thresh_dist && warning_code < 2) {
    //         if (isfinite(ranges[i]) && warning_code == 0) {
    //             if (prev_inf == false) {
    //                 warning_code = 2;
    //             }
    //             prev_inf = false;
    //         }
    //     }
    //     // a obstacle is detected if the distance is more than 1.5 and less than 3m
    //     else if (distance < thresh_dist_3 && distance > thresh_dist_2 && warning_code < 3) {
    //         if (isfinite(ranges[i]) && warning_code == 0) {
    //             if (prev_inf == false) {
    //                 warning_code = 3; 
    //             }
    //             prev_inf = false;
    //         }
    //     }
    // }

    // // Now, log the message based on the closest obstacle detected
    // if (warning_code == 1) {
    //     RCLCPP_INFO(this->get_logger(), "[WARNING 220] Obstacle detected in less than 1.0m radius");
    // } else if (warning_code == 2) {
    //     RCLCPP_INFO(this->get_logger(), "[WARNING 330] Obstacle detected in 1.0 to 2.5m radius");
    // } else if (warning_code == 3) {
    //     RCLCPP_INFO(this->get_logger(), "[WARNING 550] Obstacle detected in 2.5 to 6.0m radius");
    // }

    // roi_scan_publisher_->publish(roi_scan); // Publish the ROI LaserScan
