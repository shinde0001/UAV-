#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

class LocalObstacleFilter : public rclcpp::Node {
public:
    LocalObstacleFilter() : Node("local_obstacle_filter") {
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/uav/local_obstacles", 10);
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10, std::bind(&LocalObstacleFilter::pc_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Local Obstacle Filter Started.");
    }

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        
        // 1. Voxel Grid Filter (0.5m resolution to save processing)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.5f, 0.5f, 0.5f);
        vg.filter(*cloud_downsampled);
        
        // 2. CropBox Filter (Keep only 50x50x20m around UAV, excluding ground below z = -1.5m)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::CropBox<pcl::PointXYZ> cb;
        cb.setMin(Eigen::Vector4f(-25.0, -25.0, -1.5, 1.0)); // Filter out ground
        cb.setMax(Eigen::Vector4f(25.0, 25.0, 10.0, 1.0));
        cb.setInputCloud(cloud_downsampled);
        cb.filter(*cloud_filtered);
        
        // Convert back and publish
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*cloud_filtered, out_msg);
        out_msg.header = msg->header;
        pc_pub_->publish(out_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalObstacleFilter>());
    rclcpp::shutdown();
    return 0;
}
