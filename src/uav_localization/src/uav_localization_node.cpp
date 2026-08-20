#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class UavLocalizationNode : public rclcpp::Node {
public:
    UavLocalizationNode() : Node("uav_localization") {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/uav/odom", 10);
        vision_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10, std::bind(&UavLocalizationNode::pc_callback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        icp_.setMaxCorrespondenceDistance(2.0);
        icp_.setMaximumIterations(30);
        icp_.setTransformationEpsilon(1e-6);
        icp_.setEuclideanFitnessEpsilon(1e-6);
        
        current_pose_ = Eigen::Matrix4f::Identity();
        RCLCPP_INFO(this->get_logger(), "Lightweight ICP Localization Node Started.");
    }

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        
        if (!prev_cloud_) {
            prev_cloud_ = cloud;
            return;
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
        icp_.setInputSource(cloud);
        icp_.setInputTarget(prev_cloud_);
        icp_.align(*aligned);
        
        if (icp_.hasConverged()) {
            Eigen::Matrix4f transform = icp_.getFinalTransformation();
            current_pose_ = current_pose_ * transform.inverse();
        }
        
        // Very simplistic map management - just keep the last frame
        prev_cloud_ = cloud;
        
        // Publish Odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        
        odom.pose.pose.position.x = current_pose_(0, 3);
        odom.pose.pose.position.y = current_pose_(1, 3);
        odom.pose.pose.position.z = current_pose_(2, 3);
        
        Eigen::Matrix3f rot_mat = current_pose_.block<3,3>(0,0);
        Eigen::Quaternionf q(rot_mat);
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        
        odom_pub_->publish(odom);
        
        geometry_msgs::msg::PoseStamped vision_pose;
        vision_pose.header = odom.header;
        vision_pose.pose = odom.pose.pose;
        vision_pub_->publish(vision_pose);
        
        // Publish TF
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = msg->header.stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = odom.pose.pose.position.x;
        tf.transform.translation.y = odom.pose.pose.position.y;
        tf.transform.translation.z = odom.pose.pose.position.z;
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
    Eigen::Matrix4f current_pose_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UavLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
