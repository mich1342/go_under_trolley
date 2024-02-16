#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class GoUnderTrolley : public rclcpp::Node
{
public:
  GoUnderTrolley() : Node("go_under_trolley")
  {
    status_pub_ = this->create_publisher<std_msgs::msg::String>("trolley_status", 10);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    target_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);

    status_timer_ = this->create_wall_timer(500ms, std::bind(&GoUnderTrolley::status_timer_callback, this));

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "dummy_command", 10, std::bind(&GoUnderTrolley::command_callback, this, std::placeholders::_1));
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&GoUnderTrolley::scan_callback, this, std::placeholders::_1));

    laser_ = std::make_shared<sensor_msgs::msg::LaserScan>();

    status_msg_.data = "Idle";
    vel_msg_.linear.x = 0.0;
    vel_msg_.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Node Begin");
  }

private:
  void status_timer_callback()
  {
    status_pub_->publish(status_msg_);
    twist_pub_->publish(vel_msg_);
    visMsg();
    target_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "[0]: '%f' [100]: '%f'", laser_->ranges[0], laser_->ranges[100]);
  }

  void visMsg()
  {
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = rclcpp::Time();
    marker.ns = "target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 1.0;
    marker.pose.position.y = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  void command_callback(const std_msgs::msg::String::SharedPtr _command_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Command Callback");
    if (_command_msg->data == "Idle")
    {
      vel_msg_.linear.x = 0.0;
      vel_msg_.angular.z = 0.0;
    }
    else if (_command_msg->data == "Forward")
    {
      vel_msg_.linear.x = 0.4;
      vel_msg_.angular.z = 0.0;
    }
    else if (_command_msg->data == "Turn Right")
    {
      vel_msg_.linear.x = 0.0;
      vel_msg_.angular.z = 0.2;
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
  {
    laser_ = _msg;
  }
  std_msgs::msg::String status_msg_;
  geometry_msgs::msg::Twist vel_msg_;
  visualization_msgs::msg::Marker marker;
  sensor_msgs::msg::LaserScan::SharedPtr laser_;

  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoUnderTrolley>());
  rclcpp::shutdown();
  return 0;
}