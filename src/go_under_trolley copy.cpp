#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <vector>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/colors.h>

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

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("valid_res", 10);
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
    // visMsg();

    // RCLCPP_INFO(this->get_logger(), "[0]: '%f' [100]: '%f'", laser_->ranges[0], laser_->ranges[100]);
  }

  void visMsg(float x, float y)
  {
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = rclcpp::Time();
    marker.ns = "target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
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
    if (!MIN_INDEX && !MAX_INDEX)
    {
      MIN_INDEX = std::round(MIN_ANGLE) / _msg->angle_increment;
      MAX_INDEX = std::round(MAX_ANGLE) / _msg->angle_increment;
    }

    std::vector<std::array<float, 2>> points;
    for (long i = MIN_INDEX; i <= (MAX_INDEX); i++)
    {
      points.push_back({ _msg->ranges[i] * std::cos(i * _msg->angle_increment),
                         _msg->ranges[i] * std::sin(i * _msg->angle_increment) });
      // for (long j = i; j <= (i + INDEX_DEPTH - 2); j++)
      // {
      //   float x1 = msg_->ranges[i] * std::cos(i * msg_->angle_increment);
      //   float y1 = msg_->ranges[i] * std::sin(i * msg_->angle_increment);

      //   float x2 = msg_->ranges[j + 2] * std::cos((j + 2) * msg_->angle_increment);
      //   float y2 = msg_->ranges[j + 2] * std::sin((j + 2) * msg_->angle_increment);
      // }
    }
    std::vector<std::array<long, 2>> valid_res;
    for (int i = 0; i <= points.size() - INDEX_DEPTH; i++)
    {
      for (int j = i + SKIP_INDEX; j < (i + INDEX_DEPTH); j++)
      {
        float x1 = points[i][0];
        float y1 = points[i][1];
        float x2 = points[j][0];
        float y2 = points[j][1];

        if (std::fabs(y2 - y1) <= Y_TOLERANCES)
        {
          if (std::fabs(FIND_DIST(x1, y1, x2, y2) - X_TARGET) <= X_TOLERANCES)
          {
            // Check for flat surfaces
            std::vector<float> temp_y;
            bool invalid = false;
            for (int k = i + 1; k < j; k++)
            {
              if (std::abs(points[k][1] - ((y1 + y1) / 2)) <= MIN_DEPTH)
              {
                invalid = true;
              }
            }

            if (!invalid)
            {
              valid_res.push_back({ i, j });
            }
          }
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "Got %ld valid result", valid_res.size());
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    pcl::PointXYZRGB pt;
    pt = pcl::PointXYZRGB((uint8_t)0, (uint8_t)255, (uint8_t)0);
    for (auto s : valid_res)
    {
      int i = s[0];
      int j = s[1];
      pt.x = points[i][1];
      pt.y = -points[i][0];
      cloud_.points.push_back(pt);
      pt.x = points[j][1];
      pt.y = -points[j][0];
      cloud_.points.push_back(pt);
      // visMsg(points[i][0], points[i][1]);
      // visMsg(points[j][0], points[j][1]);
    }
    auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud_, *pc2_msg_);
    pc2_msg_->header.frame_id = "base_link";
    pc2_msg_->header.stamp = now();
    pc2_msg_->is_dense = false;
    cloud_pub_->publish(*pc2_msg_);
    // cloud_pub_->publish(cloud_);
    // target_pub_->publish(marker);
  }
  float FIND_DIST(float x1, float y1, float x2, float y2)
  {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }
  const float MIN_ANGLE = 0.00 * M_PI / 180;
  const float MAX_ANGLE = 180.00 * M_PI / 180;
  const float MIN_DEPTH = 0.75;
  const int INDEX_DEPTH = 60;
  const int SKIP_INDEX = 5;
  const float Y_TOLERANCES = 0.2;
  const float X_TARGET = 1.2;
  const float X_TOLERANCES = 0.05;
  long MIN_INDEX, MAX_INDEX;
  std_msgs::msg::String status_msg_;
  geometry_msgs::msg::Twist vel_msg_;
  visualization_msgs::msg::Marker marker;
  sensor_msgs::msg::LaserScan::SharedPtr laser_;

  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr unfiltered_cloud_sub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoUnderTrolley>());
  rclcpp::shutdown();
  return 0;
}