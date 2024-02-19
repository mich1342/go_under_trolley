#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

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
    unfiltered_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_in", rclcpp::SensorDataQoS(),
        std::bind(&GoUnderTrolley::cloud_sub_callback, this, std::placeholders::_1));
    // laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     "scan", 10, std::bind(&GoUnderTrolley::scan_callback, this, std::placeholders::_1));
    filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_cloud", 10);
    target_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/target_cloud", 10);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    cmd_timer_ = this->create_wall_timer(50ms, std::bind(&GoUnderTrolley::cmd_callback, this));
  }

private:
  void cmd_callback()
  {
    if ((target_x_ != NULL) && (target_y_ != NULL))
    {
      geometry_msgs::msg::Twist vel_msg_;
      vel_msg_.linear.x = 0.00;
      vel_msg_.linear.y = 0.00;
      vel_msg_.angular.z = 0.00;
      if (target_x_ <= -0.13)
      {
        cmd_pub_->publish(vel_msg_);
        goal_reached_count_++;
        if (goal_reached_count_ >= 40)
        {
          exit(0);
        }
      }
      else
      {
        vel_msg_.linear.x = 0.15;
        if (fabs(target_y_) >= 0.05)
        {
          vel_msg_.angular.z = target_y_ * 0.6;
        }
        // else
        // {
        //   vel_msg_.angular.z = -target_y_diff_ * 0.5;
        // }
        cmd_pub_->publish(vel_msg_);
      }
    }
  }
  void cloud_sub_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // converting ros2 pointcloud message type to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud_in);

    // creating kd tree for the pointcloud
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    cloud_in->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
    tree->setInputCloud(cloud_in);

    RCLCPP_INFO(this->get_logger(), "Created Tree");

    // clustering points in point cloud for identifying the legs of items to pickup
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
    ece.setClusterTolerance(0.4);  // 20cm
    ece.setMinClusterSize(3);
    ece.setMaxClusterSize(50);
    ece.setSearchMethod(tree);
    ece.setInputCloud(cloud_in);

    // try
    // {
    ece.extract(cluster_indices);
    RCLCPP_INFO(this->get_logger(), "ece indices %ld", cluster_indices.size());

    // creating different point cloud instances based on clusters and adding different color
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;
    for (const auto& cluster : cluster_indices)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::RGB rgb = pcl::getRandomColor();
      for (const auto& idx : cluster.indices)
      {
        (*cloud_in)[idx].r = rgb.r;
        (*cloud_in)[idx].g = rgb.g;
        (*cloud_in)[idx].b = rgb.b;
        cloud_cluster->push_back((*cloud_in)[idx]);
      }
      cloud_clusters.push_back(cloud_cluster);
    }

    std::vector<std::pair<float, float>> length_points;
    std::vector<std::pair<float, float>> breadth_points;

    for (long unsigned int i = 0; i < cloud_clusters.size(); i++)
    {
      for (long unsigned int j = i + 1; j < cloud_clusters.size(); j++)
      {
        if (cloud_clusters.at(i)->size() == 0 || cloud_clusters.at(j)->size() == 0)
        {
          break;
        }

        float y1 = std::numeric_limits<float>::max();
        float x1 = std::numeric_limits<float>::max();
        float y2 = std::numeric_limits<float>::max();
        float x2 = std::numeric_limits<float>::max();

        for (long unsigned int k = 0; k < cloud_clusters.at(i)->size(); k++)
        {
          float tempX = -cloud_clusters.at(i)->at(k).y;
          if ((tempX) < x1)
          {
            x1 = tempX;
            y1 = cloud_clusters.at(i)->at(k).x;
          }
        }

        for (long unsigned int k = 0; k < cloud_clusters.at(j)->size(); k++)
        {
          float tempX = -cloud_clusters.at(j)->at(k).y;
          if ((tempX) < x2)
          {
            x2 = tempX;
            y2 = cloud_clusters.at(j)->at(k).x;
          }
        }

        if (fabs(x1) < 2.50 && fabs(x2) < 2.50)
        {
          float distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
          distance = fabs(distance);

          if (distance < 1.25 && distance > 1.05)
          {
            RCLCPP_INFO(this->get_logger(), "distance %f", distance);
            std::pair<float, float> first_point_pair;
            first_point_pair.first = x1;
            first_point_pair.second = y1;

            std::pair<float, float> second_point_pair;
            second_point_pair.first = x2;
            second_point_pair.second = y2;

            length_points.push_back(first_point_pair);
            length_points.push_back(second_point_pair);
          }
        }
        // else if (distance < 1.10 && distance > 0.90)
        // {
        //   std::pair<float, float> first_point_pair;
        //   first_point_pair.first = x1;
        //   first_point_pair.second = y1;

        //   std::pair<float, float> second_point_pair;
        //   second_point_pair.first = x2;
        //   second_point_pair.second = y2;

        //   breadth_points.push_back(first_point_pair);
        //   breadth_points.push_back(second_point_pair);
        // }
      }
    }

    RCLCPP_INFO(this->get_logger(), "length %ld", length_points.size());
    RCLCPP_INFO(this->get_logger(), "breadth %ld", breadth_points.size());
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    pcl::PointXYZRGB pt;

    if (length_points.size() == 2)
    {
      float x1 = ((length_points.at(0).first + length_points.at(1).first) / 2);
      float y1 = ((length_points.at(0).second + length_points.at(1).second) / 2);

      if (fabs(x1) < 2.50)
      {
        pt = pcl::PointXYZRGB((uint8_t)0, (uint8_t)255, (uint8_t)0);
        pt.x = x1;
        pt.y = y1;
        cloud_.points.push_back(pt);
        pt = pcl::PointXYZRGB((uint8_t)0, (uint8_t)0, (uint8_t)255);
        pt.x = length_points.at(0).first;
        pt.y = length_points.at(0).second;
        cloud_.points.push_back(pt);
        pt.x = length_points.at(1).first;
        pt.y = length_points.at(1).second;
        cloud_.points.push_back(pt);
        RCLCPP_INFO(this->get_logger(), "cent_x %f", x1);
        RCLCPP_INFO(this->get_logger(), "cent_y %f", y1);
        target_x_ = x1;
        target_y_ = y1;
        target_y_diff_ = length_points.at(0).second - length_points.at(1).second;
      }
      // float x2 = ((length_points.at(2).first + length_points.at(3).first) / 2);
      // float y2 = ((length_points.at(2).second + length_points.at(3).second) / 2);

      // float cent_x, cent_y;
      // if (fabs(x1 - x2) < fabs(y1 - y2))
      // {
      //   cent_y = ((y1 + y2) / 2);
      // }
      // else
      // {
      //   cent_x = ((x1 + x2) / 2);
      // }

      // if (length_points.size() == 4 && breadth_points.size() == 4)
      // {
      //   float x1 = ((length_points.at(0).first + length_points.at(1).first) / 2);
      //   float y1 = ((length_points.at(0).second + length_points.at(1).second) / 2);

      //   float x2 = ((length_points.at(2).first + length_points.at(3).first) / 2);
      //   float y2 = ((length_points.at(2).second + length_points.at(3).second) / 2);

      //   float x3 = ((breadth_points.at(0).first + breadth_points.at(1).first) / 2);
      //   float y3 = ((breadth_points.at(0).second + breadth_points.at(1).second) / 2);

      //   float x4 = ((breadth_points.at(2).first + breadth_points.at(3).first) / 2);
      //   float y4 = ((breadth_points.at(2).second + breadth_points.at(3).second) / 2);

      //   float cent_x, cent_y;
      //   if (fabs(x1 - x2) < fabs(y1 - y2))
      //   {
      //     cent_y = ((y1 + y2) / 2);
      //   }
      //   else
      //   {
      //     cent_x = ((x1 + x2) / 2);
      //   }

      //   if (fabs(x3 - x4) < fabs(y3 - y4))
      //   {
      //     cent_y = ((y3 + y4) / 2);
      //   }
      //   else
      //   {
      //     cent_x = ((x3 + x4) / 2);
      //   }
      //   // RCLCPP_INFO(this->get_logger(), "[0]: '%f' [100]: '%f'", laser_->ranges[0], laser_->ranges[100]);
      //   RCLCPP_INFO(this->get_logger(), "cent_x %f", cent_x);
      //   RCLCPP_INFO(this->get_logger(), "cent_y %f", cent_y);
    }
    auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud_, *pc2_msg_);
    pc2_msg_->header.frame_id = "base_link";
    pc2_msg_->header.stamp = now();
    pc2_msg_->is_dense = false;
    target_cloud_pub_->publish(*pc2_msg_);

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_out(new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(*cloud_in, *cloud_out);
    // long unsigned int num_points_out = cloud_out->width;
    cloud_out->header.frame_id = msg->header.frame_id;
    cloud_out->header.stamp = msg->header.stamp;
    filtered_cloud_->publish(*cloud_out);
    // }
    // catch (std::exception e)
    // {
    //   RCLCPP_INFO(this->get_logger(), "Nothing Found");
    // }
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr unfiltered_cloud_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  int timeout_count_ = 0;
  float target_x_ = NULL;
  float target_y_ = NULL;
  float target_y_diff_ = NULL;
  int goal_reached_count_ = 0;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoUnderTrolley>());
  rclcpp::shutdown();
  return 0;
}