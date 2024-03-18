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
    initialize_params();
    refresh_params();
    unfiltered_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_action", rclcpp::SensorDataQoS(),
        std::bind(&GoUnderTrolley::cloud_sub_callback, this, std::placeholders::_1));
    go_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/go_under_trolley", 10, std::bind(&GoUnderTrolley::go_sub_callback, this, std::placeholders::_1));
    // laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     "scan", 10, std::bind(&GoUnderTrolley::scan_callback, this, std::placeholders::_1));
    filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_cloud", 10);
    target_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/target_cloud", 10);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // cmd_timer_ = this->create_wall_timer(50ms, std::bind(&GoUnderTrolley::cmd_callback, this));
  }

private:
  void cmd_callback()
  {
    if ((target_x_ != NULL) && (target_y_ != NULL) && (pre_target_x_ != NULL) && (pre_target_y_ != NULL) &&
        (go_ || bypass_go_))
    {
      geometry_msgs::msg::Twist vel_msg_;
      vel_msg_.linear.x = 0.00;
      vel_msg_.linear.y = 0.00;
      vel_msg_.angular.z = 0.00;
      if (pre_position_achieved_)
      {
        if (heading_achieved_)
        {
          if (target_x_ <= -postMovementOffset_)
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
            vel_msg_.linear.x = movementLinearVel_;
            if (fabs(target_y_) >= finePositioningTolerance_)
            {
              vel_msg_.angular.z = target_y_ * angularVelGain_;
            }
            else
            {
              vel_msg_.angular.z = -target_y_diff_ * angularVelGain_;
            }
            if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
              vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
            cmd_pub_->publish(vel_msg_);
          }
        }
        else
        {
          if (fabs(target_y_diff_) >= finePositioningTolerance_)
          {
            vel_msg_.linear.x = 0;
            vel_msg_.angular.z = -target_y_diff_ * angularVelGain_;
            if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
              vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
            cmd_pub_->publish(vel_msg_);
          }
          else
          {
            cmd_pub_->publish(vel_msg_);
            heading_achieved_ = true;
            go_ = false;
          }
        }
      }
      else
      {
        if ((pre_target_x_ <= coarsePositioningTolerance_) && (fabs(pre_target_y_) < coarsePositioningTolerance_))
        {
          cmd_pub_->publish(vel_msg_);
          pre_position_achieved_ = true;
          go_ = false;
        }
        else
        {
          if (fabs(pre_target_y_) >= coarsePositioningTolerance_)
          {
            vel_msg_.angular.z = pre_target_y_ * angularVelGain_;
            vel_msg_.linear.x = 0.0;
          }
          else
          {
            if (pre_target_x_ > 0)
            {
              vel_msg_.linear.x = movementLinearVel_;
            }
            else
            {
              vel_msg_.linear.x = -movementLinearVel_;
            }
            vel_msg_.angular.z = pre_target_y_ * angularVelGain_;
          }
          if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
            vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
          // cmd_pub_->publish(vel_msg_);
        }
      }
    }
  }
  void go_sub_callback(std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "GO")
    {
      go_ = true;
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

    // RCLCPP_INFO(this->get_logger(), "Created Tree");

    // clustering points in point cloud for identifying the legs of items to pickup
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
    RCLCPP_INFO(this->get_logger(), "Created Tree - %d %d %f", minClusterSize_, maxClusterSize_, clusterTolerance_);
    ece.setClusterTolerance(clusterTolerance_);  // 20cm
    ece.setMinClusterSize(minClusterSize_);
    ece.setMaxClusterSize(maxClusterSize_);
    ece.setSearchMethod(tree);
    ece.setInputCloud(cloud_in);

    // try
    // {
    ece.extract(cluster_indices);
    // RCLCPP_INFO(this->get_logger(), "ece indices %ld", cluster_indices.size());

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

        if (
              fabs(x1) < minTrolleyDistance_  
              && fabs(x2) < minTrolleyDistance_  
              && fabs(y1) < 1.5 
              && fabs(y2) < 1.5
            )
        {
          float distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
          distance = fabs(distance);

          if (distance < maxTrolleyGap_ && distance > minTrolleyGap_)
          {
            
            RCLCPP_INFO(this->get_logger(), "distance %f at %f,%f and %f,%f", distance, x1, y1, x2, y2);
            
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
      }
    }

    RCLCPP_INFO(this->get_logger(), "length %ld", length_points.size());
    // RCLCPP_INFO(this->get_logger(), "breadth %ld", breadth_points.size());
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    pcl::PointXYZRGB pt;

    if (length_points.size() == 2)
    {
      float x1 = ((length_points.at(0).first + length_points.at(1).first) / 2);
      float y1 = ((length_points.at(0).second + length_points.at(1).second) / 2);

      if (fabs(x1) < minTrolleyDistance_)
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
        // RCLCPP_INFO(this->get_logger(), "cent_x %f", x1);
        // RCLCPP_INFO(this->get_logger(), "cent_y %f", y1);
        target_x_ = x1;
        target_y_ = y1;

        if (length_points.at(0).second > length_points.at(1).second)
        {
          target_y_diff_ = length_points.at(0).first - length_points.at(1).first;
        }
        else
        {
          target_y_diff_ = length_points.at(1).first - length_points.at(0).first;
        }

        float dx = target_y_diff_ * prePositionDistance_ / trolleyGap_;
        float dy = (std::sqrt(prePositionDistance_ * prePositionDistance_ - dx * dx));

        pre_target_x_ = target_x_ - dy;
        pre_target_y_ = target_y_ + dx;
        RCLCPP_INFO(this->get_logger(), "pre_cent_x %f", pre_target_x_);
        RCLCPP_INFO(this->get_logger(), "pre_cent_y %f", pre_target_y_);
        pt = pcl::PointXYZRGB((uint8_t)255, (uint8_t)0, (uint8_t)255);
        pt.x = pre_target_x_;
        pt.y = pre_target_y_;
        cloud_.points.push_back(pt);
        pt = pcl::PointXYZRGB((uint8_t)0, (uint8_t)255, (uint8_t)255);
        pt.x = 0;
        pt.y = 0;
        cloud_.points.push_back(pt);
      }
    }
    auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud_, *pc2_msg_);
    pc2_msg_->header.frame_id = "base_link";
    pc2_msg_->header.stamp = now();
    pc2_msg_->is_dense = false;
    target_cloud_pub_->publish(*pc2_msg_);
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_out(new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(*cloud_in, *cloud_out);
    cloud_out->header.frame_id = msg->header.frame_id;
    cloud_out->header.stamp = msg->header.stamp;
    filtered_cloud_->publish(*cloud_out);
  }

  void initialize_params()
  {
    // Euclidean Cluster Extraction Related
    this->declare_parameter("minClusterSize", 3);
    this->declare_parameter("maxClusterSize", 50);
    this->declare_parameter("clusterTolerance", 0.45);

    // Trolley Related
    this->declare_parameter("minTrolleyDistance", 2.50);
    this->declare_parameter("minTrolleyGap", 1.35);
    this->declare_parameter("maxTrolleyGap", 1.45);
    this->declare_parameter("trolleyGap", 1.40);

    // Movement Related
    this->declare_parameter("prePositionDistance", 1.20);
    this->declare_parameter("postMovementOffset", 0.10);
    this->declare_parameter("movementLinearVel", 0.15);
    this->declare_parameter("maxAngularVel", 0.5);
    this->declare_parameter("angularVelGain", 0.6);
    this->declare_parameter("coarsePositioningTolerance", 0.05);
    this->declare_parameter("finePositioningTolerance", 0.03);
  }

  void refresh_params()
  {
    // Euclidean Cluster Extraction Related
    this->get_parameter_or<int>("minClusterSize", minClusterSize_, 3);
    this->get_parameter_or<int>("maxClusterSize", maxClusterSize_, 50);
    this->get_parameter_or<float>("clusterTolerance", clusterTolerance_, 0.45);

    // Trolley Related
    this->get_parameter_or<float>("minTrolleyDistance", minTrolleyDistance_, 2.50);
    this->get_parameter_or<float>("minTrolleyGap", minTrolleyGap_, 1.35);
    this->get_parameter_or<float>("maxTrolleyGap", maxTrolleyGap_, 1.45);
    this->get_parameter_or<float>("trolleyGap", trolleyGap_, 1.40);

    // Movement Related
    this->get_parameter_or<float>("prePositionDistance", prePositionDistance_, 1.20);
    this->get_parameter_or<float>("postMovementOffset", postMovementOffset_, 0.10);
    this->get_parameter_or<float>("movementLinearVel", movementLinearVel_, 0.15);
    this->get_parameter_or<float>("maxAngularVel", maxAngularVel_, 0.5);
    this->get_parameter_or<float>("angularVelGain", angularVelGain_, 0.6);
    this->get_parameter_or<float>("coarsePositioningTolerance", coarsePositioningTolerance_, 0.05);
    this->get_parameter_or<float>("finePositioningTolerance", finePositioningTolerance_, 0.03);
  }

  // Params Var
  int minClusterSize_;
  int maxClusterSize_;
  float clusterTolerance_;

  float minTrolleyDistance_;
  float minTrolleyGap_;
  float maxTrolleyGap_;
  float trolleyGap_;

  float prePositionDistance_;
  float postMovementOffset_;
  float movementLinearVel_;
  float maxAngularVel_;
  float angularVelGain_;
  float coarsePositioningTolerance_;
  float finePositioningTolerance_;

  bool go_ = false;
  bool bypass_go_ = true;
  bool pre_position_achieved_ = false;
  bool heading_achieved_ = false;
  float pre_target_x_ = NULL;
  float pre_target_y_ = NULL;

  int timeout_count_ = 0;
  float target_x_ = NULL;
  float target_y_ = NULL;
  float target_y_diff_ = NULL;
  int goal_reached_count_ = 0;

  rclcpp::TimerBase::SharedPtr cmd_timer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr unfiltered_cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr go_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoUnderTrolley>());
  rclcpp::shutdown();
  return 0;
}