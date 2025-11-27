#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <chrono>
#include <string>
#include <vector>

class ScanSubscriber : public rclcpp::Node 
{
  public:
  ScanSubscriber() : Node("scan_subcriber")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("tables", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ScanSubscriber::topic_callback, this, std::placeholders::_1));
  }

  private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {  
    const auto &ranges = msg->ranges;
    std::vector<std::vector<int>> tables;
    std::vector<int> curr_table;

    for (int i = 0; i < static_cast<int>(ranges.size()); ++i)
    {
      float r = ranges[i];
      
      if (curr_table.empty()) 
      {
          curr_table.push_back(i);
      }
      else 
      {
          int prev_i = curr_table.back();
          float prev_r = ranges[prev_i];

          if (std::fabs(r - prev_r) < 0.8f) 
          {
              curr_table.push_back(i);
          } 
          else
          {
              if (curr_table.size() > 2 && curr_table.size() < 20) tables.push_back(curr_table);
              curr_table.clear();
              curr_table.push_back(i);
          }
      }
    }

    if (curr_table.size() > 2 && curr_table.size() < 20) tables.push_back(curr_table);

    int cluster_count = tables.size();
    RCLCPP_INFO(get_logger(), "Detected clusters: %d", cluster_count);

    std::vector<std::pair<float, float>> objects;

    for (const auto &table : tables)
    {
        float sum_x = 0.0f;
        float sum_y = 0.0f;

        for (int idx : table)
        {
            float r = ranges[idx];
            float angle = msg->angle_min + idx * msg->angle_increment;

            float x = r * std::cos(angle);
            float y = r * std::sin(angle);

            sum_x += x;
            sum_y += y;
        }

        float cx = sum_x / table.size();
        float cy = sum_y / table.size();
        objects.emplace_back(cx, cy);
        RCLCPP_INFO(get_logger(), "table x: %f, y: %f", cx, cy);
    }

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = msg->header;

    for (auto &[x, y] : objects)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;

        pose.orientation.w = 1.0;
        pose_array.poses.push_back(pose);
    }

    geometry_msgs::msg::PoseArray odom_array;
    odom_array.header = msg->header;
    if(msg->header.frame_id == "base_scan")
    {
      for (const auto &pose : pose_array.poses)
      {
        geometry_msgs::msg::PoseStamped pose_out, pose_in;
        pose_in.header = msg->header;
        pose_in.pose = pose;
        tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(pose_in, pose_out, "odom", tf2::Duration(std::chrono::seconds(1)));
        RCLCPP_INFO(get_logger(), "odom_table x: %f, y: %f", pose_out.pose.position.x, pose_out.pose.position.y);
        odom_array.poses.push_back(pose_out.pose);
      }
      publisher_->publish(odom_array);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<ScanSubscriber>();
  
  rclcpp::spin(node);

  rclcpp::shutdown();
}