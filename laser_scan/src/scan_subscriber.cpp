#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "interfaces_assignment_1/srv/goalresult.hpp"

#include <memory>
#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class ScanSubscriber : public rclcpp::Node 
{
  public:
  ScanSubscriber() : Node("scan_subcriber")
  {
    finalpose_ptr = this->create_client<interfaces_assignment_1::srv::Goalresult>("goalresult");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("tables", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ScanSubscriber::detectTables, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(500ms, std::bind(&ScanSubscriber::startScan, this));
  }

  private:
  void detectTables(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {  
    if (!scan_flag) return;
    RCLCPP_INFO(get_logger(), "Starting room scansion");
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

          if (std::fabs(r - prev_r) < 0.8f && r <= 3.0) 
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
    RCLCPP_INFO(get_logger(), "Detected tables: %d", cluster_count);

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
        RCLCPP_INFO(get_logger(), "table position x: %f, y: %f", cx, cy);
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
    odom_array.header.frame_id = "odom";
    if(msg->header.frame_id == "base_scan")
    {
      for (const auto &pose : pose_array.poses)
      {
        geometry_msgs::msg::PoseStamped pose_out, pose_in;
        pose_in.header = msg->header;
        pose_in.pose = pose;
        tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(pose_in, pose_out, "odom", tf2::Duration(std::chrono::seconds(1)));
        RCLCPP_INFO(get_logger(), "table odom position x: %f, y: %f", pose_out.pose.position.x, pose_out.pose.position.y);
        odom_array.poses.push_back(pose_out.pose);
      }
      publisher_->publish(odom_array);
    }
  }

  void startScan()
  {
    if (scan_flag) 
    {
      timer_->cancel();
      return;
    }

    RCLCPP_INFO(get_logger(), "Start laser scan");

    auto request = std::make_shared<interfaces_assignment_1::srv::Goalresult::Request>();
    request->req.data = true;

    while(!(finalpose_ptr)->wait_for_service(100ms))
    {
      if (!rclcpp::ok()) 
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the goal pose service. Exiting.");
        scan_flag = false;
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Navigation service not available, waiting again...");
    }

    finalpose_ptr->async_send_request(request, [this](rclcpp::Client<interfaces_assignment_1::srv::Goalresult>::SharedFuture future)
    {
      try
      {
        auto response = future.get();
        if (response->goal_status.data) 
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Laser scan activated successfully");
          scan_flag = true;
        }else
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service goal pose");
          scan_flag = false;
        }
      }catch(const std::exception& e)
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in laser scan: %s", e.what());
        scan_flag = false;
      }
    });

  };

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::Client<interfaces_assignment_1::srv::Goalresult>::SharedPtr finalpose_ptr;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool scan_flag = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<ScanSubscriber>();
  
  rclcpp::spin(node);

  rclcpp::shutdown();
}
