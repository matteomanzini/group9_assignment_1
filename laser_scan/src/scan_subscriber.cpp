#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <memory>
#include <chrono>
#include <string>
#include <vector>

class ScanSubscriber : public rclcpp::Node 
{
  public:
  ScanSubscriber() : Node("scan_subcriber")
  {
    auto topic_callback = [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){
      
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
      RCLCPP_INFO(this->get_logger(), "Detected clusters: %d", cluster_count);

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
          RCLCPP_INFO(this->get_logger(), "table x: %f, y: %f", cx, cy);
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
      publisher_->publish(pose_array);
    };

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("tables", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, topic_callback);
  }

  private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("server");
  
  rclcpp::spin(std::make_shared<ScanSubscriber>());

  rclcpp::shutdown();
}