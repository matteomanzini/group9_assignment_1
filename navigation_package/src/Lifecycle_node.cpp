//node that manages the lifecycle of navigation stack

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "interfaces_assignment_1/srv/ready.hpp"

#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

class Lifecycle_node : public rclcpp::Node
{
    public:
    Lifecycle_node()
    : Node("lifecycle_client")
    {
        localization_ptr = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");
        navigation_ptr = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
        initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom",10, std::bind(&Lifecycle_node::odomCallback, this, std::placeholders::_1));
        ready_service = this->create_service<interfaces_assignment_1::srv::Ready>("ready", std::bind(&Lifecycle_node::readyCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Lifecycle node initialized");
        
        initialpose_sent = false;
        ready = false;
        
    }
    
    private:

    /**
     * It stores the initial pose converted in the frame map took from the topic /odom (notice only map origin equals odom origin)
     * 
     * @param msg messages representing an etstimate of a position and velocity in free space
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(!initialpose_sent)
        {
            RCLCPP_INFO(this->get_logger(), "Received initial pose");
        
            geometry_msgs::msg::PoseWithCovarianceStamped initialPose;

            initialPose.header.frame_id = "map";
            initialPose.header.stamp = msg->header.stamp;
            initialPose.pose = msg->pose;

            RCLCPP_INFO(this->get_logger(), "Initial pose is set");

            initialpose_sent = true;
            
            this->startLocalization(initialPose);
        }
        return;
    }

    /**
     * It sends a request to ManageLifecycleNodes of nav2 to activating the localization and it publishes the initial pose needed for navigation once the localization succeed
     * 
     * @param initialPose is the initial pose of robot, use for navigation service
     */
    void startLocalization(const geometry_msgs::msg::PoseWithCovarianceStamped &initialPose)
    {
        RCLCPP_INFO(this->get_logger(), "Start localization");

        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        request->command = 0; //nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;


        while (!(localization_ptr)->wait_for_service()) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the localization service. Exiting.");
                ready = false;
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Localization service not available, waiting again...");
        }
        

        localization_ptr->async_send_request(request, [this, initialPose](rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future)
        {
            try
            {
                auto response = future.get();
                if(response->success)
                {
                    RCLCPP_INFO(this->get_logger(), "localization activated successfully");
                    initial_pose_pub->publish(initialPose);
                    this->startNavigation();
                }else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to call service Localization");
                    ready = false;
                }
            }catch (const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception in localization: %s", e.what());
                ready = false;
            }
        });

        return;

    }

    /**
     * It sends a request to ManageLifecycleNodes of nav2 to activating the navigation service and set the robot to be ready
     * 
     */
    void startNavigation()
    {
        RCLCPP_INFO(this->get_logger(), "Start navigation");

        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        request->command = 0; //nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;


        while (!(navigation_ptr)->wait_for_service()) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the navigation service. Exiting.");
                ready = false;
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Navigation service not available, waiting again...");
        }
        
        
        navigation_ptr->async_send_request(request, [this](rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future)
        {
            try
            {
                auto response = future.get();
                if (response->success) 
                {
                    RCLCPP_INFO(this->get_logger(), "navigation activated successfully");
                    ready = true;
                }else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to call service navigation");
                    ready = false;
                }
            }catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception in navigation: %s", e.what());
                ready = false;
            }
        });
        odom_sub.reset();
        return;
    }

    /**
     * It notices the robot that navigatiobn stack is set and robot can start to move
     * 
     * @param request the request have to be an empty string
     * @param response true or false, it state if the robot can move or not
     */
    void readyCallback(const std::shared_ptr<interfaces_assignment_1::srv::Ready::Request> request, 
                            std::shared_ptr<interfaces_assignment_1::srv::Ready::Response> response)
    {
        if(request->req == "") 
            response->ready.data = ready;
        return;
    }
    
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_ptr;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_ptr;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Service<interfaces_assignment_1::srv::Ready>::SharedPtr ready_service;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool initialpose_sent; //state if the initial pose is published
    bool ready; //state if the robot is ready to move
      
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Lifecycle_node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}