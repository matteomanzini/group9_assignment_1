//node that manages the lifecycle of navigation stack

//client node

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

//#include "interfaces_assignment_1/msg/ready.hpp"
#include "interfaces_assignment_1/srv/ready.hpp"

#include <chrono>
//#include <random>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

class Lifecycle_node : public rclcpp::Node
                    //public std::enable_shared_from_this<Lifecycle_node>
{
    public:
    Lifecycle_node()
    : Node("lifecycle_client")
    {
        localization_ptr = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");
        navigation_ptr = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
        initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom",10, std::bind(&Lifecycle_node::odom_callback, this, std::placeholders::_1));
        ready_service = this->create_service<interfaces_assignment_1::srv::Ready>("ready", std::bind(&Lifecycle_node::ready_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lifecycle node initialized");
        
        initialpose_sent = false;
        ready = false;
        
    }
    
    private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(!initialpose_sent)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received initial pose");
        
            geometry_msgs::msg::PoseWithCovarianceStamped initialPose;

            initialPose.header.frame_id = "map";
            initialPose.header.stamp = msg->header.stamp;
            initialPose.pose = msg->pose;

            //initial_pose_pub->publish(initialPose);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initial pose is set");

            initialpose_sent = true;
            
            this->start_localization(initialPose);
        }
        return;
    }

    void start_localization(const geometry_msgs::msg::PoseWithCovarianceStamped &initialPose)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start localization");

        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        request->command = 0; //nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;


        while (!(localization_ptr)->wait_for_service()) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the localization service. Exiting.");
                //this->publish_ready_msg(false);
                ready = false;
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Localization service not available, waiting again...");
        }
        

        localization_ptr->async_send_request(request, [this, initialPose](rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future)
        {
            try
            {
                auto response = future.get();
                if(response->success)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "localization activated successfully");
                    initial_pose_pub->publish(initialPose);
                    this->start_navigation();
                }else 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service Localization");
                    //this->publish_ready_msg(false);
                    ready = false;
                }
            }catch (const std::exception& e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in localization: %s", e.what());
                //this->publish_ready_msg(false);
                ready = false;
            }
        });

        return;

    }

    

    void start_navigation()
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start navigation");

        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        request->command = 0; //nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;


        while (!(navigation_ptr)->wait_for_service()) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the navigation service. Exiting.");
                //this->publish_ready_msg(false);
                ready = false;
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Navigation service not available, waiting again...");
        }
        
        
        navigation_ptr->async_send_request(request, [this](rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future)
        {
            try
            {
                auto response = future.get();
                if (response->success) 
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "navigation activated successfully");
                    //this->publish_ready_msg(true);
                    ready = true;
                }else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service navigation");
                    //this->publish_ready_msg(false);
                    ready = false;
                }
            }catch(const std::exception& e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in navigation: %s", e.what());
                //this->publish_ready_msg(false);
                ready = false;
            }
        });
        odom_sub.reset();
        return;
    }

    /*
    void publish_ready_msg(bool ready)
    {
        auto message = interfaces_assignment_1::msg::Ready();
        message.info.stamp = this->now();
        message.info.frame_id =  "navigation";
        message.ready.data = ready;
        ready_pub->publish(message);

        if (ready) {
            RCLCPP_INFO(this->get_logger(), "Robot is ready to move!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Nav2 bringup sequence failed!");
        }
    }
    */

    void ready_callback(const std::shared_ptr<interfaces_assignment_1::srv::Ready::Request> request, 
                            std::shared_ptr<interfaces_assignment_1::srv::Ready::Response> response)
    {
        if(request->req == "") 
            response->ready.data = ready;
        return;
    }
    
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_ptr;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_ptr;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
    //rclcpp::Publisher<interfaces_assignment_1::msg::Ready>::SharedPtr ready_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool initialpose_sent;
    bool ready;
    //rclcpp::TimerBase::SharedPtr retry_timer_;
    
    rclcpp::Service<interfaces_assignment_1::srv::Ready>::SharedPtr ready_service;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Lifecycle_node>();
    //
    //node->bringup();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}