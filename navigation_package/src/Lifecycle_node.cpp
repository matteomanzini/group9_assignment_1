//node that manages the lifecycle of navigation stack

//client node

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "interfaces_assignment_1/msg/ready.hpp"

#include <chrono>
//#include <random>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

class Lifecycle_node : public rclcpp::Node,
                    public std::enable_shared_from_this<Lifecycle_node>
{
    public:
    Lifecycle_node()
    : Node("lifecycle_client")
    {
        localization_ptr = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");
        navigation_ptr = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
        initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        ready_pub = this->create_publisher<interfaces_assignment_1::msg::Ready>("/ready",10);

        start_localization();
    }

    //void bringup()
    //{   
    //    start_localization();
    //}
    
    private:

    void start_localization()
    {
        while (!(localization_ptr)->wait_for_service()) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the localization service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Localization service not available, waiting again...");
        }

        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        request->command = 0;//nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
        auto localization_result = localization_ptr->async_send_request(request);

        auto message = interfaces_assignment_1::msg::Ready();
        message.info.stamp = this->now();
        message.info.frame_id =  "navigation";

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), localization_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "localization result: %s", localization_result.get()->success == 1 ? "true" : "false");
            start_navigation(message);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service Localization");
            message.ready.data = false;
        }

        ready_pub->publish(message);
    }

    void pub_initial_pose()
    {
        geometry_msgs::msg::PoseWithCovarianceStamped initialPose;
        initialPose.header.frame_id = "map";
        initialPose.pose.pose.position.x = 0.0;
        initialPose.pose.pose.position.y = 0.0;
        initialPose.pose.pose.position.z = 0.0;
        initialPose.pose.pose.orientation.z = 0.0;
        initialPose.pose.pose.orientation.w = 1.0;
        std::array<double, 36> cov {
                        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.068, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.068, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.068, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.068
        };
        for(int i = 0; i < static_cast<int>(cov.size()); i++)
            initialPose.pose.covariance[i] = cov[i];

        initial_pose_pub->publish(initialPose);
    }

    void start_navigation(interfaces_assignment_1::msg::Ready &msg)
    {

        pub_initial_pose();

        while (!(navigation_ptr)->wait_for_service()) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the navigation service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Navigation service not available, waiting again...");
        }
        
        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        request->command = 0;//nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
        auto navigation_result = navigation_ptr->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), navigation_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "navigation result: %s", navigation_result.get()->success == 1 ? "true" : "false");
            msg.ready.data = true;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service navigation");
            msg.ready.data = false;
        }

        ready_pub->publish(msg);
    }
    
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_ptr;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_ptr;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
    rclcpp::Publisher<interfaces_assignment_1::msg::Ready>::SharedPtr ready_pub;
    //rclcpp::TimerBase::SharedPtr timer_;
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