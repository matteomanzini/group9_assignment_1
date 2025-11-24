//node that manages the lifecycle of navigation stack

//client node







#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

#include <chrono>
//#include <random>
#include <memory>

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
        
    }

    void bringup()
    {   
        this->start_localization();
        
    }
    
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

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), localization_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "localization result: %s", localization_result.get()->success == 1 ? "true" : "false");
            start_navigation();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service Localization");
        }
    }

    void start_navigation()
    {
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
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service navigation");
        }
    }
    
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_ptr;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_ptr;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Lifecycle_node>();
    //
    node->bringup();

    //rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}