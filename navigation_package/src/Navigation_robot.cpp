#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include <map> 

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class NavigationNode : public rclcpp::Node {
    public:
    NavigationNode() : Node("navigation_node"){

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscription per AprilTag detections
        subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>("/my_apriltag/detections", 10, std::bind(&NavigationNode::navigation_callback, this, std::placeholders::_1));

        navigation_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "Started node navigation_node!");
    }
    
    private:
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_;
    
    std::map<int, geometry_msgs::msg::PoseStamped> map_apriltags;
    geometry_msgs::msg::PoseStamped goal_point;
    geometry_msgs::msg::PoseStamped map_point;
    rclcpp::Time start_nav;
    rclcpp::Time end_nav;

    bool get_apriltag_pose(int tag_id, geometry_msgs::msg::PoseStamped& pose) {
        std::string tag_frame = "tag36h11:" + std::to_string(tag_id);
        std::string reference_frame = "external_camera/link/rgb_camera";
        
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(reference_frame, tag_frame, tf2::TimePointZero);
            
            pose.header.frame_id = reference_frame;
            pose.header.stamp = transform_stamped.header.stamp;
            pose.pose.position.x = transform_stamped.transform.translation.x;
            pose.pose.position.y = transform_stamped.transform.translation.y;
            pose.pose.position.z = transform_stamped.transform.translation.z;
            pose.pose.orientation = transform_stamped.transform.rotation;

            RCLCPP_INFO(this->get_logger(), "Conversion coordinates with respect to external camera frame!");
            
            return true;
        }
        catch(const tf2::TransformException &e) {
            RCLCPP_WARN(this->get_logger(), "Impossible conversion to TF for tag %d: %s", tag_id, e.what());
            return false;
        }
    }

    void navigation_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message){

        for(auto &det : message->detections) {
            int id = det.id;

            if(map_apriltags.find(id) != map_apriltags.end()) continue;

            geometry_msgs::msg::PoseStamped point;
            bool conv_to_pose = get_apriltag_pose(id, point);

            if (conv_to_pose == false) {
                RCLCPP_WARN(this->get_logger(), "Wrong conversion!");
                continue;
            }

            map_apriltags.insert({id, point});

            if(map_apriltags.size() == 2) transformation_map_callback();
        }
    }

    void transformation_map_callback(){

        goal_point.pose.position.x = 0;
        goal_point.pose.position.y = 0;
        goal_point.pose.position.z = 0;

        for(const auto& apriltag : map_apriltags) {

            auto point = apriltag.second;
            geometry_msgs::msg::PoseStamped transformed_point_to_map_;
            RCLCPP_INFO(this->get_logger(), " Apriltag %d orginal coordinates: x=%.2f, y=%.2f, z=%.2f", apriltag.first, point.pose.position.x, point.pose.position.y, point.pose.position.z);

            try {
                // TRASFORMATION IN MAP
                geometry_msgs::msg::TransformStamped transform_stamped_to_map = tf_buffer_->lookupTransform("map", point.header.frame_id, tf2::TimePointZero);
                tf2::doTransform(point, transformed_point_to_map_, transform_stamped_to_map);
            }
            catch(const tf2::TransformException &e) {
                RCLCPP_ERROR(this->get_logger(), " Error: %s", e.what());
                return;
            }

            goal_point.pose.position.x += transformed_point_to_map_.pose.position.x;
            goal_point.pose.position.y += transformed_point_to_map_.pose.position.y;
            goal_point.pose.position.z += transformed_point_to_map_.pose.position.z;

            RCLCPP_INFO(this->get_logger(), " Tag ID %d in MAP: x=%.2f, y=%.2f, z=%.2f", apriltag.first, transformed_point_to_map_.pose.position.x, transformed_point_to_map_.pose.position.y, transformed_point_to_map_.pose.position.z);
        }

        goal_point.header.frame_id = "map";
        goal_point.header.stamp = this->now();
        goal_point.pose.position.x = goal_point.pose.position.x/2;
        goal_point.pose.position.y = goal_point.pose.position.y/2;
        goal_point.pose.position.z = 0;
        goal_point.pose.orientation.w = 1;

        RCLCPP_INFO(this->get_logger(), " Middle point in MAP: x=%.2f, y=%.2f, z=%.2f", goal_point.pose.position.x, goal_point.pose.position.y, goal_point.pose.position.z);

        robot_navigation();
    }

    void robot_navigation(){
        
        if (!navigation_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), " Not available 'navigate_to_pose' node");
            return;
        }
        
        nav2_msgs::action::NavigateToPose::Goal goal_msg;
        goal_msg.pose = goal_point;
        start_nav = this->now();
        
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
            end_nav = this->now();
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_INFO(this->get_logger(), " The goal is reached!");
                RCLCPP_INFO(this->get_logger(), " Time of navigation: %.2f seconds", (end_nav - start_nav).seconds());
                return;
            }
            else if(result.code == rclcpp_action::ResultCode::ABORTED){
                RCLCPP_ERROR(this->get_logger(), " Navigation aborted!");
                return;
            }
            else if(result.code == rclcpp_action::ResultCode::CANCELED){
                RCLCPP_WARN(this->get_logger(), " Navigation canceled!");
                return;
            }
            else{
                RCLCPP_WARN(this->get_logger(), " Navigation failed!");
                return;
            }
        };

        navigation_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationNode>());
    rclcpp::shutdown();
    return 0;
}