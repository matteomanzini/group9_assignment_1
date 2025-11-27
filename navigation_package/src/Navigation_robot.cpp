#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "interfaces_assignment_1/msg/ready.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include <map>
#include <math.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class NavigationNode : public rclcpp::Node {
    public:
    NavigationNode() : Node("navigation_node"){

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscription per AprilTag detections
        subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>("/my_apriltag/detections", 10, std::bind(&NavigationNode::start_navigation_callback, this, std::placeholders::_1));
        subscription_ready_msg_ = this->create_subscription<interfaces_assignment_1::msg::Ready>("/ready", 10, std::bind(&NavigationNode::ready_callback, this, std::placeholders::_1));
        /*timer_ready = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NavigationNode::timer_ready_msg, this)
        );*/

        // path_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&NavigationNode::update_callback, this, std::placeholders::_1));

        navigation_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        fill_map = false;
        navigation_active = false;
        ready = false;
        index1 = 0;
        index10 = 0;
        final_point1.header.frame_id = "map";
        final_point1.pose.position.z = 0;
        final_point1.pose.orientation.w = 1;
        final_point10.header.frame_id = "map";
        final_point10.pose.position.z = 0;
        final_point10.pose.orientation.w = 1;

        RCLCPP_INFO(this->get_logger(), "Started node navigation_node!");
    }
    
    private:
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
    rclcpp::Subscription<interfaces_assignment_1::msg::Ready>::SharedPtr subscription_ready_msg_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr path_subscription_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    bool fill_map; // first time that compute the transforations
    bool navigation_active; // true if the robot is in navigation
    bool ready; 
    
    std::map<int, geometry_msgs::msg::PoseStamped> map_apriltags; // the coordinates are respect to the map
    std::map<int, geometry_msgs::msg::PoseStamped> mean_finalpoint; //  mean point between two apriltags
    geometry_msgs::msg::PoseStamped goal_point; // initial goal point
    geometry_msgs::msg::PoseStamped final_point; // final goal point
    geometry_msgs::msg::PoseStamped final_point1; // mean point of id's point 1
    geometry_msgs::msg::PoseStamped final_point10; // mean point of id's point 10
    int index1; // number of id points 1 
    int index10; // number of id points 10
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
    
    // time of execution
    rclcpp::Time start_nav;
    rclcpp::Time end_nav;
    // rclcpp::TimerBase::SharedPtr timer_ready;

    // COMPUTE THE DISTANCE BETWEEN TWO POINTS FROM SAME FRAME
    float getDistance(geometry_msgs::msg::PoseStamped point1, geometry_msgs::msg::PoseStamped point2){
        
        float dist = 0.0;

        if(point1.header.frame_id == point2.header.frame_id){
            float dist_x = point1.pose.position.x - point2.pose.position.x;
            float dist_y = point1.pose.position.y - point2.pose.position.y;
            float dist_z = point1.pose.position.z - point2.pose.position.z;
            dist = std::sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Different frames for two points!");
        }

        return dist;
    }

    // TRASFORMATION FUNCTION to map points
    geometry_msgs::msg::PoseStamped trasformation_to_map(const geometry_msgs::msg::PoseStamped& started_point){

        geometry_msgs::msg::PoseStamped output;

        try {
            // TRASFORMATION IN MAP
            geometry_msgs::msg::TransformStamped transform_stamped_to_map = tf_buffer_->lookupTransform("map", started_point.header.frame_id, tf2::TimePointZero);
            tf2::doTransform(started_point, output, transform_stamped_to_map);
        }
        catch(const tf2::TransformException &e) {
            RCLCPP_ERROR(this->get_logger(), "Error ! Impossible to transform to map frame: %s", e.what());
        }

        return output;
    }

    // COMPUTE THE GOAL MIDDLE POINT
    geometry_msgs::msg::PoseStamped compute_goal_point(){

        geometry_msgs::msg::PoseStamped goal;

        goal.pose.position.x = 0;
        goal.pose.position.y = 0;
        goal.pose.position.z = 0;

        for(const auto& pair : map_apriltags) {

            auto point = pair.second;

            goal.pose.position.x += point.pose.position.x;
            goal.pose.position.y += point.pose.position.y;
            goal.pose.position.z += point.pose.position.z;

            RCLCPP_INFO(this->get_logger(), "Tag ID %d in MAP: x = %.2f, y = %.2f, z = %.2f", pair.first, point.pose.position.x, point.pose.position.y, point.pose.position.z);
        }

        goal.header.frame_id = "map";
        goal.header.stamp = this->now();
        goal.pose.position.x = goal.pose.position.x/2;
        goal.pose.position.y = goal.pose.position.y/2;
        goal.pose.position.z = 0; 
        goal.pose.orientation.w = 1;

        RCLCPP_INFO(this->get_logger(), "Middle point in MAP: x = %.2f, y = %.2f, z = %.2f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

        return goal;
    }

    // CHECK IF THE POSE IS EMPTY
    bool poseEmpty(const geometry_msgs::msg::PoseStamped& current_pose){
        if(current_pose.pose.position.x == 0 && current_pose.pose.position.y == 0 && current_pose.pose.position.z == 0 && current_pose.pose.orientation.x == 0 && current_pose.pose.orientation.y == 0 && current_pose.pose.orientation.z == 0) return true;

        return false;
    }

    bool recompute_goal(){
        
        if(!navigation_active) return false;

        final_point.header.frame_id = "map";

        final_point.pose.position.x = (final_point1.pose.position.x/index1 + final_point10.pose.position.x/index10)/2;
        final_point.pose.position.y = (final_point1.pose.position.y/index1 + final_point10.pose.position.y/index10)/2;
        final_point.pose.position.z = 0;
        final_point.pose.orientation.w = 1;

        float dist_between_goals = getDistance(final_point, goal_point);

        // if the distance to goal is less than 0.05 meter, recompute the goal point
        if(dist_between_goals > 0.05){

            RCLCPP_INFO(this->get_logger(), "The goal is changed! New goal is: x = %.2f, y = %.2f", final_point.pose.position.x, final_point.pose.position.y);

            return true;
        }

        return false;
    }

    bool get_apriltag_pose(int tag_id, geometry_msgs::msg::PoseStamped& pose) {

        std::string tag_frame = "tag36h11:" + std::to_string(tag_id);
        std::string reference_frame = "external_camera/link/rgb_camera"; //"map";
        
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(reference_frame, tag_frame, tf2::TimePointZero);
            
            pose.header.frame_id = reference_frame;
            pose.header.stamp = transform_stamped.header.stamp;
            pose.pose.position.x = transform_stamped.transform.translation.x;
            pose.pose.position.y = transform_stamped.transform.translation.y;
            pose.pose.position.z = transform_stamped.transform.translation.z;
            pose.pose.orientation = transform_stamped.transform.rotation;
            
            return true;
        }
        catch(const tf2::TransformException &e) {
            RCLCPP_WARN(this->get_logger(), "Impossible obtain TF for tag %d: %s", tag_id, e.what());
            return false;
        }
    }

    void ready_callback(const interfaces_assignment_1::msg::Ready::SharedPtr message){
        
        ready = message->ready.data;
        RCLCPP_INFO(this->get_logger(), "The current value of ready is: %d!", ready);

    }
    
    /*void timer_ready_msg(){

        if(ready){
            RCLCPP_INFO(this->get_logger(), "The robot is ready for the navigation!");
        }
        else{
            RCLCPP_WARN(this->get_logger(), "The robot is not ready for the navigation!");
        }
    }*/

    void start_navigation_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message){
        
        if(!ready){
            RCLCPP_WARN(this->get_logger(), "Navigation stopped! Try later.");
            return;
        }

        for(auto &det : message->detections) {
            int id = det.id;

            geometry_msgs::msg::PoseStamped point;
            bool conv_to_pose = get_apriltag_pose(id, point);
            
            if (conv_to_pose == false) {
                RCLCPP_WARN(this->get_logger(), "Wrong conversion!");
                continue;
            }

            // insert or upload of map point
            map_apriltags[id] = trasformation_to_map(point);

            if(id == 1){
                final_point1.pose.position.x += map_apriltags[id].pose.position.x;
                final_point1.pose.position.y += map_apriltags[id].pose.position.y;
                index1++;
            }
            else{
                final_point10.pose.position.x += map_apriltags[id].pose.position.x;
                final_point10.pose.position.y += map_apriltags[id].pose.position.y;
                index10++;
            }

            // if it is the first time that the map has two elements
            if(map_apriltags.size() == 2 && !fill_map){
                fill_map = true;
                RCLCPP_INFO(this->get_logger(), "Set the goal for the first time!");
                goal_point = compute_goal_point();
                robot_navigation(goal_point);
            }
        }
    }

    void robot_navigation(geometry_msgs::msg::PoseStamped goal){
        
        /*if (!navigation_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Not available 'navigate_to_pose' server");
            return;
        }*/

        navigation_active = true;
        
        nav2_msgs::action::NavigateToPose::Goal goal_msg;
        goal_msg.pose = goal;
        start_nav = this->now();
        
        send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
            
            end_nav = this->now();
            navigation_active = false;

            if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_INFO(this->get_logger(), "The goal is reached! The goal point is: x = %.2f, y = %.2f!", goal_point.pose.position.x, goal_point.pose.position.y);
                RCLCPP_INFO(this->get_logger(), "Time of navigation: %.2f seconds", (end_nav - start_nav).seconds());

                if(recompute_goal()){
                    RCLCPP_WARN(this->get_logger(), "The goal is changed!");
                    goal_point = final_point;
                    robot_navigation(goal_point);
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "Final goal is confermed!");
                }

                return;
            }
            else if(result.code == rclcpp_action::ResultCode::ABORTED){
                RCLCPP_ERROR(this->get_logger(), "Navigation aborted for the point x = %.2f, y = %.2f after time %.2f seconds!", goal_point.pose.position.x, goal_point.pose.position.y, (end_nav - start_nav).seconds());
                return;
            }
            else if(result.code == rclcpp_action::ResultCode::CANCELED){
                RCLCPP_WARN(this->get_logger(), "Navigation canceled for the point x = %.2f, y = %.2f after time %.2f seconds!", goal_point.pose.position.x, goal_point.pose.position.y, (end_nav - start_nav).seconds());
                return;
            }
            else{
                RCLCPP_WARN(this->get_logger(), "Navigation stopped for the point x = %.2f, y = %.2f after time %.2f seconds!", goal_point.pose.position.x, goal_point.pose.position.y, (end_nav - start_nav).seconds());
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