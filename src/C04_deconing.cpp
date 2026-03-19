// task space cubic polnomial traj server

#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <cmath>
#include <sstream>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "motion_planning_abstractions/dual_arm_waypoint_programming.hpp"
#include "smw_2026/C04_deconing_waypoints.hpp"
#include "ur_msgs/srv/set_io.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;
    
void offset_position(geometry_msgs::msg::Pose& pose,std::vector<double> offset){
    pose.position.x += offset[0];
    pose.position.y += offset[1];
    pose.position.z += offset[2];
}

void offset_rotation(geometry_msgs::msg::Pose& pose, Eigen::AngleAxisd offset){
    auto current_q = Eigen::Quaterniond(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    current_q.normalize();
    auto rotation_q = Eigen::Quaterniond(offset);
    rotation_q.normalize();
    current_q = rotation_q*current_q;
    current_q.normalize();
    pose.orientation.w = current_q.w();
    pose.orientation.x = current_q.x();
    pose.orientation.y = current_q.y();
    pose.orientation.z = current_q.z();
}

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr node_=std::make_shared<rclcpp::Node>("co4_deconing");

    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor=std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node_);

    auto wp_entry_point = std::make_shared<BareBonesMoveit>(node_);
    
    auto left_set_io_client_ = node_->create_client<ur_msgs::srv::SetIO>("left_io_and_status_controller/set_io");
    auto right_set_io_client_ = node_->create_client<ur_msgs::srv::SetIO>("right_io_and_status_controller/set_io");
    
    auto left_gripper_on = [right_set_io_client_](){
        auto msg = std::make_shared<ur_msgs::srv::SetIO::Request>();
        msg->fun = msg->FUN_SET_DIGITAL_OUT;
        
        msg->pin = msg->PIN_DOUT6;
        msg->state = msg->STATE_OFF;
        right_set_io_client_->async_send_request(msg);

        msg->pin = msg->PIN_DOUT4;
        msg->state = msg->STATE_ON;
        right_set_io_client_->async_send_request(msg);
    };

    auto left_gripper_off = [right_set_io_client_](){
        auto msg = std::make_shared<ur_msgs::srv::SetIO::Request>();
        msg->fun = msg->FUN_SET_DIGITAL_OUT;
        
        msg->pin = msg->PIN_DOUT6;
        msg->state = msg->STATE_ON;
        right_set_io_client_->async_send_request(msg);

        msg->pin = msg->PIN_DOUT4;
        msg->state = msg->STATE_OFF;
        right_set_io_client_->async_send_request(msg);
    };

    auto left_gripper_neutral = [right_set_io_client_](){
        auto msg = std::make_shared<ur_msgs::srv::SetIO::Request>();
        msg->fun = msg->FUN_SET_DIGITAL_OUT;
        
        msg->pin = msg->PIN_DOUT6;
        msg->state = msg->STATE_OFF;
        right_set_io_client_->async_send_request(msg);

        msg->pin = msg->PIN_DOUT4;
        msg->state = msg->STATE_OFF;
        right_set_io_client_->async_send_request(msg);
    };

    // ACUAL CODE
    auto execute_service_callback = [wp_entry_point,node_,left_gripper_on,left_gripper_off,left_gripper_neutral](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
        auto LOGGER = node_->get_logger();
        auto waypoints = Waypoints(); // this is where all the waypoints are
        
        //waypoint to the cone
        wp_entry_point->move_to_joint_positions(waypoints.right_rest_state.joint_values,wp_entry_point->right_move_group_interface_);
        wp_entry_point->move_to_joint_positions(waypoints.left_rest.joint_values,wp_entry_point->left_move_group_interface_);
        wp_entry_point->move_to_joint_positions(waypoints.right_wp1.joint_values,wp_entry_point->right_move_group_interface_);
        wp_entry_point->move_to_joint_positions(waypoints.left_wp1.joint_values,wp_entry_point->left_move_group_interface_);
        
        // right to the cc holding the cone
        std::vector<geometry_msgs::msg::Pose> to_the_cone{
            waypoints.right_wp2.pose,
            waypoints.right_wp3.pose,
        };
        offset_position(waypoints.left_wp4.pose,std::vector<double>{0.009,0.0,0.0});
        
        left_gripper_off();
        
        // left to unlock
        std::vector<geometry_msgs::msg::Pose> left_to_the_cone{
            waypoints.left_wp3.pose,
            waypoints.left_wp4.pose,
        };

        auto right_to_the_cone_future  = wp_entry_point->async_start_execute_waypoints_cubic(to_the_cone,std::vector<double>{0.7,0.4},0.3,0.02,"right");
        auto left_to_the_cone_future = wp_entry_point->async_start_execute_waypoints_cubic(left_to_the_cone,std::vector<double>{2.9,0.7},0.3,0.04,"left");
        
        wp_entry_point->block_till_response_execute_cubic_trajectory(right_to_the_cone_future,15s);
        wp_entry_point->block_till_response_execute_cubic_trajectory(left_to_the_cone_future,15s);
        
        // left unlocked, now right apply pressure a little
        offset_rotation(waypoints.right_wp4.pose,Eigen::AngleAxisd(-0.1,Eigen::Vector3d(0,0,1)));
        std::vector<geometry_msgs::msg::Pose> right_pressure{
            waypoints.right_wp4.pose
        };
        wp_entry_point->execute_waypoints_cubic(right_pressure,std::vector<double>{1.2},0.3,0.02,"right"); 

        // now left back away
        std::vector<geometry_msgs::msg::Pose> left_back_away{
            waypoints.left_wp5.pose,
        };
        wp_entry_point->execute_waypoints_cubic(left_back_away,std::vector<double>{1.5},0.3,0.02,"left");

        // now right full unlock and downwards
        std::vector<geometry_msgs::msg::Pose> right_fullunlock{
            waypoints.right_wp5.pose,
            waypoints.right_wp6.pose
        };
        auto right_fullunlock_future = wp_entry_point->async_start_execute_waypoints_cubic(right_fullunlock,std::vector<double>{1.0,0.8},0.3,0.0,"right"); 

        // clear left away completely
        wp_entry_point->move_to_joint_positions(waypoints.left_home.joint_values, wp_entry_point->left_move_group_interface_);
        
        // get the right arm back
        std::vector<geometry_msgs::msg::Pose> right_back{
            waypoints.right_wp7.pose,
            waypoints.right_wp78.pose,
            waypoints.right_wp8.pose

        };
        wp_entry_point->block_till_response_execute_cubic_trajectory(right_fullunlock_future,15s);
        wp_entry_point->execute_waypoints_cubic(right_back,std::vector<double>{0.75,0.75,1.5},0.3,0.1,"right"); 
        left_gripper_neutral();
    };

    auto callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto execute_server = node_->create_service<std_srvs::srv::Trigger>(
        "~/execute",
        execute_service_callback,
        rmw_qos_profile_services_default,
        callback_group
    );

    executor->spin();
}