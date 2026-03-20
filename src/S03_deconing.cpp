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
#include "ur_msgs/srv/set_io.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "smw_2026/S03_deconing_waypoints.hpp"

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
        auto waypoints = Waypoints();
        
        left_gripper_neutral();

        left_gripper_off();
        wp_entry_point->move_to_joint_positions(waypoints.right_rest.joint_values,wp_entry_point->right_move_group_interface_);
        wp_entry_point->move_to_joint_positions(waypoints.left_rest.joint_values,wp_entry_point->left_move_group_interface_);

        // right parallel approach to the cone from the side
        std::vector<geometry_msgs::msg::Pose> right_to_the_cone_waypoints{
            waypoints.right_wp1.pose,
            waypoints.right_wp2.pose,
            waypoints.right_wp3.pose,
            waypoints.right_wp4.pose,
            waypoints.right_wp5.pose,
        };
        std::vector<double> right_to_the_cone_durations{
            1.3,1.3,1.3,0.9,0.9
        };

        // left parallel approach to the cone
        std::vector<geometry_msgs::msg::Pose> left_below_the_cone_waypoints{
            waypoints.left_wp1.pose,
            waypoints.left_wp2.pose,
            waypoints.left_wp3.pose,
        };
        std::vector<double> left_below_the_cone_durations{
            2.0,2.0,2.0
        };

        // start the right
        auto right_to_the_cone_future = wp_entry_point->async_start_execute_waypoints_cubic(
            right_to_the_cone_waypoints,
            right_to_the_cone_durations,
            0.3,
            0.18,
            "right"
        );
        // start the left
        auto left_to_the_cone_future = wp_entry_point->async_start_execute_waypoints_cubic(
            left_below_the_cone_waypoints,
            left_below_the_cone_durations,
            0.3,
            0.1,
            "left"
        );

        left_gripper_off();
        std::this_thread::sleep_for(500ms);
        left_gripper_neutral();

        // block till left is done
        wp_entry_point->block_till_response_execute_cubic_trajectory(left_to_the_cone_future,15s);
        
        // left to hold the cone before starting to decone
        std::vector<geometry_msgs::msg::Pose> left_hold_the_cone_waypoints{
            waypoints.left_wp4.pose,
        };
        std::vector<double> left_hold_the_cone_durations{
            0.5,
        };

        // start left to hold the cone
        auto left_hold_the_cone_future = wp_entry_point->async_start_execute_waypoints_cubic(
            left_hold_the_cone_waypoints,
            left_hold_the_cone_durations,
            0.3,
            0.0,
            "left"
        );

        // block till both the left is finished
        wp_entry_point->block_till_response_execute_cubic_trajectory(left_hold_the_cone_future,15s);
        left_gripper_on();

        // block till the right is finished
        wp_entry_point->block_till_response_execute_cubic_trajectory(right_to_the_cone_future,15s);

        // twist the cone using the left gripper
        std::vector<geometry_msgs::msg::Pose> left_twist_cone_waypoints{
            waypoints.left_wp5.pose
        };
        std::vector<double> left_twist_cone_durations{
            1.0
        };
        wp_entry_point->execute_waypoints_cubic(left_twist_cone_waypoints,left_twist_cone_durations,0.3,0.0,"left");

        // gotta move both of them down at the same time 
        geometry_msgs::msg::Pose right_wp6(waypoints.right_wp5.pose), left_wp6(waypoints.left_wp5.pose);
        offset_position(right_wp6,std::vector<double>{0.0,0.0,-0.17});
        offset_position(left_wp6,std::vector<double>{0.0,0.0,-0.18});

        std::vector<geometry_msgs::msg::Pose> right_down_waypoints{right_wp6};
        std::vector<geometry_msgs::msg::Pose> left_down_waypoints{left_wp6};
        std::vector<double> down_durations{2.0};

        // start right going down
        auto right_down_future = wp_entry_point->async_start_execute_waypoints_cubic(
            right_down_waypoints,
            down_durations,
            0.3,
            0.0,
            "right"
        );
        // start left going down
        auto left_down_future = wp_entry_point->async_start_execute_waypoints_cubic(
            left_down_waypoints,
            down_durations,
            0.3,
            0.0,
            "left"
        );
        // sync them
        wp_entry_point->block_till_response_execute_cubic_trajectory(right_down_future,15s);
        wp_entry_point->block_till_response_execute_cubic_trajectory(left_down_future,15s);

        // left untwist the cone, synchronized
        geometry_msgs::msg::Pose left_wp7;
        left_wp7.position = left_wp6.position;
        left_wp7.orientation = waypoints.left_wp4.pose.orientation;
        std::vector<geometry_msgs::msg::Pose> left_untwist_waypoints{left_wp7};
        std::vector<double> left_untwist_durations{0.5};
        wp_entry_point->execute_waypoints_cubic(
            left_untwist_waypoints,
            left_untwist_durations,
            0.3,
            0.0,
            "left"
        );

        left_gripper_off();

        // left move down
        geometry_msgs::msg::Pose left_wp8(left_wp7);
        offset_position(left_wp8,std::vector<double>{0.0,0.0,-0.1});
        std::vector<geometry_msgs::msg::Pose> left_get_down_waypoints{left_wp8};
        std::vector<double> left_get_down_durations{1.0};
        wp_entry_point->execute_waypoints_cubic(
            left_get_down_waypoints,
            left_get_down_durations,
            0.3,
            0.0,
            "left"
        );

        RCLCPP_INFO(node_->get_logger(),"NOOOOOOOOOOWW, STARTING LEFT BACK AWAY");

        // in parallel, left back away and right get back routine
        offset_position(waypoints.left_wp2.pose,std::vector<double>{0.0,0.2,-0.1});
        std::vector<geometry_msgs::msg::Pose> left_back_away_waypoints{
            waypoints.left_wp2.pose,
            waypoints.left_wp1.pose,
            waypoints.left_rest.pose
        };
        std::vector<double> left_back_away_durations{
            1.5,
            1.5,
            1.0
        };
        auto left_back_away_future = wp_entry_point->async_start_execute_waypoints_cubic(
            left_back_away_waypoints,
            left_back_away_durations,
            0.3,
            0.1,
            "left"
        );
        
        // right back away routine
        std::vector<geometry_msgs::msg::Pose> right_back{
            waypoints.right_wp7.pose,
            waypoints.right_wp78.pose,
            waypoints.right_wp8.pose
            
        };
        
        std::this_thread::sleep_for(1000ms);
        auto right_back_future = wp_entry_point->async_start_execute_waypoints_cubic(right_back,std::vector<double>{1.5,1.5,1.5},0.3,0.1,"right"); 
        
        wp_entry_point->block_till_response_execute_cubic_trajectory(left_back_away_future,15s);
        wp_entry_point->block_till_response_execute_cubic_trajectory(right_back_future,15s);

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