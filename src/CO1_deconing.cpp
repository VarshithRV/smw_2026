#include <functional>
#include <cmath>
#include <memory>
#include <cstdlib>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "motion_planning_abstractions/dual_arm_waypoint_programming.hpp"
#include "smw_2026/C01_deconing_waypoints.hpp"

#include "ur_msgs/srv/set_io.hpp"

using namespace std::chrono_literals;

bool is_joint_state_close(std::vector<double> a, std::vector<double> b){
    auto A = Eigen::Vector<double,6>(a[0],a[1],a[2],a[3],a[4],a[5]);
    auto B = Eigen::Vector<double,6>(b[0],b[1],b[2],b[3],b[4],b[5]);
    return A.isApprox(B,0.02);
}

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
    rclcpp::Node::SharedPtr node_=std::make_shared<rclcpp::Node>("so1_deconing");

    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor=std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node_);

    auto wp_entry_point = std::make_shared<DualArmControlInterface>(node_);
    
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

    auto execute_service_callback = [wp_entry_point,node_,left_gripper_on,left_gripper_off,left_gripper_neutral](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
        auto LOGGER = node_->get_logger();
        auto waypoints = Waypoints(); // this is where all the waypoints are
        
        auto left_start_joint_state = wp_entry_point->get_current_joint_state("left");
        auto right_start_joint_state = wp_entry_point->get_current_joint_state("right");
        std::vector<double> left_preaction_state{0.6327896118164062, -1.530074381535389, -2.6847221851348877, -0.4980843824199219, -1.5689991156207483, -2.2041104475604456};
        std::vector<double> right_preaction_state{0.6327896118164062, -1.530074381535389, -2.6847221851348877, -0.4980843824199219, -1.5689991156207483, -2.2041104475604456};
        std::vector<double> right_cone_transfer_state{0.9119113789548221, -1.329804040400156, 2.472230381749085, -2.7317907796696113, -4.6925038067020255, 2.23798315279257};
        std::vector<double> left_wait_state{-0.17850903321692962,-2.7613514763553777,-1.7269785153531936,-0.19646669880670722,-1.584020013016373,-3.037883709713575};
        // std::vector<double> right_wait_state{-0.4070860886603199,-0.942273518609286,2.00651399416375,-1.850991596596188,-5.691643654009712,0.7551481872603174};
        std::vector<double> right_wait_state{-0.2619536558734339, -0.19895155847583013, 1.4168875853167933, -2.7887374363341273, -4.712385479603903, 1.8327560424804688};
        
        std::vector<geometry_msgs::msg::Pose> right_waypoints;
        std::vector<double> right_durations;
        std::vector<geometry_msgs::msg::Pose> left_waypoints;
        std::vector<double> left_durations;

        if(is_joint_state_close(left_start_joint_state,left_preaction_state)){
            // left is at preaction, do nothing
            RCLCPP_INFO(node_->get_logger(), "Left at preaction, move on with deconing");
        }
        else if(is_joint_state_close(left_start_joint_state,left_wait_state)){
            // left is at wait, start moving to home
            RCLCPP_INFO(node_->get_logger(), "Left at wait state, start moving parallely to home");
            left_waypoints.insert(left_waypoints.end(),{waypoints.left_wp0.pose,waypoints.left_rest.pose});
            left_durations.insert(left_durations.end(),{3.0,3.0});
        }
        else{
            // left is elsewhere, let left go to preaction
            wp_entry_point->move_to_joint_positions(left_preaction_state,wp_entry_point->left_move_group_interface_);
        }

        if(is_joint_state_close(right_start_joint_state,right_preaction_state) or is_joint_state_close(right_start_joint_state,right_cone_transfer_state)){
            // right is at preaction or cone transfer, go to preaction
            wp_entry_point->move_to_joint_positions(waypoints.right_rest_state.joint_values,wp_entry_point->right_move_group_interface_);
            right_waypoints.insert(right_waypoints.end(),{waypoints.right_wp0.pose,waypoints.right_wp1.pose,waypoints.right_wp2.pose,waypoints.right_wp3.pose});
            right_durations.insert(right_durations.end(),{2.0,2.0,2.0,1.3});

        }
        else if(is_joint_state_close(right_start_joint_state,right_wait_state)){
            // right is at wait state, start from here to start coning, wp2 and wp3
            right_waypoints.insert(right_waypoints.end(),{waypoints.right_wp2.pose,waypoints.right_wp3.pose});
            right_durations.insert(right_durations.end(),{2.0,1.3});
        }
        else{
            // right is else where
            wp_entry_point->move_to_joint_positions(waypoints.right_rest_state.joint_values,wp_entry_point->right_move_group_interface_);
            right_waypoints.insert(right_waypoints.end(),{waypoints.right_wp0.pose,waypoints.right_wp1.pose,waypoints.right_wp2.pose,waypoints.right_wp3.pose});
            right_durations.insert(right_durations.end(),{2.0,2.0,2.0,1.3});
        }
        
        // execute the right waypoints
        RCLCPP_INFO(node_->get_logger(),"Starting parallel movement now, left first");
        auto left_move_future = wp_entry_point->async_start_execute_waypoints_cubic(left_waypoints,left_durations,0.3,0.12,"left");
        
        RCLCPP_INFO(node_->get_logger(),"Starting parallel movement now, right next first");

        wp_entry_point->execute_waypoints_cubic(right_waypoints,right_durations,0.3,0.12,"right");
        
        // twist
        std::vector<geometry_msgs::msg::Pose> twist{
            waypoints.right_wp4.pose,
        };
        wp_entry_point->execute_waypoints_cubic(twist,std::vector<double>{0.5},0.3,0.0,"right");

        // get down
        std::vector<geometry_msgs::msg::Pose> down{
            waypoints.right_wp5.pose,
        };
        wp_entry_point->execute_waypoints_cubic(down,std::vector<double>{1.0},0.3,0.05,"right");

        // backoff
        std::vector<geometry_msgs::msg::Pose> back_off{
            waypoints.right_wp6.pose,
            waypoints.right_wp7.pose,
            waypoints.right_wp8.pose,
        };
        wp_entry_point->execute_waypoints_cubic(back_off,std::vector<double>{1.3,1.3,1.0},0.3,0.05,"right");

        // wait for left to finish, prolly has long ago tho
        wp_entry_point->block_till_response_execute_cubic_trajectory(left_move_future,15s);
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