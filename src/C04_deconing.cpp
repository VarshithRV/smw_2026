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

using namespace std::chrono_literals;
    
int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr node_=std::make_shared<rclcpp::Node>("co4_deconing");

    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor=std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node_);

    auto wp_entry_point = std::make_shared<BareBonesMoveit>(node_);
    
    auto execute_service_callback = [wp_entry_point,node_](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
        auto LOGGER = node_->get_logger();
        auto waypoints = Waypoints(); // this is where all the waypoints are
        
        // right to the cc holding the cone
        wp_entry_point->move_to_joint_positions(waypoints.right_rest_state.joint_values,wp_entry_point->right_move_group_interface_);
        wp_entry_point->move_to_joint_positions(waypoints.right_wp1.joint_values,wp_entry_point->right_move_group_interface_);
        wp_entry_point->move_to_joint_positions(waypoints.right_wp2.joint_values,wp_entry_point->right_move_group_interface_);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        std::vector<geometry_msgs::msg::Pose> right_to_the_cone{
            waypoints.right_wp3.pose
        };
        wp_entry_point->execute_waypoints_cubic(right_to_the_cone,std::vector<double>{2.0},0.3,0.02,"right");

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // left to unlock
        wp_entry_point->move_to_joint_positions(waypoints.left_rest.joint_values,wp_entry_point->left_move_group_interface_);
        wp_entry_point->move_to_joint_positions(waypoints.left_wp1.joint_values,wp_entry_point->left_move_group_interface_);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        std::vector<geometry_msgs::msg::Pose> left_to_the_cone{
            waypoints.left_wp2.pose,
            waypoints.left_wp3.pose,
            waypoints.left_wp4.pose,
        };
        wp_entry_point->execute_waypoints_cubic(left_to_the_cone,std::vector<double>{2.0,2.0,2.0},0.3,0.0,"left");

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // left unlocked, now right apply pressure a little
        std::vector<geometry_msgs::msg::Pose> right_pressure{
            waypoints.right_wp4.pose
        };
        wp_entry_point->execute_waypoints_cubic(right_pressure,std::vector<double>{2.0},0.3,0.02,"right"); 

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // now left back away
        std::vector<geometry_msgs::msg::Pose> left_back_away{
            waypoints.left_wp5.pose,
        };
        wp_entry_point->execute_waypoints_cubic(left_back_away,std::vector<double>{2.0},0.3,0.02,"left");

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // now right full unlock and downwards
        std::vector<geometry_msgs::msg::Pose> right_fullunlock{
            waypoints.right_wp5.pose,
            waypoints.right_wp6.pose

        };
        wp_entry_point->execute_waypoints_cubic(right_fullunlock,std::vector<double>{2.0,2.0},0.3,0.0,"right"); 
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // clear left away completely
        wp_entry_point->move_to_joint_positions(waypoints.left_home.joint_values, wp_entry_point->left_move_group_interface_);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // get the right arm back
        std::vector<geometry_msgs::msg::Pose> right_back{
            waypoints.right_wp7.pose,
            waypoints.right_wp78.pose,
            waypoints.right_wp8.pose

        };
        wp_entry_point->execute_waypoints_cubic(right_back,std::vector<double>{1.75,1.75,1.75},0.3,0.02,"right"); 
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
