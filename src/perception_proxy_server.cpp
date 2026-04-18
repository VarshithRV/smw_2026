// this will host a server in ros2 that will call the perception
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "motion_planning_abstractions/dual_arm_waypoint_programming.hpp"

#include "smw_2026/Global_waypoints.hpp"
#include <Eigen/Dense>

#define PORT 8080
using namespace std::chrono_literals;


bool is_joint_state_close(std::vector<double> a, std::vector<double> b){
    auto A = Eigen::Vector<double,6>(a[0],a[1],a[2],a[3],a[4],a[5]);
    auto B = Eigen::Vector<double,6>(b[0],b[1],b[2],b[3],b[4],b[5]);
    return A.isApprox(B,0.02);
}

int main(const int argc, const char** argv){

    // ros initialization
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("perception_proxy");
    auto dual_arm_control_interface = std::make_shared<DualArmControlInterface>(node);
    
    // socket initialization
    int sock=0;
    struct sockaddr_in serv_addr;
    const char *hello = "L";

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock<0){
        std::cerr<<"Socket creation error"<<std::endl;
        return -1;
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    if(inet_pton(AF_INET,"192.168.131.1",&serv_addr.sin_addr)<=0){
        std::cerr<<"Invalid address/ Address not supported"<<std::endl;
        return -1;
    }
    if(connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){
        std::cerr << "Connection Failed" << std::endl;
        return -1;
    }

    auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto co1_client = node->create_client<std_srvs::srv::Trigger>("/co1_deconing/execute");
    auto co4_client = node->create_client<std_srvs::srv::Trigger>("/co4_deconing/execute");
    auto so1_client = node->create_client<std_srvs::srv::Trigger>("/so1_deconing/execute");
    auto so3_client = node->create_client<std_srvs::srv::Trigger>("/so3_deconing/execute");

    std::this_thread::sleep_for(std::chrono::seconds(3));

    if(co1_client->wait_for_service(15s)!=co1_client->service_is_ready()){
        RCLCPP_ERROR(node->get_logger(),"Co1 service is not ready");
        return 0;
    }
    if(so1_client->wait_for_service(15s)!=so1_client->service_is_ready()){
        RCLCPP_ERROR(node->get_logger(),"So1 service is not ready");
        return 0;
    }
    if(co4_client->wait_for_service(15s)!=co4_client->service_is_ready()){
        RCLCPP_ERROR(node->get_logger(),"Co4 service is not ready");
        return 0;
    }
    if(so3_client->wait_for_service(15s)!=so3_client->service_is_ready()){
        RCLCPP_ERROR(node->get_logger(),"So3 service is not ready");
        return 0;
    }
    RCLCPP_INFO(node->get_logger(),"All clients are ready, creating the server");

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service =
    node->create_service<std_srvs::srv::Trigger>(
        "decone", 
        [node,sock,hello,co1_client,so1_client,so3_client,co4_client,dual_arm_control_interface](std_srvs::srv::Trigger_Request::SharedPtr req, std_srvs::srv::Trigger_Response::SharedPtr res){
            
            /// this is only for testing
            char buffer[1024] = {0};
            // char buffer[1024] = "C04L";

            auto waypoints = Waypoints();
            // put ros shit here
            // gotta send data here and then start listening
            // send data
            send(sock, hello, strlen(hello), 0);
            std::cout<<"Message sent"<<std::endl;

            // move robots here, if start at preaction : follow the path until wp1, else if at cone_transfer, else anywhere else
            auto left_start_joint_state = dual_arm_control_interface->get_current_joint_state("left");
            auto right_start_joint_state = dual_arm_control_interface->get_current_joint_state("right");

            std::vector<double> left_preaction_state{0.6327896118164062, -1.530074381535389, -2.6847221851348877, -0.4980843824199219, -1.5689991156207483, -2.2041104475604456};
            std::vector<double> right_preaction_state{-2.3459141890155237,-2.1818372211852015,2.570200506840841,-3.2559219799437464,-5.47044271627535,2.995593070983};
            std::vector<double> right_cone_transfer_state{0.9119113789548221, -1.329804040400156, 2.472230381749085, -2.7317907796696113, -4.6925038067020255, 2.23798315279257};

            std::vector<geometry_msgs::msg::Pose> right_waypoints;
            std::vector<double> right_durations;
            std::vector<geometry_msgs::msg::Pose> left_waypoints;
            std::vector<double> left_durations;

            if(is_joint_state_close(left_preaction_state,left_start_joint_state)){
                // left is at preaction, left to wp1
                RCLCPP_INFO(node->get_logger(),"Left at preaction");
                std::vector<geometry_msgs::msg::Pose> to_left_wait_position{waypoints.left_wp0.pose,waypoints.left_wp1.pose};
                left_waypoints = to_left_wait_position;
                std::vector<double> durations{3.0,2.5};
                left_durations = durations;
            }
            else{
                // left is not at preaction
                RCLCPP_INFO(node->get_logger(),"Left not at preaction");
                dual_arm_control_interface->move_to_joint_positions(left_preaction_state,dual_arm_control_interface->left_move_group_interface_);
                std::vector<geometry_msgs::msg::Pose> to_left_wait_position{
                    waypoints.left_wp0.pose,
                    waypoints.left_wp1.pose
                };
                left_waypoints = to_left_wait_position;
                std::vector<double> durations{3.0,2.5};
                left_durations = durations;
            }

            if(is_joint_state_close(right_preaction_state,right_start_joint_state)){
                // right is at preaction
                RCLCPP_INFO(node->get_logger(),"Right at preaction");
                std::vector<geometry_msgs::msg::Pose> to_right_wait_position{
                    waypoints.right_wp0.pose,
                    waypoints.right_wp1.pose
                };
                right_waypoints = to_right_wait_position;
                std::vector<double> durations{1.75,1.75};
                right_durations = durations;
            }
            else if(is_joint_state_close(right_cone_transfer_state,right_start_joint_state)){
                RCLCPP_INFO(node->get_logger(),"Right at cone transfer");
                // right is at cone transfer, need to find waypoints between right cone transfer and wp1
                std::vector<geometry_msgs::msg::Pose> to_right_wait_position{
                    waypoints.right_wp2.pose,
                    waypoints.right_wp3.pose,
                    waypoints.right_wp1.pose
                };
                right_waypoints = to_right_wait_position;
                std::vector<double> durations{3.0,3.0,3.0};
                right_durations = durations;
            }
            else{
                // right is somewhere else
                RCLCPP_INFO(node->get_logger(),"Right not at preaction");
                dual_arm_control_interface->move_to_joint_positions(right_preaction_state,dual_arm_control_interface->right_move_group_interface_);
                std::vector<geometry_msgs::msg::Pose> to_right_wait_position{
                    waypoints.right_wp0.pose,
                    waypoints.right_wp1.pose
                };
                right_waypoints = to_right_wait_position;
                std::vector<double> durations{1.75,1.75};
                right_durations = durations;
            }

            // need to execute the waypoints in cubic simultaneously
            RCLCPP_INFO(node->get_logger(),"Starting Simultaneous Movement, right first");
            auto right_movement_future = dual_arm_control_interface->async_start_execute_waypoints_cubic(right_waypoints,right_durations,0.3,0.2,"right");
            RCLCPP_INFO(node->get_logger(),"Starting Simultaneous Movement, left next");
            auto left_movement_future = dual_arm_control_interface->async_start_execute_waypoints_cubic(left_waypoints,left_durations,0.3,0.2,"left");

            // receive
            read(sock,buffer,1024);

            std::cout<<"Message from the detection server : "<<buffer<<" size of message : "<<std::string(buffer).length()<<std::endl;

            // wait for the movement to complete
            RCLCPP_INFO(node->get_logger(),"Completeing movements");
            dual_arm_control_interface->block_till_response_execute_cubic_trajectory(right_movement_future,std::chrono::seconds(15));
            dual_arm_control_interface->block_till_response_execute_cubic_trajectory(left_movement_future,std::chrono::seconds(15));
            RCLCPP_INFO(node->get_logger(),"Movement finished, starting cone routines");

            if(std::string(buffer) == "C01L" or std::string(buffer) == "C01R"){
                co1_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            }
            else if(std::string(buffer) == "S01L"){
                so1_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            }
            else if(std::string(buffer) == "S03L"){
                so3_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            }
            else if(std::string(buffer) == "C04L"){
                co4_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            }
            else{
                RCLCPP_ERROR(node->get_logger(),"Cone [%s] not supported",buffer);
                dual_arm_control_interface->move_to_joint_positions(left_preaction_state,dual_arm_control_interface->left_move_group_interface_);
                dual_arm_control_interface->move_to_joint_positions(right_preaction_state,dual_arm_control_interface->right_move_group_interface_);
            }
            res->success=true;
        },
        rmw_qos_profile_services_default,
        cb_group
    );

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to act as a tcp proxy for ros1 perception");
    

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();

    // close socket
    close(sock);
    return 0;
}
