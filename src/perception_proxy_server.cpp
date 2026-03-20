// this will host a server in ros2 that will call the perception
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <memory>

#define PORT 8080
using namespace std::chrono_literals;

int main(const int argc, const char** argv){

    // ros initialization
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("perception_proxy");
    
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
    node->create_service<std_srvs::srv::Trigger>("decone", 
        [node,sock,hello,co1_client,so1_client,so3_client,co4_client](std_srvs::srv::Trigger_Request::SharedPtr req, std_srvs::srv::Trigger_Response::SharedPtr res){
            char buffer[1024] = {0};
            // put ros shit here
            // gotta send data here and then start listening
            // send data
            send(sock, hello, strlen(hello), 0);
            std::cout<<"Message sent"<<std::endl;

            // receive
            read(sock,buffer,1024);
            std::cout<<"Message from the detection server : "<<buffer<<" size of message : "<<std::string(buffer).length()<<std::endl;
            if(std::string(buffer) == "C01L"){
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
            }
            res->success=true;
        }
    );

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to act as a tcp proxy for ros1 perception");
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    // close socket
    close(sock);
    return 0;
}
