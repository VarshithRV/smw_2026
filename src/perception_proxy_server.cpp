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

int main(const int argc, const char** argv){

    // ros initialization
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("perception_proxy");
    
    // socket initialization
    int sock=0;
    struct sockaddr_in serv_addr;
    const char *hello = "Hello from client";

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock<0){
        std::cerr<<"Socket creation error"<<std::endl;
        return -1;
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    if(inet_pton(AF_INET,"127.0.0.1",&serv_addr.sin_addr)<=0){
        std::cerr<<"Invalid address/ Address not supported"<<std::endl;
        return -1;
    }
    if(connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){
        std::cerr << "Connection Failed" << std::endl;
        return -1;
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service =
    node->create_service<std_srvs::srv::Trigger>("perception_tcp_proxy", 
        [sock,hello](std_srvs::srv::Trigger_Request::SharedPtr req, std_srvs::srv::Trigger_Response::SharedPtr res){
            char buffer[1024] = {0};
            // put ros shit here
            // gotta send data here and then start listening
            // send data
            send(sock, hello, strlen(hello), 0);
            std::cout<<"Message sent"<<std::endl;

            // receive
            read(sock,buffer,1024);
            std::cout<<"Message from the server : "<<buffer<<std::endl;
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
