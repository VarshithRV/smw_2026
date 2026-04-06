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

    // bind the socket
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

    std::this_thread::sleep_for(std::chrono::seconds(3));

    int i = 0;
    int correct = 0;

    while(i<20){
        
        char buffer[1024] = {0};
        send(sock, hello, strlen(hello), 0);
        std::cout<<"Message sent"<<std::endl;
        

        read(sock,buffer,1024);
        std::cout<<"Message from the detection server : "<<buffer<<" size of message : "<<std::string(buffer).length()<<std::endl;
        if(std::string(buffer) == "C01L"){
            correct +=1;
        }
        else if(std::string(buffer) == "S01L"){
            // nothing
        }
        else if(std::string(buffer) == "S03L"){
            // nothing
        }
        else if(std::string(buffer) == "C04L"){
            // nothing
        }
        else{
            RCLCPP_ERROR(node->get_logger(),"Cone [%s] not supported",buffer);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success : %f", correct/20);

    // close socket
    close(sock);
    return 0;
}
