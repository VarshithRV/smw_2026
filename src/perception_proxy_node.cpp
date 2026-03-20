#include "ros/ros.h"
#include "std_msgs/String.h"
#include "thread"
#include "chrono"

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <cstring>
#include <iostream>
#include <string>

#define PORT 8080

std::string cone;
using namespace std::chrono_literals;

void wp2_chatter_callback(const std_msgs::String::ConstPtr& str){
    cone = str->data;
    ROS_INFO("Received data : %s", cone.c_str());
}

int main(int argc, char** argv){

    // ros stuff
    ros::init(argc, argv, "percpetion_proxy_node");
    ros::NodeHandle nh;

    auto sub = nh.subscribe("/wp2_chatter", 1000, wp2_chatter_callback);
    ros::Publisher publisher = nh.advertise<std_msgs::String>("/robot_status", 1000);

    // tcp stuff
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt failed");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0) {
        perror("listen failed");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    ROS_INFO("Perception Proxy node TCP server listening on port %d", PORT);

    while (ros::ok()) {

        ROS_WARN("Waiting for TCP client to connect...");
        new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);

        if (new_socket < 0) {
            perror("accept failed");
            std::this_thread::sleep_for(1s);
            continue;
        }

        ROS_WARN("TCP client connected");

        while (ros::ok()) {
            char buffer[1024] = {0};

            ssize_t bytes_read = read(new_socket, buffer, 1023);

            if (bytes_read < 0) {
                perror("read failed");
                ROS_WARN("Read failed. Assuming client connection is broken.");
                close(new_socket);
                break;
            }

            if (bytes_read == 0) {
                ROS_WARN("Client disconnected. Waiting for reconnection...");
                close(new_socket);
                break;
            }

            buffer[bytes_read] = '\0';
            ROS_INFO("Received req from proxy server: %s", buffer);

            // this should be in the tcp loop
            std_msgs::String msg;
            msg.data = "L";
            publisher.publish(msg);

            ros::spinOnce();

            auto received_msg =
                ros::topic::waitForMessage<std_msgs::String>("/wp2_chatter", nh, ros::Duration(30.0));

            std::string reply;

            if (received_msg == nullptr) {
                ROS_ERROR("No data received yet");
                reply = "ERROR:NO_DATA";
            } else {
                ROS_INFO("Cone : %s", received_msg->data.c_str());
                reply = received_msg->data;
            }

            ssize_t bytes_sent = send(new_socket, reply.c_str(), reply.size(), 0);

            if (bytes_sent < 0) {
                perror("send failed");
                ROS_WARN("Send failed. Assuming client disconnected. Waiting for reconnection...");
                close(new_socket);
                break;
            }

            ROS_INFO("Done listening now");
            ros::spinOnce();
        }
    }

    close(server_fd);
    return 0;
}