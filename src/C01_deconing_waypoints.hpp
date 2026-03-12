#include "geometry_msgs/msg/pose.hpp"
#include <cmath>

class Waypoints{
    public:
        struct Waypoint
        {
            std::vector<double> joint_values;
            geometry_msgs::msg::Pose pose;
        };

        Waypoints(){
            initialize_wp(wp1,{1,0,0,0,0,0},{3,4,0,1,0,0,0});
            initialize_wp(wp2,{0,0,0,0,0,0},{0,0,0,1,0,0,0});
            initialize_wp(wp3,{0,0,0,0,0,0},{0,0,0,1,0,0,0});
        }

        void initialize_wp(Waypoint& wp,std::vector<double> joint_values,std::vector<double> pose){
            wp.pose.position.x = pose[0];
            wp.pose.position.y = pose[1];
            wp.pose.position.z = pose[2];
            wp.pose.orientation.w = pose[3];
            wp.pose.orientation.x = pose[4];
            wp.pose.orientation.y = pose[5];
            wp.pose.orientation.z = pose[6];
            wp.joint_values=joint_values;
        }

        Waypoint wp1,wp2,wp3;        

};