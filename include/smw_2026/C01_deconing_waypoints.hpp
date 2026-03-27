#include "geometry_msgs/msg/pose.hpp"
#include <vector>

class Waypoints{
public:
    struct Waypoint
    {
        std::vector<double> joint_values;
        geometry_msgs::msg::Pose pose;
    };

    // deconing program for C01
    Waypoint right_rest_state;
    Waypoint right_wp0;
    Waypoint right_wp1;
    Waypoint right_wp2;
    // at the corner cast
    Waypoint right_wp3;
    // - z 90 deg
    Waypoint right_wp4;
    // down
    Waypoint right_wp5;
    // come back
    Waypoint right_wp6;
    Waypoint right_wp7;
    // come back all the way
    Waypoint right_wp8;

    Waypoints()
    {
        initialize_wp(
            right_rest_state,
            {
                -2.3459141890155237,
                -2.1818372211852015,
                2.570200506840841,
                -3.2559219799437464,
                -5.47044271627535,
                2.9955930709838867
            },
            {
                0.280, -0.576, 0.473,
                0.529, 0.454, 0.442, 0.564
            }
        );

        // goin to cc
        initialize_wp(
            right_wp0,
            {},
            {
                0.619, -0.591, 0.260,
                0.321, 0.242, 0.494, 0.771
            }
        );

        // goin to cc
        initialize_wp(
            right_wp1,
            {},
            {
                0.734, -0.295, 0.137,
                0.294, 0.060, 0.370, 0.879
            }
        );

        initialize_wp(
            right_wp2,
            {},
            {
                0.768, -0.022, 0.196,
                0.003, -0.004, -0.029, 1.000
            }
        );

        initialize_wp(
            right_wp3,
            {},
            {
                0.768, -0.0, 0.370,
                0.003, -0.004, -0.029, 1.000
            }
        );

        initialize_wp(
            right_wp4,
            {},
            {
                0.768, -0.0, 0.370,
                0.785, -0.025, -0.013, 0.618
            }
        );

        initialize_wp(
            right_wp5,
            {},
            {
                0.768, -0.0, 0.196,
                0.785, -0.025, -0.013, 0.618
            }
        );

        initialize_wp(
            right_wp6,
            {},
            {
                0.504, -0.100, 0.348,
                0.002, -0.014, -0.001, 1.000
            }
        );

        initialize_wp(
            right_wp7,
            {},
            {
                0.3, -0.05, 0.50,
                0.710, 0.008, 0.011, -0.704
            }
        );

        initialize_wp(
            right_wp8,
            {},
            {
                -0.022, 0.005, 0.520,
                0.710, 0.008, 0.011, -0.704
            }
        );
    }

    void initialize_wp(Waypoint& wp, std::vector<double> joint_values, std::vector<double> pose)
    {
        if(pose.size()!=0){
            wp.pose.position.x = pose[0];
            wp.pose.position.y = pose[1];
            wp.pose.position.z = pose[2];
            wp.pose.orientation.w = pose[3];
            wp.pose.orientation.x = pose[4];
            wp.pose.orientation.y = pose[5];
            wp.pose.orientation.z = pose[6];
        }
        wp.joint_values = joint_values;
    }
};