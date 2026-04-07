#include "geometry_msgs/msg/pose.hpp"
#include <vector>

class Waypoints{
public:
    struct Waypoint
    {
        std::vector<double> joint_values;
        geometry_msgs::msg::Pose pose;
    };

    Waypoint right_rest_state;
    Waypoint right_wp0;
    Waypoint right_wp1;
    Waypoint right_wp2;
    Waypoint right_wp3;
    Waypoint right_wp4;
    Waypoint right_wp5;
    Waypoint right_wp6;
    Waypoint right_wp7;
    Waypoint right_wp78;
    Waypoint right_wp8;

    Waypoint left_rest;
    Waypoint left_wp0;
    Waypoint left_wp1;
    Waypoint left_wp2;
    Waypoint left_wp3;
    Waypoint left_wp4;
    Waypoint left_wp5;
    Waypoint left_wp6;
    Waypoint left_home;

    Waypoints()
    {
        // right side first

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

        // goin to cc, this is a good place to stay while waiting for perception
        initialize_wp(
            right_wp1,
            {},
            {
                0.734, -0.295, 0.137,
                // 0.294, 0.060, 0.370, 0.879
                0.321, 0.242, 0.494, 0.771
            }
        );

        // below cc
        initialize_wp(
            right_wp2,
            {},
            {
                0.768, -0.022, 0.196,
                0.003, -0.004, -0.029, 1.000
            }
        );

        // at cc
        initialize_wp(
            right_wp3,
            {},
            {
                0.768, -0.022, 0.374,
                0.003, -0.004, -0.019, 1.000 
            }
        );

        //// CODE SPECIFIC TO SO1




        // get down
        initialize_wp(
            right_wp6,
            {},
            {
                0.768, -0.022, 0.196,
                0.003, -0.004, -0.019, 1.000 
            }
        );

        // back off
        initialize_wp(
            right_wp7,
            {},
            {
                0.504, -0.100, 0.348,
                0.002, -0.014, -0.001, 1.000
            }
        );

        initialize_wp(
            right_wp78,
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

        // left side next
        initialize_wp(
            left_rest,
            {
                0.6327896118164062, 
                -1.530074381535389, 
                -2.6847221851348877, 
                -0.4980843824199219, 
                -1.5689991156207483, 
                -2.2041104475604456
            },
            {}
        );

        // left approach to the cone
        initialize_wp(
        	left_wp0,
        	{},
        	{
        		0.504, 0.306, 0.452,
        		0.680, 0.003, -0.015, -0.733,
        	}
        );

        // good place to wait for left
        initialize_wp(
        	left_wp1,
        	{},
        	{
        		0.597, 0.117, 0.247,
        		0.680, 0.003, -0.015, -0.733,
        	}
        );
        
        //// CODE SPECIFIC TO SO1
        initialize_wp(
            left_wp2,
            {},
            {
                0.765, 0.00, 0.179,
                1.00, -0.000, -0.0, 0.000
            }
        );

        // at the cone
        initialize_wp(
            left_wp3,
            {},
            {
                0.765, 0.00, 0.297,
                1.00, -0.000, -0.0, 0.000
            }
        );

        // gripper on
        // twist the cone
        initialize_wp(
            left_wp4,
            {},
            {
                0.765, 0.00, 0.297,
                0.297, -0.000, -0.002, 0.955
            }
        );

        // gripper off
        // left get down
        initialize_wp(
            left_wp5,
            {},
            {
                0.765, 0.00, 0.179,
                0.297, -0.000, -0.002, 0.955
            }
        );

        // back off
        initialize_wp(
            left_wp6,
            {},
            {
                0.584, 0.134, 0.179,
                0.297, -0.000, -0.002, 0.955
            }
        );

        initialize_wp(
            left_home,
            {
                0.6327896118164062, 
                -1.530074381535389, 
                -2.6847221851348877, 
                -0.4980843824199219, 
                -1.5689991156207483, 
                -2.2041104475604456
            },
            {}
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