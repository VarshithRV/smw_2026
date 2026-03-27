#include "geometry_msgs/msg/pose.hpp"
#include <vector>

class Waypoints{
public:
    struct Waypoint
    {
        std::vector<double> joint_values;
        geometry_msgs::msg::Pose pose;
    };

    Waypoint right_rest;
    Waypoint right_wp1;
    Waypoint right_wp2;
    Waypoint right_wp3;
    Waypoint right_wp4;
    Waypoint right_wp5;
    Waypoint right_wp7;
    Waypoint right_wp78;
    Waypoint right_wp8;

    Waypoint left_rest;
    Waypoint left_wp1;
    Waypoint left_wp2;
    Waypoint left_wp3;
    Waypoint left_wp4;
    Waypoint left_wp5;
    Waypoint left_home;

    Waypoints()
    {
        // right side first
        initialize_wp(
        	right_rest, // same as all other rests
        	{
        		-2.3459141890155237,
        		-2.1818372211852015,
        		2.570200506840841,
        		-3.2559219799437464,
        		-5.47044271627535,
        		2.9955930709838867
        	},
        	{}
        );

        // task space approach to the cone from the side
        initialize_wp(
        	right_wp1,
        	{},
        	{
        		0.510, -0.741, 0.363,
        		0.479, 0.270, 0.386, 0.740
        	}
        );
        initialize_wp(
        	right_wp2,
        	{},
        	{
        		0.717, -0.455, 0.363,
        		0.359, 0.041, 0.294, 0.885
        	}
        );
        initialize_wp(
        	right_wp3,
        	{},
        	{
        		0.767, -0.295, 0.336,
        		0.00, -0.000, -0.00, 1.000
        	}
        );
        initialize_wp(
        	right_wp4,
        	{},
        	{
        		0.767, -0.195, 0.336,
        		0.00, -0.000, -0.00, 1.000
        	}
        );
        initialize_wp(
        	right_wp5,
        	{},
        	{
        		0.767, -0.035, 0.351,
        		0.00, -0.000, -0.00, 1.000
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


        // left side
        initialize_wp( // same as other left rests
        	left_rest,
        	{
        		0.6327896118164062,
        		-1.530074381535389,
        		-2.6847221851348877,
        		-0.4980843824199219,
        		-1.5689991156207483,
        		-2.2041104475604456
        	},
        	{
                0.323, 0.419, 0.579,
                1,0,0,0
            }
        );

        // left approach to the cone
        initialize_wp(
        	left_wp1,
        	{},
        	{
        		0.504, 0.306, 0.452,
        		1,0,0,0
        	}
        );
        initialize_wp(
        	left_wp2,
        	{},
        	{
        		0.597, 0.117, 0.247,
        		1,0,0,0
        	}
        );
        initialize_wp( // stop here
        	left_wp3,
        	{},
        	{
        		0.768, 0.008, 0.217,
                0.791, 0.001, 0.001, -0.612
        		// 0.894, 0.001, 0.001, -0.449
        	}
        );

        initialize_wp( // straight up and close gripper and stop
        	left_wp4,
        	{},
        	{
        		0.768, 0.008, 0.310,
        		0.791, 0.001, 0.001, -0.612 
                // 0.894, 0.001, 0.001, -0.449
        	}
        ); 

        initialize_wp( // twist until unlock
        	left_wp5,
        	{},
        	{
        		0.768, 0.008, 0.310,
                0.838, 0.001, -0.001, 0.546
                // -0.159, -0.001, -0.001, 0.987
        		// 0.670, 0.000, -0.001, 0.742
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