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
    // move j or cubic
    Waypoint right_wp1;
    Waypoint right_wp2;
    // move l and wait till the left unlocks
    Waypoint right_wp3;
    // move l, rotate just a little once left unlocks and let left back away
    Waypoint right_wp4;
    // move l, full unlock and let left back away farther
    Waypoint right_wp5;
    // move l downwards, and let left completely get clear
    Waypoint right_wp6;
    // move l get the cone back
    Waypoint right_wp7;
    Waypoint right_wp78;
    Waypoint right_wp8;


    Waypoint left_rest;
    // move j
    Waypoint left_wp1;
    // move l
    Waypoint left_wp2;
    // move l
    Waypoint left_wp3;
    // move l, this is unlock, after this waypoint, wait for a while for the right to twist a little bit
    Waypoint left_wp4;
    // wait
    // move l to back away a little and let the right arm descend
    Waypoint left_wp5;
    // let the right arm descend
    // movej to go back to home
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
            {}
        );

        // goin to cc
        initialize_wp(
            right_wp1,
            {
                -0.45845014253725225,
                -0.8516943019679566,
                2.167271439229147,
                -1.531251722281315,
                -5.1756232420550745,
                1.7074766159057617
            },
            {}
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
                0.768, -0.022, 0.350,
                0.003, -0.004, -0.019, 1.000 
            }
        );

        // small twist
        initialize_wp(
            right_wp4,
            {},
            {
                0.768, -0.022, 0.350, 
                -0.165, 0.004, -0.010, 0.986
            }
        );

        // big twist
        initialize_wp(
            right_wp5,
            {},
            {
                0.768, -0.022, 0.350,
                -0.257, 0.005, -0.010, 0.966
            } 
        );

        // get down
        initialize_wp(
            right_wp6,
            {},
            {0.793, -0.042, 0.232, -0.257, 0.005, -0.010, 0.966}
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
                0.7448494020060257,
                -1.6898253014238567,
                -2.456773372167499,
                -0.6417658203742237,
                -1.5154170572868646,
                2.4306718044994686
            },
            {}
        );

        // going to the cc
        initialize_wp(
            left_wp1,
            {
                -0.0617602507220667,
                -2.7942682705321253,
                -1.8408832550048828,
                -0.051759080295898485,
                -1.5994308630572718,
                3.23614501953125
            },
            {}
        );

        // below cc
        initialize_wp(
            left_wp3,
            {},
            {
                // 0.718, 0.058, 0.241, 
                0.696, 0.058, 0.229,
                0.680, 0.003, -0.015, -0.733,
            }
        );

        // unlock
        initialize_wp(
            left_wp4,
            {},
            {
                0.696, 0.058, 0.329, 
                0.680, 0.003, -0.015, -0.733
            }
        );

        // back off
        initialize_wp(
            left_wp5,
            {},
            {
                0.584, 0.134, 0.307, 
                0.680, 0.003, -0.015, -0.733
            }
        );

        initialize_wp(
            left_home,
            {
                0.7448494020060257,
                -1.6898253014238567,
                -2.456773372167499,
                -0.6417658203742237,
                -1.5154170572868646,
                2.4306718044994686
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