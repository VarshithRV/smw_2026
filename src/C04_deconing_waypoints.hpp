#include "geometry_msgs/msg/pose.hpp"
#include <vector>

class Waypoints{
public:
    struct Waypoint
    {
        std::vector<double> joint_values;
        geometry_msgs::msg::Pose pose;
    };

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
            {0.280, -0.576, 0.473, 0.529, 0.454, 0.442, 0.564}
        );

        initialize_wp(
            right_wp1,
            {
                -0.44624644914735967,
                -0.32769663751635747,
                2.3091300169574183,
                -3.2120100460448207,
                -5.492254559193746,
                1.2104703187942505
            },
            {0.606, -0.251, 0.010, 0.299, 0.173, 0.368, 0.864}
        );

        initialize_wp(
            right_wp2,
            {
                0.08043240755796432,
                -0.2439463895610352,
                1.1628320852862757,
                -2.484305521050924,
                -4.691539827977316,
                1.5569124221801758
            },
            {0.794, -0.042, 0.215, -0.033, 0.003, -0.010, 0.999}
        );

        initialize_wp(
            right_wp3,
            {
                0.08036408573389053,
                -0.4725662034801026,
                1.249934498463766,
                -2.342678209344381,
                -4.691028896962301,
                1.5571919679641724
            },
            {0.794, -0.043, 0.347, -0.033, 0.003, -0.011, 0.999}
        );

        initialize_wp(
            right_wp4,
            {
                0.08075587451457977,
                -0.45598967493090825,
                1.2053373495685022,
                -2.312906881371969,
                -4.691434685383932,
                1.647143840789795
            },
            {0.794, -0.043, 0.347, -0.165, 0.004, -0.010, 0.986}
        );

        initialize_wp(
            right_wp5,
            {
                0.08618207275867462,
                -0.4123276037028809,
                1.0903084913836878,
                -2.2376591167845667,
                -4.692976776753561,
                1.8583850860595703
            },
            {0.794, -0.043, 0.347, -0.257, 0.005, -0.010, 0.966} 
        );

        initialize_wp(
            right_wp6,
            {
                0.08742086589336395,
                -0.24680276334796147,
                1.018090550099508,
                -2.3306333027281703,
                -4.6935662666903895,
                1.8845486640930176
            },
            {0.793, -0.042, 0.232, -0.257, 0.005, -0.010, 0.966}
        );

        initialize_wp(
            right_wp7,
            {
                0.034685876220464706,
                -0.399868444805481,
                1.9671152273761194,
                -3.1844078503050746,
                -4.667190376912252,
                2.025068998336792
            },
            {0.490, -0.109, 0.214, -0.242, -0.027, -0.017, 0.970}
        );

        initialize_wp(
            right_wp78,
            {
                0.27142732669626113,
                0.7447617755447601,
                1.9517318175224956,
                -2.599008207404107,
                -4.691823526459071,
                1.7002673323978097
            },
            {0.482, 0.036, 0.451, -0.199, 0.005, -0.010, 0.980}
        );

        initialize_wp(
            right_wp8,
            {
                0.5750105977058411,
                -1.3234364551356812,
                2.481882158909933,
                -2.7579080067076625,
                -4.652886692677633,
                2.354644536972046
            },
            {0.110, -0.033, 0.520, 0.628, 0.017, 0.028, -0.777}
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
            {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}
        );

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
            {0.580, 0.195, 0.224, 0.718, -0.003, -0.019, -0.695}
        );

        initialize_wp(
            left_wp2,
            {
                -0.16009885469545537,
                -2.9919687710204066,
                -1.4048900604248047,
                -0.28763802469287114,
                -1.5851691404925745,
                3.227363348007202
            },
            {0.662, 0.119, 0.150, 0.680, 0.003, -0.015, -0.733}
        );

        initialize_wp(
            left_wp3,
            {
                -0.22412521043886358,
                -2.9137731991209925,
                -1.2056723833084106,
                -0.5640774530223389,
                -1.5828593412982386,
                3.291940927505493
            },
            {0.718, 0.058, 0.241, 0.680, 0.003, -0.015, -0.733}
        );

        initialize_wp(
            left_wp4,
            {
                -0.22404033342470342,
                -2.7612558803954066,
                -1.2831690311431885,
                -0.6391633313945313,
                -1.5828960577594202,
                3.2921130657196045
            },
            {0.718, 0.058, 0.321, 0.680, 0.003, -0.015, -0.733}
        );

        initialize_wp(
            left_wp5,
            {
                -0.1558626333819788,
                -2.626902242700094,
                -1.8371628522872925,
                -0.22057004392657475,
                -1.5855796972857874,
                3.22310733795166
            },
            {0.584, 0.134, 0.307, 0.680, 0.003, -0.015, -0.733}
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
            {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}
        );
    }

    void initialize_wp(Waypoint& wp, std::vector<double> joint_values, std::vector<double> pose)
    {
        wp.pose.position.x = pose[0];
        wp.pose.position.y = pose[1];
        wp.pose.position.z = pose[2];
        wp.pose.orientation.w = pose[3];
        wp.pose.orientation.x = pose[4];
        wp.pose.orientation.y = pose[5];
        wp.pose.orientation.z = pose[6];
        wp.joint_values = joint_values;
    }

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
};