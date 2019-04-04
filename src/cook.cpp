#include <ros/ros.h>
#include "remy/pick.h"
// #include "remy/pour.h"
// #include "remy/place.h"



int main(int argc, char **argv) {
    ros::init(argc, argv, "cook");

    std::string robot_name("left_hand");

    Pick pick(robot_name);
    // Pour pour();
    // Place place();
    // Return return();

    ros::spin();
    return 0;
}