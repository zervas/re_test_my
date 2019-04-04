#include <ros/ros.h>
#include "remy/pick.h"
// #include "remy/pour.h"
// #include "remy/place.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "cook");

    // The robot name normaly is taken from the launch file, hardcoded here
    std::string robot_name("left_hand");

    Pick pick(robot_name);
    // Pour pour();         // not implemented
    // Place place();       // not implemented
    // Return return();     // not implemented

    ros::spin();
    return 0;
}