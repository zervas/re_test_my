#include <ros/ros.h>
#include "remy/pick.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "chef");

    // The robot to command
    std::string robot_name("left_hand");

    actionlib::SimpleActionClient<remy::PickAction> client(
        robot_name + "_pick", true);
    client.waitForServer();
    remy::PickGoal goal;
    goal.tool = "tomato";
    client.sendGoal(goal);

    bool finished_before_timeout =
        client.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out");
    }

    ros::spin();
    return 0;
}