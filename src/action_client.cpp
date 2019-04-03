#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <remy/FibonacciAction.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_fibonacci");

    actionlib::SimpleActionClient<remy::FibonacciAction> ac("fibonacci", true);

    ROS_INFO("Waiting for action server to start");

    ac.waitForServer();

    ROS_INFO("Action server started, sending goal");
    remy::FibonacciGoal goal;
    goal.order = 20;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(
        30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state  ac.getState();
        ROS_INFO("Action finished %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out");
    }

    return 0;
}