#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <remy/PickAction.h>
#include "remy/ObjectDetection.h"
#include "geometry_msgs/Pose.h"
#include "remy/robot_planner.h"


class PickAction : RobotPlanner {
 protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<remy::PickAction> action_;
    std::string action_name_;   // * <---
    remy::PickFeedback feedback_;
    remy::PickResult result_;
    std::string object_type_;   // * <---
    // * The client for the object detection service
    ros::ServiceClient object_detection =
        nh_.serviceClient<remy::ObjectDetection>("object_detection");
    geometry_msgs::Pose goal_pose_;

    void run() {
        actionlib::SimpleActionClient<remy::PickAction> client(
            "pick_client", true);
        client.waitForServer();
        remy::PickGoal goal;
        goal.pose = goal_pose_;
        client.sendGoal(goal);

        bool finished_before_timeout =
            client.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = client.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        } else {
            ROS_INFO("Action did not finish before the time out");
        }
    }

 public:
    PickAction(std::string name) : action_(
        nh_, name, boost::bind(&PickAction::executeCB, this, _1),
        false), action_name_(name) {

        action_.start();
    }

    ~PickAction(void) {}

    void executeCB(const remy::PickGoalConstPtr &goal) {
        bool success = true;


        feedback_.status.clear();
        feedback_.status.push_back("Received new pick command");
        action_.publishFeedback(feedback_);

        if (action_.isPreemptRequested() || !ros::ok()) {
            // Notify the client for pre-emption
            action_.setPreempted();
            result_.status = false;
            action_.setSucceeded(result_);
        }

        // Call object detectin to get the end effector goal position
        remy::ObjectDetection srv;
        // srv.request.object = goal->pose;
        srv.request.object = "tool";
        if (object_detection.call(srv)) {
            // Give feedback to the clien
            feedback_.status.push_back("Object Detection successfull");
            action_.publishFeedback(feedback_);
            // Get the planner's goal
            geometry_msgs::Pose ee_goal = srv.response.goal_pose;
            ROS_INFO("Goal position x: %d", ee_goal.position.x);
            // Call the plan actin
            // geometry_msgs::Pose result;
            // result.position.x = 0.1;
            // result.position.y = 0.2;
            // result.position.z = 0.3;
            // result.orientation.z = 0.4;
            // result_.status = true;
            // action_.setSucceeded(result);
            setMoveGroup("panda_arm", "robot1/robot_description");
        }
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_action");

    PickAction pick("pick_action");

    ros::spin();
    return 0;
}