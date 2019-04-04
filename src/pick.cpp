#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <remy/PickAction.h>
#include "remy/ObjectDetection.h"
#include "remy/robot_planner.h"


class Pick : RobotPlanner {
 protected:
    ros::NodeHandle nh_;
    std::string name_;
    std::string robot_;
    std::string tool_;
    actionlib::SimpleActionServer<remy::PickAction> action_;
    remy::PickResult result_;
    remy::PickFeedback feedback_;
    // * The client for the object detection service
    ros::ServiceClient object_detection =
        nh_.serviceClient<remy::ObjectDetection>("object_detection");
    geometry_msgs::Pose goal_pose_;

 private:
    void run(std::string &tool) {
        actionlib::SimpleActionClient<remy::PickAction> client(
            "pick_client", true);
        client.waitForServer();
        remy::PickGoal goal;
        goal.tool = tool;
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
    Pick(std::string name) : action_(
        nh_, name, boost::bind(&Pick::executeCB, this, _1),
        false), name_(name) {

        action_.start();
    }

    ~Pick(void) {}

    void executeCB(const remy::PickGoalConstPtr &goal) {
        bool success = true;

        feedback_.status.clear();
        feedback_.status.push_back("'Pick' for" + name_ + " started");
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
        srv.request.object = goal->tool;
        if (object_detection.call(srv)) {
            // Give feedback to the client
            feedback_.status.push_back("Object Detection successfull");
            action_.publishFeedback(feedback_);
            // Get the planner's goal
            geometry_msgs::Pose ee_goal = srv.response.goal_pose;
            setMoveGroup("panda_arm", robot_);
            feedback_.status.push_back("Starting the planner");
            action_.publishFeedback(feedback_);
            plan(ee_goal);
            // Check for the success of the planner
            /* ... */
            result_.status = true;
            action_.setSucceeded(result_);
        }
    }

    void pick(std::string robot, std::string tool) {
        robot_ = robot;
        run(tool);
    }
};


// int main(int argc, char **argv) {
//     ros::init(argc, argv, "pick_action");

//     Pick pick("left_robot");

//     ros::spin();
//     return 0;
// }