#ifndef REMY_PICK_H_
#define REMY_PICK_H_
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <remy/PickAction.h>
#include "remy/PlanAction.h"
#include <string>
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
    ros::ServiceClient object_detection_ =
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
        nh_, (name + "_pick"), boost::bind(&Pick::executeCB, this, _1),
        false), name_(name) {

        action_.start();
    }

    ~Pick(void) {}

    void executeCB(const remy::PickGoalConstPtr &goal) {
        bool success = true;

        feedback_.status.clear();
        feedback_.status = "Pick for " + name_ + " started";
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
        if (object_detection_.call(srv)) {
            // Give feedback to the client
            feedback_.status = "Object Detection successfull";
            action_.publishFeedback(feedback_);
            // Get the planner's goal
            geometry_msgs::Pose ee_goal = srv.response.goal_pose;
            // setMoveGroup("panda_arm", robot_);   // in FAKE mode
            feedback_.status = "Starting the planner";
            action_.publishFeedback(feedback_);
            //  planner call
            actionlib::SimpleActionClient<remy::PlanAction> plan_client(
                "joint_space_planner", true);
            plan_client.waitForServer();
            remy::PlanGoal goal;
            goal.pose = ee_goal;
            plan_client.sendGoal(goal);
            bool finished_before_timeout =
                plan_client.waitForResult(ros::Duration(5.0));

            if (finished_before_timeout) {
                actionlib::SimpleClientGoalState state = plan_client.getState();
                ROS_INFO("Action finished: %s", state.toString().c_str());
            } else {
                ROS_INFO("Action did not finish before the time out");
                success = false;
            }


        } else {
            ROS_INFO("WTF");
        }

        if (success) {
           result_.status = true;
           // set the action state to succeeded
           action_.setSucceeded(result_);
        } else {
            action_.setPreempted();
        }
    }

    void pick(std::string robot, std::string tool) {
        robot_ = robot;
        run(tool);
    }
};

#endif  // REMY_PICK_H_
