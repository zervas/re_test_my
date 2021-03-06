#ifndef REMY_ROBOT_PLANNER_H_
#define REMY_ROBOT_PLANNER_H_
// ROS
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <string>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// REMY
#include "remy/PlanAction.h"



class RobotPlanner {
 protected:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface *move_group_;
    const robot_state::JointModelGroup* joint_model_group_;
    moveit::planning_interface::MoveGroupInterface::Plan movement_;
    double planning_time_;
    std::string action_name_;
    // Action
    actionlib::SimpleActionServer<remy::PlanAction> action_;
    remy::PlanFeedback feedback_;
    remy::PlanResult result_;


    void setMoveGroup(const std::string group_name,
                      const std::string description) {
        /* This is the function that chooses robot through its namespace */
        /*  -- description: /namespace/robot_description                 */
        static const moveit::planning_interface::MoveGroup::Options options_(
            group_name, description, nh_);
        const ros::WallDuration wait_for(3);
        move_group_ = new moveit::planning_interface::MoveGroupInterface(
            options_, NULL, wait_for);
        joint_model_group_ =
            move_group_->getCurrentState()->getJointModelGroup(group_name);
    }

    void setPlanningScene() { /* TODO */ }

    void setPlanningTime(double time) {
        planning_time_ = time;
    }

    void execute() {
        move_group_->move();
    }

 public:
    RobotPlanner() : action_(
        nh_, "joint_space_planner", boost::bind(&RobotPlanner::executeCB,
        this, _1), false), planning_time_(10.0) {

        action_.start();
    }

    virtual ~RobotPlanner() {}

    void executeCB(const remy::PlanGoalConstPtr &goal) {
        /* Very simple implementation (possibly non complete), for  */
        /*  purposes of the 'test' only.                            */
        // moveit::core::RobotStatePtr current_state =
        //     move_group_->getCurrentState();
        // move_group_->setStartState(*current_state);
        // move_group_->setPoseTarget(goal->pose);
        // move_group_->setPlanningTime(planning_time_);
        // // Plan
        // bool success = (move_group_->plan(movement_) ==
        //     moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // execute();
        feedback_.status = "planner is running";
        action_.publishFeedback(feedback_);
        feedback_.status = "planner is executing";
        action_.publishFeedback(feedback_);

        result_.status = true;
        action_.setSucceeded(result_);
    }
};

#endif  // REMY_ROBOT_PLANNER_H_
