#include <ros/ros.h>
#include "remy/ObjectDetection.h"
#include "geometry_msgs/Pose.h"
#include <string>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <remy/RobotPlanAction.h>



class RobotPlanner {
 protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<remy::RobotPlanAction> my_action_;
    moveit::planning_interface::MoveGroupInterface *move_group_;
    const robot_state::JointModelGroup* joint_model_group_;
    moveit::planning_interface::MoveGroupInterface::Plan movement_;
    double planning_time_;
    // Action
    // remy::RobotPlanFeedback my_feedback_;
    // remy::RobotPlanResult my_result_;


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

    void plan(geometry_msgs::Pose pose) {
        moveit::core::RobotStatePtr current_state =
            move_group_->getCurrentState();
        move_group_->setStartState(*current_state);

        move_group_->setPoseTarget(pose);

        move_group_->setPlanningTime(planning_time_);
        // Plan
        bool success = (move_group_->plan(movement_) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    void execute() {
        move_group_->move();
    }

 private:
    /* data */
 public:
    RobotPlanner() {
        // TODO(Get these from rosparam set by launch file)
        planning_time_ = 10.0;
    }

    virtual ~RobotPlanner(){}
};
