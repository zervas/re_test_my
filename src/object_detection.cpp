#include "ros/ros.h"
#include "remy/ObjectDetection.h"


bool get_pose(remy::ObjectDetection::Request &req,
         remy::ObjectDetection::Response &res) {

    // call the object detection service here that takes as
    // parameter the object to detect (tool, pizza, etc)
    /* geometry_msgs/Pose ee_goal = get_goal_pose(req.object) */
    // Fake the result
    res.goal_pose.position.x = 0.232;
    res.goal_pose.position.y = 0.1;
    res.goal_pose.position.z = 0.4;
    res.goal_pose.orientation.w = 1.2;

    return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "end_effector_goal");
  ros::NodeHandle node_handle;

  ros::ServiceServer service = node_handle.advertiseService(
      "object_detection", get_pose);

  ros::spin();
  return 0;
}