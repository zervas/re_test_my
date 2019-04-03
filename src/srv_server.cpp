#include "ros/ros.h"
#include "remy/TwoInts.h"


bool add(remy::TwoInts::Request &req, remy::TwoInts::Response &res) {
    res.sum = req.a + req.b;
    ROS_INFO("request: x=%d, y=%d", (int)req.a, (int)req.b);
    ROS_INFO("sending back response: [%d]", (int)res.sum);
    return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
//   ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
