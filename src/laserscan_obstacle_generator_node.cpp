#include <laserscan_obstacle_generator/laserscan_obstacle_generator.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserscan_obstacle_generator");
  ros::NodeHandle pnh("~");

#ifdef DEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  LaserscanObstacleGenerator laserscan_obstacle_generator(pnh);
  if (!laserscan_obstacle_generator.init()) {
    return 1;
  }
  else {
    ROS_INFO("Laserscan obstacle generation node succesfully initialized");
  }

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}
