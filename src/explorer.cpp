/*
Frontier based exploration

simple waypoint publisher
*/
#include <explorer.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "explorer_node");
  ros::NodeHandle n;
  ros::Publisher waypoints_pub = n.advertise<nav_msgs::Path>("/planner/waypoints", 1000);
  ros::Rate loop_rate(10);

  nav_msgs::Path mypath;
  geometry_msgs::PoseStamped single_pose;
  single_pose.pose.position.x =2;
  mypath.poses.push_back(single_pose);


  while (ros::ok())
  {
    waypoints_pub.publish(mypath);

    ros::spinOnce();
    loop_rate.sleep();
  }

  
  return 0;
}
