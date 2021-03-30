/*
Frontier based exploration

TODO:
* [done] send a single point
* [done]send three point(manually) and make the drone follow
simple waypoint publisher
*/
#include <explorer.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "explorer_node");
  ros::NodeHandle n;
  ros::Publisher waypoints_pub = n.advertise<nav_msgs::Path>("/planner/waypoints", 1000);
  ros::Rate loop_rate(0.03);

  nav_msgs::Path mypath;
  geometry_msgs::PoseStamped waypoints[3];

  waypoints[0].pose.position.x = -2;
  waypoints[1].pose.position.x = -2.5;
  waypoints[2].pose.position.x = -3;

  waypoints[0].pose.position.z =4;
  waypoints[1].pose.position.z =4;
  waypoints[2].pose.position.z =4;

  for (int i = 0; i < 3; i++)
  {
    mypath.poses.push_back(waypoints[i]);
  }
  

  while (ros::ok())
  {
    loop_rate.sleep();  // Wait for 30 second
    waypoints_pub.publish(mypath);
    ROS_INFO("data published");

    ros::spinOnce();
    loop_rate.sleep();
  }

  
  return 0;
}
