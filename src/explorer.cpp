/*
Frontier based exploration

TODO:
* [done] send a single point
* [done]send three point(manually) and make the drone follow
simple waypoint publisher
*/
#include <explorer.h>


void trigger_rotate()
{ 
  
  //Create the path and set the header
  nav_msgs::Path mypath;
  mypath.header.frame_id = 'R';

  const int no_points = 3;
  geometry_msgs::PoseStamped waypoints[no_points];
  
  tf2::Quaternion angle;
  double rad_angle = angles::from_degrees(90);
  angle.setRPY(0, 0, rad_angle);
  geometry_msgs::Quaternion myangle = tf2::toMsg(angle);

  for(int i=0; i<no_points; i++)
  {
      waypoints[i].pose.orientation = myangle;
      waypoints[i].pose.position.x = 3;
      waypoints[i].pose.position.y = 3;
      waypoints[i].pose.position.z = 3;      
      mypath.poses.push_back(waypoints[i]);
  }
  
  myrotate.publish(mypath);  //publish the path
}

/*
int main(int argc, char** argv)
{
  ros::init(argc, argv, "explorer_node");
  ros::NodeHandle n;
  waypoints_pub = n.advertise<nav_msgs::Path>("/planner/waypoints", 1000);  //change to rottate_pub
  ros::Rate loop_rate(1);  

  while (ros::ok())
  {
    loop_rate.sleep();  // Wait for 30 second
    trigger_rotate();
    ROS_INFO("data published");

    ros::spinOnce();
    //loop_rate.sleep();
  }

  
  return 0;
}
*/