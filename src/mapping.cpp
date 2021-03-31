/*
* Know the size of the the map
* and if the node is occupied or not
* getSize() the size of each node is 0.2, which is equal to resolution
* map->isNodeOccupied(*it)check if the node is occupied or not
* using get size on the first node we get the dimensions of the map
*/
#include <mapping.h>
#include <explorer.h>

// Global variables, TODO: make a clean structure
static octomap::OcTree* ourmap; // octree map
ros::Publisher waypoints_pub;   //publisher


void publish_point(octomap::point3d p)
{
    nav_msgs::Path mypath;
    geometry_msgs::PoseStamped waypoints[3];
    const int no_points = 1;
    for(int i=0; i<no_points; i++)
    {
        waypoints[i].pose.position.x = p.x();
        waypoints[i].pose.position.y = p.y();
        if (p.z() < 0.6)
        {
            waypoints[i].pose.position.z = 1;
        }
        else
        {
            waypoints[i].pose.position.z = p.z();
        }
        mypath.poses.push_back(waypoints[i]);
        std::cout << "Point" << p.x() << "," << p.y() << "," << p.z() << "added"<< std::endl;
    }
    waypoints_pub.publish(mypath);
    std::cout << "Path Published" << std::endl;
}

void setOctomapFromBinaryMsg(const octomap_msgs::Octomap& msg) {
    
    ourmap = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(msg));
    float dist = 0;
    octomap::point3d farpoint;

    for (octomap::OcTree::leaf_iterator it = ourmap->begin_leafs();
        it != ourmap->end_leafs(); ++it) 
       {
           if(!ourmap->isNodeOccupied(*it))
           {
                octomap::point3d freepoint = it.getCoordinate();
                if (abs((freepoint.x()*freepoint.x()) + (freepoint.y()*freepoint.y()) 
                    + (freepoint.z()*freepoint.z())) > dist)
                {
                    dist = abs((freepoint.x()*freepoint.x()) + (freepoint.y()*freepoint.y()) 
                        + (freepoint.z()*freepoint.z()));
                    farpoint = freepoint;
                }
           }
       }
    std::cout << "Point: " << farpoint.x() << "," << farpoint.y() << "," << farpoint.z() << std::endl;
    publish_point(farpoint);
    std::cout<<"End of tree" << "\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.2);

    waypoints_pub = n.advertise<nav_msgs::Path>("/planner/waypoints", 1000); // publisher

    //ros::Subscriber sub = n.subscribe("/octomap_binary", 1000, setOctomapFromBinaryMsg);
    
    //create a client
    ros::ServiceClient client = n.serviceClient<octomap_msgs::GetOctomap>("octomap_binary");
    octomap_msgs::GetOctomap srv;
    while (ros::ok())
    {
        bool succeded = client.call(srv);
        if (succeded)
            setOctomapFromBinaryMsg(srv.response.map);
        loop_rate.sleep();
    }
    

    return 0;
}