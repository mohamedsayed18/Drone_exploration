/*
* Know the size of the the map
* and if the node is occupied or not
* getSize() the size of each leaf node is 0.2, which is equal to resolution
* map->isNodeOccupied(*it)check if the node is occupied or not
* using get size on the first node we get the dimensions of the map
* begin_leafs_bbx(min_point, max_point), this iterate leafs in a certain bound
* can I get a leaf not at the last depth
*/
#include <mapping.h>
#include <explorer.h>

// Global variables, TODO: make a clean structure
static octomap::OcTree* ourmap; // octree map
ros::Publisher waypoints_pub;   //publisher

void loop_nodes(octomap::OcTree* map)
{
    std::cout << "Size "<< map->size() << std::endl;
    std::cout << "Depth "<< map->getTreeDepth() << std::endl;
    for (octomap::OcTree::tree_iterator it = map->begin_tree();
        it != map->end_tree(); it++)
    {
       print(it.getCoordinate());
       std::cout << "Depth " << it.getDepth() << std::endl;
    }
    
}

void loop_leafs(octomap::OcTree* map)
{
    for (octomap::OcTree::leaf_iterator it = map->begin_leafs();
    it != map->end_leafs(); ++it) 
    {
        print(it.getCoordinate());
        std::cout << "Depth " << it.getDepth() << std::endl;
        /*
        if(!map->isNodeOccupied(*it))
        {
            octomap::point3d freepoint = it.getCoordinate();
            if (distance(freepoint, position) > dist)
            {
                dist = distance(freepoint, position);
                farpoint = freepoint;
            }
        }
        */
    }
}

void loop_leafs_bbx(octomap::OcTree* map)
{
    octomap::point3d min_point(0, 0, 0);
    octomap::point3d max_point(2000, 2000, 2000);

    std::cout << "Size: "<< map->size() << std::endl;
    std::cout << "Depth: "<< map->getTreeDepth() << std::endl;

    for (octomap::OcTree::leaf_bbx_iterator it = map->begin_leafs_bbx(min_point, max_point);
        it != map->end_leafs_bbx(); ++it)
        {
            print(it.getCoordinate());
            std::cout << "Depth " << it.getDepth() << std::endl;
        }
}

void print(octomap::point3d p, std::string title)
{
    std::cout << title << " " << p.x() << ", " << p.y() << ", " << p.z()<< std::endl;
}
void print(geometry_msgs::PoseStamped p)
{
    double x = p.pose.position.x;
    double y = p.pose.position.y;
    double z = p.pose.position.z;

    std::cout << "Drone: " << x << ", " << y << ", " << z << std::endl; 
}

bool goal_reached(geometry_msgs::PoseStamped goal)
{
    boost::shared_ptr<geometry_msgs::PoseStamped const> msg =  ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/mavros/local_position/pose");
    geo_pose position =  *(msg);
    double d = distance(position, goal);
    if (d < 0.5)
    {
        return true;
    }
    else
    {
        std::cout << "position" << position.pose.position.x << position.pose.position.y << position.pose.position.z << std::endl;
        std::cout << "distance " << d << std::endl;
        return false;
    }
}

double distance(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b)
{
    // return eculidan distance
    double x = a.pose.position.x - b.pose.position.x;
    double y = a.pose.position.y - b.pose.position.y;
    double z = a.pose.position.z - b.pose.position.z;

    double dist = pow(x, 2) + pow(y, 2) + pow(z, 2);
    dist = sqrt(dist);

    return dist;
}

double distance(octomap::point3d a, geometry_msgs::PoseStamped b)
{
    // return eculidan distance
    double x = a.x() - b.pose.position.x;
    double y = a.y() - b.pose.position.y;
    double z = a.z() - b.pose.position.z;

    double dist = pow(x, 2) + pow(y, 2) + pow(z, 2);
    dist = sqrt(dist);

    return dist;
}

geometry_msgs::PoseStamped publish_point(octomap::point3d p)
{
    /*convert the point3d to posestamped and 
    check if it is valid
    then publish it
    */
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
        std::cout << "Point" << p.x() << "," << p.y() << "," << p.z() << " added"<< std::endl;
    }
    waypoints_pub.publish(mypath);
    std::cout << "Path Published" << std::endl;
    return waypoints[0];
}

octomap::point3d setOctomapFromBinaryMsg(const octomap_msgs::Octomap& msg) 
{
    ourmap = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(msg));
    const octomap::point3d min_point(0.1, 0.1, 0.1);
    octomap::point3d max_point(2, 2, 2);
    //ourmap->setBBXMax(max_point);
    //ourmap->bbxSet();
    /*
    octomap::point3d box_limits(3, 3, 3);
    ourmap->setBBXMax(box_limits);  // the bounding box is half the value, expected
    octomap::point3d bounds = ourmap->getBBXBounds();
    print(bounds);
    */
    //loop_nodes(ourmap);
    


    double dist = 0; //the maximum variable
    octomap::point3d farpoint;  //maximum point

    // those lines are repeated, subscribe to local_position/pose
    boost::shared_ptr<geometry_msgs::PoseStamped const> wwe =  ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/mavros/local_position/pose");
    geo_pose position = *(wwe);
    
    //TODO the leaf don't have to be the farest point, I should use bound boxes
    print(position);
    loop_leafs(ourmap);
    return max_point;
    for (octomap::OcTree::leaf_bbx_iterator it = ourmap->begin_leafs_bbx(min_point, max_point);
        it != ourmap->end_leafs_bbx(); ++it)
       {
           print(it.getCoordinate());
           if(!ourmap->isNodeOccupied(*it))
           {
                octomap::point3d freepoint = it.getCoordinate();
                if (distance(freepoint, position) > dist)
                {
                    dist = distance(freepoint, position);
                    farpoint = freepoint;
                }
           }
       }
    print(farpoint, "Far Point");
    return farpoint;
    //std::cout<<"End of tree" << "\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.2);
    
    waypoints_pub = n.advertise<nav_msgs::Path>("/planner/waypoints", 1000); // publisher

    //create a client
    ros::ServiceClient client = n.serviceClient<octomap_msgs::GetOctomap>("octomap_binary");
    octomap_msgs::GetOctomap srv;

    // The main routine
    while (ros::ok())
    {
        std::cout << "Call the service" << std::endl;
        ros::service::waitForService("octomap_binary"); //wait for the service to be available 
        bool succeded = client.call(srv);
        if (succeded)
        {
            octomap::point3d g = setOctomapFromBinaryMsg(srv.response.map);
            ros::shutdown();
        }
    }
    

    return 0;
}