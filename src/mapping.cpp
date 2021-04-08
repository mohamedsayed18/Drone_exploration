/*
* Know the size of the the map
* and if the node is occupied or not
* getSize() the size of each leaf node is 0.2, which is equal to resolution
* map->isNodeOccupied(*it)check if the node is occupied or not
* using get size on the first node we get the dimensions of the map
* begin_leafs_bbx(min_point, max_point), this iterate leafs in a certain bound
* can I get a leaf not at the last depth
*/

/* TODO
Merge the step of finding frontier and clustering
Learn how to do debug for ROS on VS code
find neighbours using keys as follows:
octomap::OcTreeKey k = ourmap->coordToKey(*i);
std::cout << k[0] << " " << k[1]<< " " << k[2]<< std::endl;
*/
#include <mapping.h>
#include <explorer.h>

// Global variables, TODO: make a clean structure
static octomap::OcTree* ourmap; // octree map
ros::Publisher waypoints_pub;   //publisher

bool operator== (const octomath::Vector3 v1, const octomath::Vector3 v2)
{
    return (v1.x()==v2.x() &&  v1.y()==v2.y() && v1.y()==v2.y());
}

octomap::point3d_list check_neighbours(octomap::point3d p)
{
    /*
    TODO edit and use neighborkey = key;
    neighborkey[0] +=1; // neighbor in pos. x-direction
    OcTreeNode* result = octree.search(neighborkey);
    */ 
    std::cout << "check neighbours" << std::endl;
    double octomap_res = 0.2;
    octomap::point3d_list l;

    for (double i = p.x()-octomap_res; i <= p.x()+octomap_res; i+=octomap_res)
    {
        for(double j = p.y()-octomap_res; j <= p.y()+octomap_res; j+=octomap_res)
        {
            if(i==p.x() && j== p.y()) continue;
            l.push_back(octomap::point3d(i, j, p.z()));
        }
    }
    std::cout << "check neighbours_done" << std::endl;
    return l;
    
}

octomap::point3d_list get_frontiers(octomap::OcTree map)
{
    octomap::point3d_list frontiers;

    for (octomap::OcTree::leaf_iterator it = map.begin_leafs();
    it != map.end_leafs(); ++it) 
    {   
        if(!map.isNodeOccupied(*it))   //check if the node is not occupied
        {
            octomap::point3d freepoint = it.getCoordinate();
            octomap::point3d_list neighbours = check_neighbours(freepoint);
            print(freepoint, "Free point ");
            std::cout << "Number of neighbours " << neighbours.size() << std::endl;

            int Nu_unknowns = 0; 
            for(auto i=neighbours.begin(); i != neighbours.end(); i++)
            {
                if(!map.search(*i))  // if node is not found in the tree
                {
                    Nu_unknowns++;
                }
            }
            if(Nu_unknowns>0)   //Nu_u
            {
                frontiers.push_back(freepoint);
            }
        }
    }
    return frontiers;
}

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

void publish_point(geometry_msgs::PoseStamped p)
{
    nav_msgs::Path mypath;
    mypath.poses.push_back(p);
    waypoints_pub.publish(mypath);
}
void rotate()
{
    double start_angle = 45;    //increment angles
    double inc_angle = 45;  //increment angles
    geometry_msgs::PoseStamped waypoints[8];
    
    //waypoint.header.frame_id = "Rotate";

    tf2::Quaternion angle;

    for (int i = 0; i < 8; i++)
    {
        double rad_angle = angles::from_degrees(start_angle);
        angle.setRPY(0, 0, rad_angle);
        geometry_msgs::Quaternion myangle = tf2::toMsg(angle);
        waypoints[i].pose.orientation = myangle;
        start_angle += inc_angle;
    }
    

}

std::list<std::vector<octomap::point3d>> make_clusters(octomap::point3d_list frontiers)
{
    std::list<std::vector<octomap::point3d>> cluster;
    
    int nu_clusters = 0;

    while (!frontiers.empty())
    {
        
        nu_clusters++;
        octomap::point3d f = frontiers.front(); //get the first element
        frontiers.pop_front();  //remove the element from the list
        std::vector<octomap::point3d> q;
        std::vector<octomap::point3d> group;
        q.push_back(f);  //add it to the queue;
        group.push_back(f); //add it to the group;
        while (!q.empty())
        {
            auto point = q.front();
            q.pop_back();
            octomap::point3d_list neighbours = check_neighbours(point); //get its neighbours
            for(auto i=neighbours.begin(); i != neighbours.end(); i++)
            {
                auto it = std::find(frontiers.begin(), frontiers.end(), *i);
                if(it != frontiers.end())
                {
                    q.push_back(*it);
                    group.push_back(*it);
                    frontiers.erase(it);
                }
            }
        }
        cluster.push_back(group);
    }
    return cluster;
    
}


octomap::point3d setOctomapFromBinaryMsg(const octomap_msgs::Octomap& msg) 
{
    // Local points
    const octomap::point3d min_point(-10, -10, 0);
    octomap::point3d max_point(10, 10, 10);
    octomap::point3d_list fron;

    ourmap = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(msg));

    // Get the frontriers
    fron = get_frontiers(*ourmap);
    std::cout << "Number of frontiers" << fron.size() << std::endl;

    std::list<std::vector<octomap::point3d>> clusters = make_clusters(fron);
    // Loop the clusters
    std::cout << "number of clusters" << clusters.size() << std::endl;
    int clu_size = 0;
    for(auto it=clusters.begin(); it != clusters.end(); it++)
    {
        std::sort(it->begin(), it->end());
        //std::cout << "Vector size" << it->size() << std::endl;
        clu_size+= it->size();
        int mid = it->size() / 2; //maybe a problem because of division
        print(it->at(mid), "Centre of Cluster");
    }
    std::cout <<"Number of nodes in all clusters " << clu_size << std::endl;
    /*
    for(auto i=fron.begin(); i != fron.end(); i++)
    {
        print(*i, "Frontier");
        //octomap::OcTreeKey k = ourmap->coordToKey(*i);
        //std::cout << k[0] << " " << k[1]<< " " << k[2]<< std::endl;
        //std::cout << k.k << std::endl;
        //octomap::OcTreeNode* n = ourmap->search(*i);

    }
    */
    return *(fron.begin());
    //ourmap->setBBXMax(max_point);
    //ourmap->bbxSet();
    /*
    octomap::point3d box_limits(3, 3, 3);
    ourmap->setBBXMax(box_limits);  // the bounding box is half the value, expected
    octomap::point3d bounds = ourmap->getBBXBounds();
    print(bounds);
    */
    //loop_nodes(ourmap);
    // octree methods
    std::cout << ourmap->getNumLeafNodes() << std::endl;
    /*
    ourmap->getUnknownLeafCenters(l, min_point, max_point);
    for (auto i = l.begin(); i != l.end(); i++)
    {
        print(*i, "Uknown");
    }
    */

    // create a new octree
    octomap::OcTree mytree(0.2);
    octomap::OcTree* global_tree = dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::createTree("Global", 0.2));
    std::cout << ourmap->getNumLeafNodes() << std::endl;

    


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