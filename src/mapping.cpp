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

change return of make cluster to vector<vector>>

check shared pointers, to avoid repetation

//TODO
Consider leaf nodes of size higher than resolution size: expandNode() and account its leafs
check bbxSet to check the size
*/
#include <mapping.h>
#include <explorer.h>

// Global variables, TODO: make a clean structure
static octomap::OcTree* ourmap; // octree map
ros::Publisher waypoints_pub;   //publisher
ros::Publisher myrotate, trigger_pub;   //publisher
void reg_trigger()
{
    /*
    TODO I think it will be a better idea to make this function set a global variable there
    */
    //trigger_rotate();// stop the fsm trajectory
    std::cout << "reg_trigger" << std::endl;
    boost::shared_ptr<geometry_msgs::PoseStamped const> now_pose =  ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/mavros/local_position/pose");
    geo_pose cur_pos = *(now_pose);
    std::cout << "Pose got from mavros" << std::endl;

    hagen_msgs::PoseCommand r;

    r.header.frame_id = "rotate";
/*
    r.position.x = cur_pos.pose.position.x;
    r.position.y = cur_pos.pose.position.y;
    r.position.z = 2;

    r.position.x = 3;
    r.position.y = 3;
    r.position.z = 3;
*/
    trigger_pub.publish(r);
}

bool operator== (const octomath::Vector3 v1, const octomath::Vector3 v2)
{
    return (v1.x()==v2.x() &&  v1.y()==v2.y() && v1.y()==v2.y());
}

octomap::point3d_list check_neighbours(octomap::point3d p, octomap::OcTree map)
{
    /*
    TODO edit and use neighborkey = key;
    neighborkey[0] +=1; // neighbor in pos. x-direction
    OcTreeNode* result = octree.search(neighborkey);
    rename to get_neighbours
    */ 
    //std::cout << "check neighbours" << std::endl;
    double octomap_res = 0.2;
    int nu_unknowns = 0;
    octomap::point3d_list l;    //list of the neighbours
    octomap::OcTreeKey neighborkey = map.coordToKey(p);

    for(int i=-1; i<=1; i++)
    {
        for(int j=-1; j<=1; j++)
        {
            for(int k=-1; k<=1; k++)
            {
                if(i==j==k==0)    //skip the node it self
                    continue;
                octomap::OcTreeKey n = neighborkey;
                n[0]+=i; n[1]+=j; n[2]+=k;
                l.push_back( map.keyToCoord(n));
                if(!map.search(n))
                {
                    nu_unknowns++;
                }
            }
        }   
    }
    return l;
    //return nu_unknowns;
    
}

octomap::point3d_list get_frontiers(octomap::OcTree map)
{
    // The limits to search for in the map
    octomap::point3d min_point(-10, -10, -10);
    octomap::point3d max_point(10, 10, 10);

    octomap::point3d_list frontiers;    //list of frontiers

    for (octomap::OcTree::leaf_bbx_iterator it = map.begin_leafs_bbx(min_point, max_point);
    it != map.end_leafs_bbx(); ++it) 
    {   
        if(!map.isNodeOccupied(*it))   //check if the node is free
        {
            octomap::point3d freepoint = it.getCoordinate();
            print(freepoint, "Free point ");
            std::cout << "size " << it.getSize() << std::endl;
            
            if(it.getSize() == 0.2) // TODO, Handle other sizes
            {
                octomap::point3d_list neighbours = check_neighbours(freepoint, map);
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
    }
    std::cout << "frontiers done " << frontiers.size() << std::endl;
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
void print(geometry_msgs::PoseStamped p, std::string title)
{
    double x = p.pose.position.x;
    double y = p.pose.position.y;
    double z = p.pose.position.z;

    std::cout << title << " " << x << ", " << y << ", " << z << std::endl;
    //std::cout << "Drone: " << x << ", " << y << ", " << z << std::endl; 
}

bool goal_reached(octomap::point3d goal)
{
    //TODO shared pointer repetation
    boost::shared_ptr<nav_msgs::Odometry const> msg =  ros::topic::waitForMessage<nav_msgs::Odometry>("/planning/current_state");
    nav_msgs::Odometry position =  *(msg);
    
    double d = distance(position, goal);
    
    if (d < 2)
    {
        return true;
    }
    else
    {
        //std::cout << "position" << position.pose.position.x << position.pose.position.y << position.pose.position.z << std::endl;
        std::cout << "distance " << d << std::endl;
        return false;
    }
}

double distance(nav_msgs::Odometry a, octomap::point3d b)
{
    // return eculidan distance
    double x = a.pose.pose.position.x - b.x();
    double y = a.pose.pose.position.y - b.y();
    double z = a.pose.pose.position.z - b.z();

    double dist = pow(x, 2) + pow(y, 2) + pow(z, 2);
    dist = sqrt(dist);

    return dist;
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
    geometry_msgs::PoseStamped waypoints[1];
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
        print(waypoints[i], "Point Added");
    }
    waypoints_pub.publish(mypath);
    std::cout << "Path Published" << std::endl;
    return waypoints[0];
}

/*
void publish_point(geometry_msgs::PoseStamped p)
{
    nav_msgs::Path mypath;
    mypath.poses.push_back(p);
    waypoints_pub.publish(mypath);
}
*/
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

std::list<std::vector<octomap::point3d>> make_clusters(octomap::point3d_list frontiers, octomap::OcTree map)
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
            octomap::point3d_list neighbours = check_neighbours(point, map); //get its neighbours
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

std::vector<octomap::point3d> get_candidates(std::list<std::vector<octomap::point3d>> clusters)
{
    /*
    TODO: Maybe we do Mean or different way instead of centre 
    input: list of clusters
    return: centre of each cluster 
    */
    std::cout << "number of clusters " << clusters.size() << std::endl;
    std::vector<octomap::point3d> centers;
    int clu_size = 0;
    for(auto it=clusters.begin(); it != clusters.end(); it++)
    {
        std::sort(it->begin(), it->end());
        //std::cout << "Vector size" << it->size() << std::endl;
        clu_size+= it->size();
        int mid = it->size() / 2; //maybe a problem because of division
        print(it->at(mid), "Centre of Cluster");
        centers.push_back(it->at(mid));
    }
    std::cout <<"Number of nodes in all clusters " << clu_size << std::endl;
    return centers;
}

octomap::point3d get_goal(std::vector<octomap::point3d> candidates, nav_msgs::Odometry position)
{
    /*Here we should assign the criteria and give cost of every candidate
    some suggested criteria:
    distance between the drone and this candidate
    Distance To edge of the exploration area(expected area to explore when reach this point)
    speed and angle desired by the drone to reach it
    TODO we can use BFS or any other algorthim to find the closest, instead of eculidean distance..
    we can also integrate it with path planner so we don't calculate the path twice
    */
    double min_distance = 0; //the maximum variable
    octomap::point3d closest_point;  //maximum point

   for(auto it = candidates.begin(); it != candidates.end(); it++)
   {
       if(distance(position, *it) > min_distance)
       {
           min_distance = distance(position, *it);
           closest_point = *it;
       }
   }
   return closest_point;
}

octomap::point3d get_goal(std::vector<octomap::point3d> candidates, geo_pose position)
{
    /*Here we should assign the criteria and give cost of every candidate
    some suggested criteria:
    distance between the drone and this candidate
    Distance To edge of the exploration area(expected area to explore when reach this point)
    speed and angle desired by the drone to reach it
    TODO we can use BFS or any other algorthim to find the closest, instead of eculidean distance..
    we can also integrate it with path planner so we don't calculate the path twice
    */
    double min_distance = 0; //the maximum variable
    octomap::point3d closest_point;  //maximum point

   for(auto it = candidates.begin(); it != candidates.end(); it++)
   {
       if(distance(*it, position) > min_distance)
       {
           min_distance = distance(*it, position);
           closest_point = *it;
       }
   }
   return closest_point;
}

octomap::OcTree setOctomapFromBinaryMsg(const octomap_msgs::Octomap& msg) 
{
    ourmap = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(msg));
    return *ourmap;
}

int main(int argc, char** argv)
{
    bool first_time = true;
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.2);
    
    waypoints_pub = n.advertise<nav_msgs::Path>("/planner/waypoints", 1000); // publisher
    myrotate = n.advertise<nav_msgs::Path>("/planner/waypoints", 1000);
    trigger_pub = n.advertise<hagen_msgs::PoseCommand>("/planning/pos_cmd", 1000);

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
            std::cout << "Octomap service found" << std::endl;
            octomap::OcTree mymap = setOctomapFromBinaryMsg(srv.response.map);
            if (waypoints_pub.getNumSubscribers()>0)
            {
                trigger_rotate();
                ros::topic::waitForMessage<std_msgs::Bool>("/check/rotation");
                octomap::point3d_list frontiers = get_frontiers(mymap);
                std::list<std::vector<octomap::point3d>> clusters = make_clusters(frontiers, mymap);
                std::vector<octomap::point3d> centers = get_candidates(clusters);
                std::cout << "**Got the candidates**" << std::endl;
                
                //Get the drone position, and the goal
                octomap::point3d goal;
                if(first_time)
                {
                    boost::shared_ptr<geometry_msgs::PoseStamped const> d_pose =  ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/mavros/local_position/pose");
                    geo_pose position = *(d_pose);
                    //first_time = false;   
                    //TODO seem like the other it wait a lot to subscribe 
                    // but it was working normally before when subscribe to the /planning/current_state
                    std::cout <<"First message got from current state" << std::endl;
                    goal = get_goal(centers, position);
                }
                else
                {
                    //TODO I can check if the topic exsit 
                    boost::shared_ptr<nav_msgs::Odometry const> p_pose =  ros::topic::waitForMessage<nav_msgs::Odometry>("/planning/current_state");
                    nav_msgs::Odometry position = *(p_pose);
                    std::cout <<"message got from current state" << std::endl;
                    goal = get_goal(centers, position);
                }
                print(goal, "The next goal");
                publish_point(goal);    //publish point 
                while (! goal_reached(goal))
                {
                    //loop_rate.sleep();
                    //std::cout << "Moving to the goal"<< std::endl;
                }
                std::cout << "goal reached"<<std::endl;

                // Do 360 rotation
                //trigger_rotate();
                //reg_trigger();
                //ros::topic::waitForMessage<std_msgs::Bool>("/check/rotation");
                //play();
                //ros::shutdown();
            }
            else
            {
                std::cout << "No subscribers" << std::endl;
            }
        }
    }

    return 0;
}