/*
* Know the size of the the map
* and if the node is occupied or not
* getSize() the size of each node is 0.2, which is equal to resolution
* map->isNodeOccupied(*it)check if the node is occupied or not
* using get size on the first node we get the dimensions of the map
*/
#include <mapping.h>

static octomap::OcTree* ourmap;

void setOctomapFromBinaryMsg(const octomap_msgs::Octomap& msg) {
    ourmap = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(msg));
    /*
    double maxX, maxY, maxZ;
    ourmap->getMetricSize(maxX, maxY, maxZ);   
    octomap::point3d maxBb = octomap::point3d(maxX, maxY, maxZ);
    
    std::cout << "metric max " << maxBb.x() << "," << maxBb.y() << "," << maxBb.z() << std::endl;
    */

    // metric max 0.8,1,1.6
    //iterate over leaf nodes
    
    float dist = 0;
    octomap::point3d farpoint;
    for (octomap::OcTree::leaf_iterator it = ourmap->begin_leafs();
        it != ourmap->end_leafs(); ++it) 
       {
           if(!ourmap->isNodeOccupied(*it))
           {
                octomap::point3d freepoint = it.getCoordinate();
                //std::cout << "Point: " << freepoint.x() << "," << freepoint.y() << "," << freepoint.z() << std::endl;
                //get the farest free point
                if (abs((freepoint.x()*freepoint.x()) + (freepoint.y()*freepoint.y()) 
                    + (freepoint.z()*freepoint.z())) > dist)
                {
                    dist = abs((freepoint.x()*freepoint.x()) + (freepoint.y()*freepoint.y()) 
                        + (freepoint.z()*freepoint.z()));
                    farpoint = freepoint;
                }
           }
           //std::cout<< "Node center: " << it.getCoordinate();
           //std::cout<< " value: " << it->getValue()<< "\n";
           //std::cout<< "size of node " << it.getSize() << "\n";
           //std::cout<< ourmap->isNodeOccupied(*it) <<"\n";
           //std::cout<< (ourmap->getRoot()).getSize();
       }
    std::cout << "Point: " << farpoint.x() << "," << farpoint.y() << "," << farpoint.z() << std::endl;
    std::cout<<"End of tree" << "\n";
    //iterate over all nodes
    //octomap::OcTreeNode * node = ourmap->getRoot();
    /*
    std::cout<< "the size of the map" << ourmap->size() << std::endl;
    for (octomap::OcTree::tree_iterator it = ourmap->begin_tree(); 
        it != ourmap->end_tree(); ++it)
    {
        std::cout<< it->getValue() <<std::endl;
    }
    
    std::cout<<"End of tree" << "\n";
    */
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/octomap_binary", 1000, setOctomapFromBinaryMsg);
    ros::spin();

    return 0;
}