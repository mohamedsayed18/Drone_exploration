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
    
    //iterate over leaf nodes
    /*
    for (octomap::OcTree::leaf_iterator it = ourmap->begin_leafs();
       it != ourmap->end_leafs(); ++it) {
           //std::cout<< "Node center: " << it.getCoordinate();
           //std::cout<< " value: " << it->getValue()<< "\n";
           //std::cout<< "size of node " << it.getSize() << "\n";
           //std::cout<< ourmap->isNodeOccupied(*it) <<"\n";
           std::cout<< (ourmap->getRoot()).getSize();
           break;
       }
    */
    //iterate over all nodes
    octomap::OcTreeNode * node = ourmap->getRoot();

    for (octomap::OcTree::tree_iterator it = ourmap->begin_tree(); 
        it != ourmap->end_tree(); ++it)
    {
        std::cout<< it.getSize() <<;
    }
    
    std::cout<<"End of tree" << "\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/octomap_binary", 1000, setOctomapFromBinaryMsg);
    ros::spin();

    return 0;
}