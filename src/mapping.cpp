#include <mapping.h>

static octomap::OcTree* ourmap;

void setOctomapFromBinaryMsg(const octomap_msgs::Octomap& msg) {
    ourmap = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(msg));
    
    //iterate over the map
    for (octomap::OcTree::leaf_iterator it = ourmap->begin_leafs();
       it != ourmap->end_leafs(); ++it) {
           std::cout<<"found a leaf";
       }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/octomap_binary", 1000, setOctomapFromBinaryMsg);
    ros::spin();

    return 0;
}