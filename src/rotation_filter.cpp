/* rotation filter is a ros node that allows user to filter the point cloud. The
filtering technique is similar to the passthrough filter provided by the pcl. However,
this node allows user to rotate th point cloud and then apply the filtering.

How to use or run this node?
1. adjust the parameter of "Passthtough filter" and "rotation" through dynamic_reconfigure
2. View on the rviz for the result

*/
#include "ros/ros.h"
#include "rotation_filter.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotation_filter");
  rotation_filter_().spin_();
  return 0;
}
