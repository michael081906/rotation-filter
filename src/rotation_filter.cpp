/* rotation filter is a ros node that allows user to filter the point cloud. The
filtering technique is similar to the passthrough filter provided by the pcl. However,
this node allows user to rotate th point cloud and then apply the filtering.

How to use or run this node?
1. adjust the parameter of "Passthtough filter" and "rotation" through dynamic_reconfigure
2. View on the rviz for the

*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <boost/make_shared.hpp>
#include <std_msgs/Bool.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h> // used for "pcl::fromROSMsg"
#include <dynamic_reconfigure/server.h>
#include <rotation_filter/rotationConfig.h>


class rotation_filter_{
private:
  ros::NodeHandle n;
  ros::Publisher points_filtered;
  ros::Subscriber sub_pcl;
  double passthrough_x, passthrough_y, passthrough_z;
  double rotation_x, rotation_y, rotation_z;
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  PointCloud pointcloud_in;
  PointCloud pointcloud_filtered;

  void callback(const sensor_msgs::PointCloud2Ptr& cloud) {
    pcl::fromROSMsg(*cloud, pointcloud_in);  // copy sensor_msg::Pointcloud message into pcl::PointCloud
  }

  dynamic_reconfigure::Server<rotation_filter::rotationConfig> server;
  dynamic_reconfigure::Server<rotation_filter::rotationConfig>::CallbackType f;
  void callback_d(rotation_filter::rotationConfig &config, uint32_t level) {
   ROS_INFO("Reconfigure Request: %f %f %f - %f %f %f",
              config.passthrough_x, config.passthrough_y,config.passthrough_z,
              config.rotation_x, config.rotation_y,config.rotation_z);
              rotation_x = config.rotation_x;
              rotation_y = config.rotation_y;
              rotation_z = config.rotation_z;
              passthrough_x = config.passthrough_x;
              passthrough_y = config.passthrough_y;
              passthrough_z = config.passthrough_z;
  }

public:


  rotation_filter_()
  {
    passthrough_x = 0.0;
    passthrough_y = 0.0;
    passthrough_z = 0.0;
    rotation_x = 0.0;
    rotation_y = 0.0;
    rotation_z = 0.0;
    points_filtered = n.advertise<PointCloud> ("points",1);
    sub_pcl = n.subscribe("/see_scope/overlay/cog", 1, &rotation_filter_::callback, this);
    f = boost::bind(&rotation_filter_::callback_d,this, _1, _2);
    server.setCallback(f);
  }
  ~rotation_filter_()
  {}

  void spin_()
  {
    ros::Rate loop_rate(0.5);
    int seq = 0;
    while (ros::ok())
    {
      // rotating the point cloud 


      // passthrough filtering




      // remove the indices






      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.seq = seq++; // is this correct
      header.frame_id = std::string("mono");
      pointcloud_filtered.header = pcl_conversions::toPCL(header);
      points_filtered.publish(pointcloud_filtered);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }// end spin()


};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotation_filter");
  rotation_filter_().spin_();
  return 0;
}
