/* rotation filter is a ros node that allows user to filter the point cloud. The
filtering technique is similar to the passthrough filter provided by the pcl. However,
this node allows user to rotate th point cloud and then apply the filtering.

How to use or run this node?
1. adjust the parameter of "Passthtough filter" and "rotation" through dynamic_reconfigure
2. View on the rviz for the result

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
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> // used for "pcl::fromROSMsg"
#include <dynamic_reconfigure/server.h>
#include <rotation_filter/rotationConfig.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>


class rotation_filter_{
private:
  ros::NodeHandle n;
  ros::Publisher points_filtered;
  ros::Publisher points_filtered_dbg;
  ros::Subscriber sub_pcl;
  float passthrough_x_p, passthrough_y_p, passthrough_z_p;
  float passthrough_x_n, passthrough_y_n, passthrough_z_n;
  double rotation_x, rotation_y, rotation_z;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  PointCloud pointcloud_in;
  PointCloud pointcloud_filtered;
  pcl::PointIndices PointIndices_x;
  pcl::PointIndices PointIndices_y;
  pcl::PointIndices PointIndices_z;
  void callback(const sensor_msgs::PointCloud2Ptr& cloud) {
    pcl::fromROSMsg(*cloud, pointcloud_in);  // copy sensor_msg::Pointcloud message into pcl::PointCloud
  }

  dynamic_reconfigure::Server<rotation_filter::rotationConfig> server;
  dynamic_reconfigure::Server<rotation_filter::rotationConfig>::CallbackType f;
  void callback_d(rotation_filter::rotationConfig &config, uint32_t level) {
   ROS_INFO("Reconfigure Request: %f %f %f - %f %f %f - %f %f %f",
              config.passthrough_x_p, config.passthrough_y_p,config.passthrough_z_p,
              config.passthrough_x_n, config.passthrough_y_n,config.passthrough_z_n,
              config.rotation_x, config.rotation_y,config.rotation_z);
              rotation_x = config.rotation_x * M_PI/180.0;
              rotation_y = config.rotation_y * M_PI/180.0;
              rotation_z = config.rotation_z * M_PI/180.0;
              passthrough_x_p = config.passthrough_x_p;
              passthrough_y_p = config.passthrough_y_p;
              passthrough_z_p = config.passthrough_z_p;
              passthrough_x_n = config.passthrough_x_n;
              passthrough_y_n = config.passthrough_y_n;
              passthrough_z_n = config.passthrough_z_n;
  }

public:


  rotation_filter_()
  {
    passthrough_x_p = 0.01;
    passthrough_y_p = 0.01;
    passthrough_z_p = 0.01;
    passthrough_x_n = -0.01;
    passthrough_y_n = -0.01;
    passthrough_z_n = -0.01;
    rotation_x = 0.0;
    rotation_y = 0.0;
    rotation_z = 0.0;
    points_filtered = n.advertise<PointCloud> ("points",1);
    points_filtered_dbg = n.advertise<PointCloud> ("points_dbg",1);
    sub_pcl = n.subscribe("/camera/depth_registered/points", 1, &rotation_filter_::callback, this);
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
      Eigen::Affine3f transforms_point = Eigen::Affine3f::Identity();
      transforms_point.translation() << 0.0 ,0.0 ,0.0;
      transforms_point.rotate(Eigen::AngleAxisf (rotation_z, Eigen::Vector3f::UnitZ())* Eigen::AngleAxisf (rotation_y, Eigen::Vector3f::UnitY()) *
                                   Eigen::AngleAxisf (rotation_x, Eigen::Vector3f::UnitX()));

      // Executing the transformation
      PointCloud::Ptr source_cloud(new PointCloud(pointcloud_in));
      //source_cloud = pointcloud_in.makeShared();
      PointCloud::Ptr transformed_cloud (new PointCloud ());
      pcl::transformPointCloud (*source_cloud, *transformed_cloud, transforms_point);

      PointCloud::Ptr source_cloud_dbg(new PointCloud());


      // passthrough filtering
      /*
      In order to use the getRemovedIndices system you need to initialize your class differently:
      change this:
      pcl::PassThrough<pcl::PointXYZ> pass;
      into this:
      pcl::PassThrough<pcl::PointXYZ> pass (true);
      */

      pcl::PassThrough<pcl::PointXYZRGB> pass(true);
      pass.setInputCloud (transformed_cloud); // setinput needs ptr
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (passthrough_x_n, passthrough_x_p);
      pass.filter (*transformed_cloud);  // filter needs point cloud
      pass.getRemovedIndices(PointIndices_x);
      //std::cout << PointIndices_x << std::endl;
      pass.setInputCloud (transformed_cloud);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (passthrough_y_n, passthrough_y_p);
      pass.filter (*transformed_cloud);
      pass.getRemovedIndices(PointIndices_y);
      pass.setInputCloud (transformed_cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (passthrough_z_n, passthrough_z_p);
      pass.filter (*transformed_cloud);
      pass.getRemovedIndices(PointIndices_z);




      // remove the indices
      pcl::PointIndices::Ptr indices_x(new pcl::PointIndices(PointIndices_x));
      pcl::PointIndices::Ptr indices_y(new pcl::PointIndices(PointIndices_y));
      pcl::PointIndices::Ptr indices_z(new pcl::PointIndices(PointIndices_z));
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(source_cloud);
      extract.setIndices(indices_x);
      extract.setNegative(true);
      extract.filter(*source_cloud);
      extract.setInputCloud(source_cloud);
      extract.setIndices(indices_y);
      extract.setNegative(true);
      extract.filter(*source_cloud);
      extract.setInputCloud(source_cloud);
      extract.setIndices(indices_z);
      extract.setNegative(true);
      extract.filter(*source_cloud);

      //pointcloud_filtered = *source_cloud;
      //pointcloud_filtered = *transformed_cloud;

      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.seq = seq++; // is this correct
      header.frame_id = std::string("camera_color_optical_frame");
      source_cloud->header = pcl_conversions::toPCL(header);
      points_filtered.publish(*source_cloud);

      transformed_cloud->header = pcl_conversions::toPCL(header);
      points_filtered_dbg.publish(*transformed_cloud);
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
