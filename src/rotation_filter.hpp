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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <rotation_filter_/rtf.h>


class rotation_filter_{
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

    private:
    ros::NodeHandle n;
    ros::Publisher points_filtered;
    ros::Publisher points_filtered_dbg;
    ros::Subscriber sub_pcl;
    ros::Publisher vis_pub;
    ros::ServiceServer rtf_sv;
    // Parameters used in pcl::passthrough
    float passthrough_x_p, passthrough_y_p, passthrough_z_p;
    float passthrough_x_n, passthrough_y_n, passthrough_z_n;
    // User can adjust the rotation of the filter itself
    double rotation_x, rotation_y, rotation_z;
    double centroid_x, centroid_y, centroid_z;

    visualization_msgs::Marker marker_for_drawing_rviz; 

    PointCloud pointcloud_in;
    PointCloud pointcloud_filtered;
    
    pcl::PointIndices PointIndices_x;
    pcl::PointIndices PointIndices_y;
    pcl::PointIndices PointIndices_z;
    void callback(const sensor_msgs::PointCloud2Ptr& cloud);
    bool rtf_main_callback(rotation_filter_::rtf::Request& req, rotation_filter_::rtf::Response& res);
    
    dynamic_reconfigure::Server<rotation_filter::rotationConfig> server;
    dynamic_reconfigure::Server<rotation_filter::rotationConfig>::CallbackType f;
    void callback_d(rotation_filter::rotationConfig &config, uint32_t level);
    void store_points_boundaries_rviz();
    void points_corner_computes();
    geometry_msgs::Point point;
    geometry_msgs::Point point_centroid;
    std::vector<geometry_msgs::Point> points_for_corner;
    std_msgs::ColorRGBA color;

      enum RTF_TASK {
      /*2020/8/8*/SET_TF_TASK  = 1,
      /*2020/8/8*/SET_OFFSET_TASK = 2,
      /*2020/8/8*/SET_TF_OFFSET_TASK = 3,
      };

    std::string rviz_frame;

    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::Transform transform;
    bool broadcaster_set = false;
    void set_broadcaster(std::string tf_target_ref, std::string tf_target, std::string tf_new, double offest_x, doulbe offset_y, double offset_z);
    double R;
    double P;
    double Y;
    std::string tf_target_frame;
    std::string tf_new_frame_wrt_target_frame;
    tf::Quaternion new_q;


    public:
    rotation_filter_();
    void spin_();
    typedef boost::function<bool (rotation_filter_::rtf::Request& req, rotation_filter_::rtf::Response& res)> rtf_t;
};

bool rotation_filter_::rtf_main_callback(rotation_filter_::rtf::Request& req, rotation_filter_::rtf::Response& res)
{  
      ROS_INFO_STREAM("[RTF] A service was called ");
      res.module_ = "RTF";
      int error_ = 0;
        switch(req.task_id)
        {
         case RTF_TASK::SET_TF_TASK: {
         ROS_INFO_STREAM("RTF_TASK::SET_TF_TASK");
         set_broadcaster(req.tf_ref, req.tf_target, req.tf_new, 0, 0, 0);
         break;
       }
         case RTF_TASK::SET_OFFSET_TASK: {
         ROS_INFO_STREAM("RTF_TASK::SET_OFFSET_TASK");
         transform.setOrigin( tf::Vector3(offest_x, offset_y, offset_z) );
         break;
       }
        case RTF_TASK::SET_TF_OFFSET_TASK: {
         ROS_INFO_STREAM("RTF_TASK::SET_TF_OFFSET_TASK");
         set_broadcaster(req.tf_ref, req.tf_target, req.tf_new, offset_x, offset_y, offset_z);
         break;
       }
       
       default:
       error_ = 2;
       break;
        }//switch

       res.error_ = error_;
       return true;
 }

rotation_filter_::rotation_filter_()
  {
    centroid_x = 0.0;
    centroid_y = 0.0;
    centroid_z = 0.0;
    passthrough_x_p = 0.5;
    passthrough_y_p = 0.5;
    passthrough_z_p = 0.5;
    passthrough_x_n = -0.5;
    passthrough_y_n = -0.5;
    passthrough_z_n = -0.5;
    rotation_x = 0.0;
    rotation_y = 0.0;
    rotation_z = 0.0;
    point_centroid.x = 0.0;
    point_centroid.y = 0.0;
    point_centroid.z = 0.0;
    
    points_filtered = n.advertise<PointCloud> ("points",1);
    points_filtered_dbg = n.advertise<PointCloud> ("points_dbg",1);
    vis_pub =  n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    sub_pcl = n.subscribe("/camera/depth_registered/points", 1, &rotation_filter_::callback, this);
    f = boost::bind(&rotation_filter_::callback_d,this, _1, _2);
    server.setCallback(f);
    
    rviz_frame = "world";

    // http://www.menucool.com/rgba-color-picker
    color.a = 1.0;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    points_for_corner.reserve(16);
    points_corner_computes();
    store_points_boundaries_rviz();

    rtf_t rtf_main_callback = boost::bind(&rotation_filter_::pcp_main_callback, this, _1, _2);
    rtf_sv = n.advertiseService("/rtf_client", rtf_main_callback);
     
  }

void rotation_filter_::callback(const sensor_msgs::PointCloud2Ptr& cloud) {
    pcl::fromROSMsg(*cloud, pointcloud_in);  // copy sensor_msg::Pointcloud message into pcl::PointCloud
  }

void rotation_filter_::callback_d(rotation_filter::rotationConfig &config, uint32_t level) {
   ROS_INFO("Reconfigure Request: %f %f %f - %f %f %f - %f %f %f",
              config.passthrough_x_p, config.passthrough_y_p,config.passthrough_z_p,
              config.passthrough_x_n, config.passthrough_y_n,config.passthrough_z_n,
              config.rotation_x, config.rotation_y,config.rotation_z);
            
              rotation_x = config.rotation_x * M_PI/180.0;
              rotation_y = config.rotation_y * M_PI/180.0;
              rotation_z = config.rotation_z * M_PI/180.0;

              R = R + rotation_x;
              P = P + rotation_y;
              Y = Y + rotation_z;
              new_q.setRPY(R,P,Y);
              transform.setRotation(new_q);
              
              passthrough_x_p = config.passthrough_x_p;
              passthrough_y_p = config.passthrough_y_p;
              passthrough_z_p = config.passthrough_z_p;
              passthrough_x_n = config.passthrough_x_n;
              passthrough_y_n = config.passthrough_y_n;
              passthrough_z_n = config.passthrough_z_n;

              points_corner_computes();
              store_points_boundaries_rviz();
  }

void rotation_filter_::spin_()
{
    ros::Rate loop_rate(2);
    int seq = 0;
    while (ros::ok())
    {
      if(broadcaster_set) br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tf_target_frame, tf_new_frame_wrt_target_frame));

      /*
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
/*
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
      */
      ros::spinOnce();
      loop_rate.sleep();
    }
}// end spin()

void rotation_filter_::points_corner_computes()
{
    // 1 
    points_for_corner.clear();
    point.x = point_centroid.x + passthrough_x_p;
    point.y = point_centroid.y + passthrough_y_p;
    point.z = point_centroid.z + passthrough_z_p;
    points_for_corner.push_back(point);
    // 2 
    point.x = point_centroid.x + passthrough_x_n;
    point.y = point_centroid.y + passthrough_y_p;
    point.z = point_centroid.z + passthrough_z_p;
    points_for_corner.push_back(point);
    // 3
    point.x = point_centroid.x + passthrough_x_n;
    point.y = point_centroid.y + passthrough_y_n;
    point.z = point_centroid.z + passthrough_z_p;
    points_for_corner.push_back(point);
    // 4
    point.x = point_centroid.x + passthrough_x_p;
    point.y = point_centroid.y + passthrough_y_n;
    point.z = point_centroid.z + passthrough_z_p;
    points_for_corner.push_back(point);
    // 5
    point.x = point_centroid.x + passthrough_x_p;
    point.y = point_centroid.y + passthrough_y_p;
    point.z = point_centroid.z + passthrough_z_n;
    points_for_corner.push_back(point);
    // 6
    point.x = point_centroid.x + passthrough_x_n;
    point.y = point_centroid.y + passthrough_y_p;
    point.z = point_centroid.z + passthrough_z_n;
    points_for_corner.push_back(point);
    // 7
    point.x = point_centroid.x + passthrough_x_n;
    point.y = point_centroid.y + passthrough_y_n;
    point.z = point_centroid.z + passthrough_z_n;
    points_for_corner.push_back(point);
    // 8
    point.x = point_centroid.x + passthrough_x_p;
    point.y = point_centroid.y + passthrough_y_n;
    point.z = point_centroid.z + passthrough_z_n;
    points_for_corner.push_back(point);

}

void rotation_filter_::store_points_boundaries_rviz()
{
    marker_for_drawing_rviz.points.clear();
    marker_for_drawing_rviz.colors.clear();
    marker_for_drawing_rviz.header.frame_id = rviz_frame;
    marker_for_drawing_rviz.header.stamp = ros::Time();
    marker_for_drawing_rviz.scale.x = 0.01;
    //marker_for_drawing_rviz.scale.y = 0.1;
    //marker_for_drawing_rviz.scale.z = 0.1;
    marker_for_drawing_rviz.action = visualization_msgs::Marker::ADD;
    marker_for_drawing_rviz.type = visualization_msgs::Marker::LINE_STRIP;
    marker_for_drawing_rviz.points.push_back(points_for_corner[0]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[1]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[2]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[3]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[0]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[4]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[7]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[3]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[7]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[6]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[2]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[6]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[5]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[1]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[5]);
    marker_for_drawing_rviz.points.push_back(points_for_corner[4]);
    
    for(int i = 0; i < marker_for_drawing_rviz.points.size(); i++) marker_for_drawing_rviz.colors.push_back(color); 
    vis_pub.publish(marker_for_drawing_rviz);
    ros::spinOnce();
}

void rotation_filter_::set_broadcaster(std::string tf_target_ref, std::string tf_target, std::string tf_new, double offest_x, doulbe offset_y, double offset_z)
{
  // first listen to the tf_ref 
  tf::StampedTransform stamped_transform;
  bool No_info; 
  while(No_info)
  { No_info = false;
    try{
      listener.lookupTransform(tf_target_ref, tf_target,  
                               ros::Time(0), stamped_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      No_info = true;
    }
  }

  // second, use it orientation to publish
  transform.setOrigin( tf::Vector3(offest_x, offset_y, offset_z) );
  transform.setRotation(stamped_transform.getRotation());
  stamped_transform.getBasis().getRPY(R,P,Y);
  broadcaster_set = true;
  tf_target_frame = tf_target ; 
  tf_new_frame_wrt_target_frame = tf_new; 
}