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
#include <rotation_filter/rtf.h>

template <class T>
class rotation_filter_{
    typedef pcl::PointCloud<T> PointCloud;

    private:
    ros::NodeHandle n;
    ros::Publisher points_filtered_passthrough;
    ros::Publisher points_filtered_extract;
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
    PointCloud pointcloud_user_frame;
    PointCloud pointcloud_filtered;

    pcl::PointIndices PointIndices_x;
    pcl::PointIndices PointIndices_y;
    pcl::PointIndices PointIndices_z;
    void callback(const sensor_msgs::PointCloud2Ptr& cloud);
    bool rtf_main_callback(rotation_filter::rtf::Request& req, rotation_filter::rtf::Response& res);

    dynamic_reconfigure::Server<rotation_filter::rotationConfig> server;
    dynamic_reconfigure::Server<rotation_filter::rotationConfig>::CallbackType f;
    void callback_d(rotation_filter::rotationConfig &config, uint32_t level);
    void store_points_boundaries_rviz();
    void points_corner_computes();
    void points_corner_computes_sub(double corner_x, double corner_y, double corner_z);
    void filter_points();

    geometry_msgs::Point point;
    geometry_msgs::Point point_centroid;
    std::vector<geometry_msgs::Point> points_for_corner;
    std_msgs::ColorRGBA color;

      enum RTF_TASK {
      /*2020/8/8*/SET_TF_TASK  = 1,
      /*2020/8/8*/SET_OFFSET_TASK = 2,
      /*2020/8/8*/SET_TF_OFFSET_TASK = 3,
      /*2020/8/11*/SET_FILTER_TASK = 4,
      };

    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::TransformListener listener_used_in_filtered_cb;
    tf::Transform transform;
    bool broadcaster_set;
    void set_broadcaster(std::string tf_target_ref, std::string tf_target, std::string tf_new, double offest_x, double offset_y, double offset_z);
    double R;
    double P;
    double Y;
    std::string tf_target_frame;
    std::string tf_new_frame_wrt_target_frame;
    tf::Quaternion new_q;

    bool filtered_;

    public:
    rotation_filter_();
    void spin_();
    typedef boost::function<bool (rotation_filter::rtf::Request& req, rotation_filter::rtf::Response& res)> rtf_t;
};

template <class T>
bool rotation_filter_<T>::rtf_main_callback(rotation_filter::rtf::Request& req, rotation_filter::rtf::Response& res)
{
      ROS_INFO_STREAM("[RTF] A service was called ");
      res.module_ = "RTF";
      int error_ = 0;
        switch(req.task_id)
        {
         case SET_TF_TASK: {
         ROS_INFO_STREAM("RTF_TASK::SET_TF_TASK");
         set_broadcaster(req.tf_ref, req.tf_target, req.tf_new, 0, 0, 0);
         break;
       }
         case SET_OFFSET_TASK: {
         ROS_INFO_STREAM("RTF_TASK::SET_OFFSET_TASK");
         transform.setOrigin( tf::Vector3(req.offset_x, req.offset_y, req.offset_z) );
         break;
       }
        case SET_TF_OFFSET_TASK: {
         ROS_INFO_STREAM("RTF_TASK::SET_TF_OFFSET_TASK");
         set_broadcaster(req.tf_ref, req.tf_target, req.tf_new, req.offset_x, req.offset_y, req.offset_z);
         break;
       }
       case SET_FILTER_TASK: {
        ROS_INFO_STREAM("RTF_TASK::SET_FILTER_TASK");
        filtered_ = req.filter;
        break;
      }


       default:
       error_ = 2;
       break;
        }//switch

       res.error_ = error_;
       return true;
 }

template <class T>
rotation_filter_<T>::rotation_filter_()
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

    points_filtered_extract = n.advertise<PointCloud> ("points_extract",1);
    points_filtered_passthrough = n.advertise<PointCloud> ("points_passthrough",1);
    vis_pub =  n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    sub_pcl = n.subscribe("/see_scope/points", 1, &rotation_filter_::callback, this);
    f = boost::bind(&rotation_filter_::callback_d, this, _1, _2);
    server.setCallback(f);

    // http://www.menucool.com/rgba-color-picker
    color.a = 1.0;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    points_for_corner.reserve(16);
    points_corner_computes();
    store_points_boundaries_rviz();

    rtf_t rtf_main_callback = boost::bind(&rotation_filter_::rtf_main_callback, this, _1, _2);
    rtf_sv = n.advertiseService("/rtf_client", rtf_main_callback);

    broadcaster_set = false;
    filtered_ = false;
  }

template <class T>
void rotation_filter_<T>::callback(const sensor_msgs::PointCloud2Ptr& cloud)
{
    pcl::fromROSMsg(*cloud, pointcloud_in);  // copy sensor_msg::Pointcloud message into pcl::PointCloud
  }

template <class T>
void rotation_filter_<T>::callback_d(rotation_filter::rotationConfig &config, uint32_t level)
{
   ROS_INFO("Reconfigure Request: %f %f %f - %f %f %f - %f %f %f",
              config.passthrough_x_p, config.passthrough_y_p,config.passthrough_z_p,
              config.passthrough_x_n, config.passthrough_y_n,config.passthrough_z_n,
              config.rotation_x, config.rotation_y,config.rotation_z);

              rotation_x = config.rotation_x * M_PI/180.0;
              rotation_y = config.rotation_y * M_PI/180.0;
              rotation_z = config.rotation_z * M_PI/180.0;

              R = 0 + rotation_x;
              P = 0 + rotation_y;
              Y = 0 + rotation_z;
              //R = R + rotation_x;
              //P = P + rotation_y;
              //Y = Y + rotation_z;
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

template <class T>
void rotation_filter_<T>::spin_()
{
    ros::Rate loop_rate(5);
    int seq = 0;
    while (ros::ok())
    {
      if(broadcaster_set) br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tf_target_frame, tf_new_frame_wrt_target_frame));
      if(filtered_) filter_points();
      ros::spinOnce();
      loop_rate.sleep();
    }
// end spin()
}

template <class T>
void rotation_filter_<T>::points_corner_computes_sub(double corner_x, double corner_y, double corner_z)
{
    static tf::Vector3 v3_temp;
    v3_temp = transform *(tf::Vector3(corner_x, corner_y, corner_z));
    point.x = v3_temp.x();
    point.y = v3_temp.y();
    point.z = v3_temp.z();
    points_for_corner.push_back(point);
}

template <class T>
void rotation_filter_<T>::points_corner_computes()
{
    // 1
    points_for_corner.clear();
    //point.x = point_centroid.x + passthrough_x_p;
    //point.y = point_centroid.y + passthrough_y_p;
    //point.z = point_centroid.z + passthrough_z_p;
    //points_for_corner.push_back(point);
    points_corner_computes_sub(point_centroid.x + passthrough_x_p,
                               point_centroid.y + passthrough_y_p,
                               point_centroid.z + passthrough_z_p);
    // 2
    //point.x = point_centroid.x + passthrough_x_n;
    //point.y = point_centroid.y + passthrough_y_p;
    //point.z = point_centroid.z + passthrough_z_p;
    //points_for_corner.push_back(point);
    points_corner_computes_sub(point_centroid.x + passthrough_x_n,
                               point_centroid.y + passthrough_y_p,
                               point_centroid.z + passthrough_z_p);
    // 3
    //point.x = point_centroid.x + passthrough_x_n;
    //point.y = point_centroid.y + passthrough_y_n;
    //point.z = point_centroid.z + passthrough_z_p;
    //points_for_corner.push_back(point);
    points_corner_computes_sub(point_centroid.x + passthrough_x_n,
                               point_centroid.y + passthrough_y_n,
                               point_centroid.z + passthrough_z_p);
    // 4
    //point.x = point_centroid.x + passthrough_x_p;
    //point.y = point_centroid.y + passthrough_y_n;
    //point.z = point_centroid.z + passthrough_z_p;
    //points_for_corner.push_back(point);
    points_corner_computes_sub(point_centroid.x + passthrough_x_p,
                               point_centroid.y + passthrough_y_n,
                               point_centroid.z + passthrough_z_p);
    // 5
    //point.x = point_centroid.x + passthrough_x_p;
    //point.y = point_centroid.y + passthrough_y_p;
    //point.z = point_centroid.z + passthrough_z_n;
    //points_for_corner.push_back(point);
    points_corner_computes_sub(point_centroid.x + passthrough_x_p,
                               point_centroid.y + passthrough_y_p,
                               point_centroid.z + passthrough_z_n);
    // 6
    //point.x = point_centroid.x + passthrough_x_n;
    //point.y = point_centroid.y + passthrough_y_p;
    //point.z = point_centroid.z + passthrough_z_n;
    //points_for_corner.push_back(point);
    points_corner_computes_sub(point_centroid.x + passthrough_x_n,
                               point_centroid.y + passthrough_y_p,
                               point_centroid.z + passthrough_z_n);
    // 7
    //point.x = point_centroid.x + passthrough_x_n;
    //point.y = point_centroid.y + passthrough_y_n;
    //point.z = point_centroid.z + passthrough_z_n;
    //points_for_corner.push_back(point);
    points_corner_computes_sub(point_centroid.x + passthrough_x_n,
                               point_centroid.y + passthrough_y_n,
                               point_centroid.z + passthrough_z_n);
    // 8
    //point.x = point_centroid.x + passthrough_x_p;
    //point.y = point_centroid.y + passthrough_y_n;
    //point.z = point_centroid.z + passthrough_z_n;
    //points_for_corner.push_back(point);
    points_corner_computes_sub(point_centroid.x + passthrough_x_p,
                               point_centroid.y + passthrough_y_n,
                               point_centroid.z + passthrough_z_n);

}

template <class T>
void rotation_filter_<T>::store_points_boundaries_rviz()
{
    marker_for_drawing_rviz.points.clear();
    marker_for_drawing_rviz.colors.clear();
    marker_for_drawing_rviz.header.frame_id = tf_target_frame;
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

template <class T>
void rotation_filter_<T>::set_broadcaster(std::string tf_target_ref, std::string tf_target, std::string tf_new, double offest_x, double offset_y, double offset_z)
{
  // first listen to the tf_ref
  tf::StampedTransform stamped_transform;
  static bool No_info;
  No_info = false;
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

template <class T>
void rotation_filter_<T>::filter_points()
{
    // 1. assuming we have the transformation : transform
    // 2. assuming we have the camera frame: tf_target_frame
    // 3. assuming we have the point frame specified by user as the centroid: tf_new_frame_wrt_target_frame

    // Setp1 : transform the point cloud from camera frame to point frame
    //                           P_camera_frame          P_user_frame       R_transform
    pcl_ros::transformPointCloud(pointcloud_in, pointcloud_user_frame,      transform.inverse());

     // passthrough filtering
      /*
      In order to use the getRemovedIndices system you need to initialize your class differently:
      change this:
      pcl::PassThrough<pcl::PointXYZ> pass;
      into this:
      pcl::PassThrough<pcl::PointXYZ> pass (true);
      */
      typename PointCloud::Ptr transformed_cloud (new PointCloud(pointcloud_user_frame));
      typename pcl::PassThrough<T> pass(true);
      pass.setInputCloud (transformed_cloud); // setinput needs ptr
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (passthrough_x_n, passthrough_x_p);
      pass.filter (*transformed_cloud);  // filter needs point cloud
      pass.getRemovedIndices(PointIndices_x);
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
      typename PointCloud::Ptr source_cloud_ex(new PointCloud(pointcloud_in));
      pcl::PointIndices::Ptr indices_x(new pcl::PointIndices(PointIndices_x));
      pcl::PointIndices::Ptr indices_y(new pcl::PointIndices(PointIndices_y));
      pcl::PointIndices::Ptr indices_z(new pcl::PointIndices(PointIndices_z));
      typename pcl::ExtractIndices<T> extract;
      extract.setInputCloud(source_cloud_ex);
      extract.setIndices(indices_x);
      extract.setNegative(true);
      extract.filter(*source_cloud_ex);
      extract.setInputCloud(source_cloud_ex);
      extract.setIndices(indices_y);
      extract.setNegative(true);
      extract.filter(*source_cloud_ex);
      extract.setInputCloud(source_cloud_ex);
      extract.setIndices(indices_z);
      extract.setNegative(true);
      extract.filter(*source_cloud_ex);

      std_msgs::Header header;
      header.stamp = ros::Time::now();
      //header.frame_id = std::string("camera_frame");
      header.frame_id = std::string(tf_target_frame);
      source_cloud_ex->header = pcl_conversions::toPCL(header);
      points_filtered_extract.publish(*source_cloud_ex); // pointcloud_in from camera_frame

      //header.frame_id = std::string("user_frame");
      header.frame_id = std::string(tf_new_frame_wrt_target_frame);
      transformed_cloud->header = pcl_conversions::toPCL(header);
      points_filtered_passthrough.publish(*transformed_cloud); //pointcloud_out from user_frame



}
