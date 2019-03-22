#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl/range_image/range_image.h>
#include "std_msgs/String.h"
#include <vector>
#include <pcl/pcl_macros.h>
#include <pcl/common/distances.h>

#include <pcl/point_types.h>
#include <typeinfo>


/**
 * 
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <boost/foreach.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include <tf_conversions/tf_eigen.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub;

boost::shared_ptr<tf::TransformListener> g_tf;



class My_Range_Image: public pcl::RangeImage 
{
public:
 template <typename PointCloudType> void 
 v_range_image(const PointCloudType& pointCloud,
		      float angularResolution,
		      float maxAngleWidth,
		      float maxAngleHeight,
		      pcl::RangeImage::CoordinateFrame coordinate_frame,
		      Eigen::Affine3f& sensorPose,
		      float minRange,
		      float noiseLevel);
};


 template <typename PointCloudType> void 
 My_Range_Image::v_range_image(const PointCloudType& pointCloud,
		      float angularResolution,
		      float maxAngleWidth,
		      float maxAngleHeight,
		      pcl::RangeImage::CoordinateFrame coordinate_frame,
		      Eigen::Affine3f& sensorPose,
		      float minRange,
		      float noiseLevel)
{
  setAngularResolution (angularResolution, angularResolution);

  width  = static_cast<uint32_t> (pcl_lrint (floor (maxAngleWidth*angular_resolution_x_reciprocal_)));
  height = static_cast<uint32_t> (pcl_lrint (floor (maxAngleHeight*angular_resolution_y_reciprocal_)));

  int full_width  = static_cast<int> (pcl_lrint (floor (pcl::deg2rad (360.0f)*angular_resolution_x_reciprocal_)));
  int full_height = static_cast<int> (pcl_lrint (floor (pcl::deg2rad (180.0f)*angular_resolution_y_reciprocal_)));
  
  image_offset_x_ = (full_width -static_cast<int> (width) )/2;
  image_offset_y_ = (full_height-static_cast<int> (height))/2;
  is_dense = false;
  getCoordinateFrameTransformation (coordinate_frame, to_world_system_);
  to_world_system_ = sensorPose * to_world_system_;

  to_range_image_system_ = to_world_system_.inverse (Eigen::Isometry);
  
  unsigned int size = width*height;
  points.clear();
  points.resize (width*height);
  
  int top=height, right=-1, bottom=-1, left=width;
  doZBuffer (pointCloud, noiseLevel, minRange, top, right, bottom, left);
}


tf::StampedTransform get_transform()
{
  g_tf->waitForTransform("laser_scanner_link_3", "camera_depth_optical_frame", ros::Time(0), ros::Duration(5.0));
  tf::StampedTransform transform ;
  try
  {
    g_tf->lookupTransform("laser_scanner_link_3", "camera_depth_optical_frame", ros::Time(0), transform);
  //  printf("Got the transform!\n");
    return transform;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Exception caught %s",ex.what());
    ros::Duration(1.0).sleep();
    return transform;
  }
}

void to_range_image(PointCloud::Ptr pointCloud)
{

  // Our parameters----
  float angularResolution = (float) (  0.5f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (130.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (1.0 * (M_PI/180.0f));  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;
  float minRange = 0.0f;
  int borderSize = 1;
  
  // Our code for creating range image...
  My_Range_Image  my_range_image;
  
  my_range_image.v_range_image(*pointCloud,
		      angularResolution,
		      maxAngleWidth,
		      maxAngleHeight,
		      coordinate_frame,
		      sensorPose,minRange,noiseLevel);
  
   
  
  /*
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(*pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  */
   
  //std::cout << "\n The converted rangeImage ="<<my_range_image << "\n";
  
  
  // Prepare the message, and publish it to the topic...
  sensor_msgs::LaserScan msg;
    
  msg.angle_min = -maxAngleWidth/2;
  msg.angle_max = maxAngleWidth/2;
  msg.angle_increment = angularResolution;
  msg.time_increment = 0;
  msg.scan_time = 0;
  
  float min_range, max_range;
  my_range_image.getMinMaxRanges(min_range, max_range);
  msg.range_min = min_range;
  msg.range_max = max_range;

  for( int i = my_range_image.width-1; i >= 0 ; i--)
  {
	float range = my_range_image.getPoint(i,0).range;
	if(range < 0 || range > 5.0)
		range = NAN;

	msg.ranges.push_back(range);
  }
  
  msg.header.frame_id = "laser_scanner_link";
  msg.header.stamp = pcl_conversions::fromPCL(pointCloud->header.stamp);
  pub.publish(msg);
}

// void PointCloudCallback(const PointCloud& source_cloud)
void PointCloudCallback(const PointCloud::ConstPtr& source_cloud)
{ 
  tf::StampedTransform transform = get_transform();
  Eigen::Affine3d transformEigen;
  tf::transformTFToEigen(transform, transformEigen);
  
  PointCloud::Ptr transformed(new PointCloud);
   
  pcl::transformPointCloud(*source_cloud, *transformed, transformEigen.cast<float>());
  
  to_range_image(transformed); 
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.Publishers:
   */
  
  ros::init(argc, argv, "camera_listener");

  
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n1;
  
  pub = n1.advertise<sensor_msgs::LaserScan>("laser_scanner_sim", 1000);
  g_tf.reset(new tf::TransformListener);

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */    
  ros::Subscriber sub = n1.subscribe<PointCloud>("/camera/depth_registered/points", 1, PointCloudCallback);
 
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

