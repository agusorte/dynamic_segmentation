 ///////////////////////////////////////
 ///ROS
 /////////////////////////////////////
 #include <sensor_msgs/PointCloud2.h>
 #include "ros/ros.h"
 #include <message_filters/subscriber.h>
 #include <message_filters/time_synchronizer.h> 
 #include <ros/names.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <message_filters/sync_policies/exact_time.h>
 #include <message_filters/sync_policies/approximate_time.h>
 
 #include <rosbag/bag.h>
  #include <rosbag/view.h>
 ///OTHERS
 #include <string>
 #include <iostream>
 
 /////////////////////////////////
 ///PCL
 #include <pcl/point_types.h>
 #include <pcl/visualization/pcl_visualizer.h>
 #include <pcl/console/parse.h>
 #include <pcl/range_image/range_image.h>
 
 //visualization 
 
 #include <pcl/visualization/cloud_viewer.h>
 #include <pcl/visualization/pcl_visualizer.h>
 #include <pcl/visualization/range_image_visualizer.h>
 #include <pcl/visualization/range_image_visualizer.h>
 #include <pcl/filters/statistical_outlier_removal.h>
 #include <pcl/filters/extract_indices.h>
 #include <pcl/filters/project_inliers.h>
 #include <pcl/filters/passthrough.h>
 
//#include "boost/bind.hpp"
 
 #include "image_utilities.h"
 

 
 using namespace std;
 using namespace pcl;
 namespace enc = sensor_msgs::image_encodings;
 
 using namespace cv;
 using namespace message_filters;
 
 class process_cloud{
   
 protected:
   
   ros::NodeHandle nh_;
   
 private:
   string prefix_;
   
 public:
   string cloud_topic_;
   string image_topic_;
   string image_topic_2;
   
   ///ROS subscribers
   ros::Subscriber sub_; 
   ros::V_Subscriber subs_;
   
   
   
  typedef message_filters::Subscriber< sensor_msgs::Image > ImageSubscriber;
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2 > CloudSubscriber;

  
   CloudSubscriber cloud_sub;
   ImageSubscriber image_sub;
   ImageSubscriber image_sub2;
   
   //rosbag::Bag<sensor_msgs::Image> image_sub,Image_sub2;
   ////////////////////////////////////////////////////////////
   //visualization
   boost::shared_ptr<visualization::PCLVisualizer> viewer;
   boost::shared_ptr<pcl::visualization::RangeImageVisualizer> range_image_widget;
   boost::shared_ptr<pcl::RangeImage> range_image;
   
   ////////////////////////////////////////////////
   ///range image values
   Eigen::Affine3f sensorPose;
   pcl::RangeImage::CoordinateFrame coordinate_frame; 
   float noiseLevel;
   float minRange; 
   int borderSize;
   
   
   
   //////////////////////////////////////////////////////////////////////////
   ///constructor
   process_cloud();
   
   void cloud_Callback (const sensor_msgs::PointCloud2ConstPtr& cloudI);
   void image_Callback(const sensor_msgs::ImageConstPtr& msg);
   
   void Callback(const sensor_msgs::ImageConstPtr& msg,
		 const sensor_msgs::PointCloud2ConstPtr& cloudI
		 );
   
   void Callback_imgs(const sensor_msgs::ImageConstPtr& image1,
		 const sensor_msgs::ImageConstPtr& image2);
   
   void filter_cloud(PointCloud<PointXYZ>::ConstPtr cloud_,
		     const PointCloud<PointXYZ>::Ptr &  cloud_filtered);
   
   ////////////////////////////////////////////////////////////////////////
   /// images range processing point clouds
   bool firstFrame;
   Mat im_range_current;
   Mat im_range_previous;
   Mat im_range_current_norm;
   Mat im_range_previous_norm;
   Mat flow,cflow,color_flow;
   Mat falseColorsMap;
   ///////////////////////////////////////////////////////////////
   /// images range
   bool im_firstFrame;
   Mat im_current;
   Mat im_previous;
   Mat im_current_norm;
   Mat im_previous_norm;
   Mat im_flow,im_cflow,im_color_flow;
   Mat im_falseColorsMap;
   
   ///objects
   image_utilities image_utilities_obj;
   
   ///////////////////////////////////////////////////////////////////
   ///guassian mixtures
   BackgroundSubtractorMOG2 bg,bg_im;
   std::vector<std::vector<cv::Point> > contours;
   std::vector<std::vector<cv::Point> > im_contours;
   Mat back; // background image
   Mat fore; // foreground mask
   
   Mat im_back; // background image
   Mat im_fore; // foreground mask
 };