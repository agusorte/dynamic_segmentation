 #include <string>
 #include <iostream>
 #include <rosbag/bag.h>
 #include <rosbag/view.h>
 #include "ros/ros.h"
 #include <ros/names.h>
 
 #include <boost/foreach.hpp>
 #include <boost/assign/list_of.hpp>
 
 #include <sensor_msgs/Image.h>
 #include <sensor_msgs/CameraInfo.h>
 #include <sensor_msgs/PointCloud2.h>
 
 #include <opencv/cv.h>
 #include <opencv/highgui.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 
 #include <message_filters/sync_policies/exact_time.h>
 
 #include <message_filters/sync_policies/approximate_time.h>
 
 #include <message_filters/subscriber.h>
 #include <message_filters/sync_policies/exact_time.h>
 #include <message_filters/time_synchronizer.h> 
 
 #include <pcl/ModelCoefficients.h>
 #include <pcl/point_types.h>
 #include <pcl/filters/extract_indices.h>
 #include <pcl/filters/voxel_grid.h>
 #include <pcl/features/normal_3d.h>
 #include <pcl/kdtree/kdtree.h>
 #include <pcl/sample_consensus/method_types.h>
 #include <pcl/sample_consensus/model_types.h>
 #include <pcl/segmentation/sac_segmentation.h>
 #include <pcl/segmentation/extract_clusters.h>
 
 
 #include <pcl/point_types.h>
 
 
 //visualization 
 #include <pcl/visualization/cloud_viewer.h>
 #include <pcl/visualization/pcl_visualizer.h>
 #include <pcl/visualization/range_image_visualizer.h>
 
 //segmentation
 #include <pcl/sample_consensus/method_types.h>
 #include <pcl/sample_consensus/model_types.h>
 #include <pcl/segmentation/sac_segmentation.h>
 #include <pcl/segmentation/extract_clusters.h>
 #include <pcl/kdtree/kdtree.h>  
 
 #include "pcl/io/io.h"
 #include "pcl/io/pcd_io.h"
 #include "pcl/filters/voxel_grid.h"
 #include "pcl/filters/passthrough.h"
 #include "pcl/filters/extract_indices.h"
 #include "pcl/features/normal_3d.h"
 
 #include <pcl/features/normal_3d.h>
 #include <pcl/kdtree/kdtree.h>
 #include "pcl/kdtree/io.h"
 #include <pcl/sample_consensus/method_types.h>
 #include <pcl/sample_consensus/model_types.h>
 #include <pcl/segmentation/sac_segmentation.h>
 #include <pcl/segmentation/extract_clusters.h>
 
 #include <pcl/console/parse.h>
 // #include <terminal_tools/print.h>
 // #include <terminal_tools/parse.h>
 
 #include <pcl/filters/statistical_outlier_removal.h>
 #include <pcl/filters/extract_indices.h>
 #include <pcl/filters/project_inliers.h>
 
 
 #include "cxcore.h"
 #include <cv.h>
 #include <highgui.h>
 #include <math.h>
 #include <vector>
 #include <ros/ros.h>
 #include "sensor_msgs/Image.h"
 #include "image_transport/image_transport.h"
 #include "cv_bridge/CvBridge.h"
 #include <opencv/cv.h>
 #include <opencv/highgui.h>
 
 #include <pcl/console/parse.h>
 
 
 
 using namespace pcl;
 using namespace std;
 using namespace Eigen;
 using namespace message_filters;
 
 using namespace cv;
 namespace enc = sensor_msgs::image_encodings;
 
 typedef message_filters::Subscriber< sensor_msgs::Image > ImageSubscriber;
 
 typedef message_filters::Subscriber<sensor_msgs::PointCloud2 > CloudSubscriber;
 boost::shared_ptr<visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

 boost::shared_ptr<pcl::visualization::RangeImageVisualizer> range_image_widget(new pcl::visualization::RangeImageVisualizer("Range image"));

 
 
 boost::shared_ptr<pcl::RangeImage> range_image(new pcl::RangeImage);
 
 void Callback(const sensor_msgs::PointCloud2ConstPtr& cloudI,
	       const sensor_msgs::ImageConstPtr& msg)
 {
    ROS_INFO("image2 timestamps %i.%i ", msg->header.stamp.sec,msg->header.stamp.nsec);

    ROS_INFO("cloud timestamps %i.%i ", cloudI->header.stamp.sec,cloudI->header.stamp.nsec);
   
   
    //////////////////////////////////////////////////////////////////////////////////
    ///initialize objects visualization 
    //////////////////////////////////////////////////////////////////////////////////
    
    viewer->setBackgroundColor (0, 0, 0);

    
    viewer->addCoordinateSystem (1.0);

    
    
    viewer->initCameraParameters ();

    viewer->addCoordinateSystem(1.0);

    viewer->setCameraPose (-10,0,0,1,0,0,0,0,1);
    
   
 

    
    //for range images

    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

    float noiseLevel=0.00;

    float minRange = 0.0f;

    int borderSize = 1;

  
    /////////////////////////////////////////////////////////////////////
    /// point cloud 
    ///////////////////////////////////////////////////////////////////
    PointCloud<PointXYZ>::Ptr cloud_ptr (new PointCloud<PointXYZ>);
   
   
   if ((cloudI->width * cloudI->height) == 0)
     return ;
   
   ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
   
   (int)cloudI->width * cloudI->height,
   
   cloudI->header.frame_id.c_str (),
   
   pcl::getFieldsList (*cloudI).c_str ());
   
   
   
   
   pcl::fromROSMsg(*cloudI, *cloud_ptr);
   
   ////////////////////////////////////////////////////////////////
   ///image
   ////////////////////////////////////////////////////////////////
   cv_bridge::CvImageConstPtr cv_ptr;
   
   try
   
   {
     
     if (enc::isColor(msg->encoding))
       
       cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
     
     else
       
       cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
     
     ROS_INFO("Image data size (%d, %d)",cv_ptr->image.rows, cv_ptr->image.cols);
     
   }
   
   catch (cv_bridge::Exception& e)
   
   {
     
     ROS_ERROR("cv_bridge exception: %s", e.what());
     
     
     return ;
     
   }
   
   /////////////////////////////////////////////////////////////////////////
   /// visualize data
   ////////////////////////////////////////////////////////////////////////
   
   //////////////////////////////////////////////////////////////////////////////////////////////////////

  /// visualize cloud

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  visualization::PointCloudColorHandlerCustom <PointXYZ> point_cloud_color_handler (cloud_ptr, 150, 150, 150);   
  viewer->addPointCloud<PointXYZ> (cloud_ptr, point_cloud_color_handler, "original point cloud"); 
  
  range_image->createFromPointCloud (*cloud_ptr, pcl::deg2rad(0.6f),
				     pcl::deg2rad (360.f), pcl::deg2rad (180.0f),

				     sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
 
  range_image_widget->showRangeImage(*range_image);
  viewer->spinOnce();
  pcl_sleep(1.0);
  
  viewer->removePointCloud("original point cloud");
  ROS_INFO("pass here");
  ///////////////////////////////////////////////////////////////////
  /// image 
  /////////////////////////////////////////////////////////////////
  Mat im_current;
  im_current=cv_ptr->image.clone();
 
  imshow( "Image",  im_current);  
  ROS_INFO("pass here");
  waitKey(30);
 }
 
 
 
 int main (int argc, char** argv){
   
   
   typedef sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
   typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy_Aprox;
   
   ros::init(argc, argv, "Syncronization_node");
   
   ros::NodeHandle nh_; 
   string cloud_topic_;
   
   string image_topic_;
   
   
   
   CloudSubscriber cloud_sub;
   
   ImageSubscriber image_sub;

   
   ROS_INFO("Registering data");
   
   
   cloud_sub.subscribe(nh_,"clouds",1);
   image_sub.subscribe(nh_ ,"images" ,1);
   
   
   ROS_INFO("try rosrun velodyne_process_sync_topics velodyne_process_sync_topics clouds:/velodyne_points images:/camera/image_raw");
   //TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(cloud_sub, image_sub, 10);
   //TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub, image_sub2, 1000);
   // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, image_sub2);
   Synchronizer<MySyncPolicy_Aprox> sync(MySyncPolicy_Aprox(10), cloud_sub, image_sub);
   
   sync.registerCallback(boost::bind(&Callback,_1,_2));
   
   
   ros::spin();
   
   
   return 0;
   
 }
 
