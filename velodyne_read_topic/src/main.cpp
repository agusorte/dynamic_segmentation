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

 using namespace cv;


#include "process_cloud.h"
 void help(){
   
   ROS_ERROR("Error please try : \n\n"   );
   ROS_ERROR("velodye_read_topic clouds:=/velodyne_points images:/teo/stereo/left/image_raw"   );
 }
 
 
 
 int main (int argc, char** argv){
   

   //////////////////////////////////////////////////////////
   // first check argumenst
     ///if you want
   ////////////////////////////////////////////////////////
//    if(Read_arguments(argc, argv,file,w, h,selection,method)==-1){
//   
     ros::init( argc, argv, "process_cloud" );
    // ros::init (argc, argv, "process_cloud", ros::init_options::AnonymousName);
  
     process_cloud process_cloud;
     
   
     ros::spin();
     

   
   return 0;
   
 }
 