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
 
 //using terminal_tools::parse_argument;
 
 
 
#include "manual_calib.h"
 
 
 using namespace pcl;
 using namespace std;
 using namespace Eigen;
 
 using namespace cv;

 
int idx=1;
pcl::PointCloud<PointXYZ>::Ptr cloud_p(new PointCloud<pcl::PointXYZ> ());

pcl::PointCloud<PointXYZ>::Ptr cloud_p2(new PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<PointXYZ>::Ptr cloud_Manual(new PointCloud<pcl::PointXYZ> ());
 //////////////////////////////////////////////////////////

 //this is our principal class
 //////////////////////////////////////////////////////////

//  void PickingEventOccurred_(const visualization::PointPickingEvent& event, void* viewer_void){
// 
//   
// 
//   boost::shared_ptr<visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<
// 
//   visualization::PCLVisualizer> *>(viewer_void);
// 
// 
//   idx = event.getPointIndex ();
// 
//   if (idx == -1)
// 
//     return;
// 
//   
// 
//   vector<int> indices (1);
// 
//   vector<float> distances (1);
// 
//  
//   // Use mutices to make sure we get the right cloud
// 
//   //boost::mutex::scoped_lock lock1 (cloud_mutex_);
// 
//   
// 
//   pcl::PointXYZ   PointAux;
// 
//   
// 
//   event.getPoint ( PointAux.x,  PointAux.y,  PointAux.z);
// 
//   
// 
//   char str[512];
// 
//   sprintf(str, "sphere_%d",idx);
// 
//   
// 
//   std::cout << " (" << PointAux.x << ", " << PointAux.y <<", " << PointAux.z<< "......)"
// 
//   << str<<std::endl;
// 
//   
// 
//   cloud_p->points.push_back (PointAux);
// 
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_p, 255, 0, 0);
// 
//   //viewer_void->addSphere(PointAux, str);
// 
//   viewer->addPointCloud<pcl::PointXYZ>(cloud_p,single_color, str,0);
// 
//   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,str);
// 
//   
// 
// }

 void help(){
   
   ROS_ERROR("Error please try : \n\n"   );
   ROS_ERROR("rosrun laser_calib_manual  laser_calib_manual input:=/teo/h3d/h3dcloud image:=/teo/stereo/right/image_raw --file /home/aortega/Experiments/calibration/camera_right.yaml --method LU \n"   );
   ROS_ERROR("Try with this data --->rosbag play front_stereo_h3d_2012-06-12-15-34-58.bag");
 }
 
 int Read_arguments(int argc, char **argv,
		    string & file, string& method){
   
   
   string strs_;
   
 
   if(pcl::console::find_argument (argc, argv, "--file") >= 1){
     
     console::parse_argument (argc, argv, "--file", file);
     
     ROS_INFO("File %s\n\n",file.c_str());
   }else
   {
     ROS_INFO("File %s\n\n",file.c_str());
     help();
     return(-1);
     
   }

   
   
 
    if(pcl::console::find_argument (argc, argv, "--method") >= 1){
     console::parse_argument (argc, argv, "--method", method);
     
     
   }else
   {
     
     help();
     return(-1);  
   }
   

   
   return(0);
   
   
 }
 
 
 int main (int argc, char** argv){
   
   string file,selection,method;
   int w,h;
   
   //////////////////////////////////////////////////////////
   // first check argumenst
   ////////////////////////////////////////////////////////
   if(Read_arguments(argc, argv,file,method)==-1){
     
     return (-1);
   }
   else{/////////start our node if the intputs are ok
     
     
     
     ros::init (argc, argv, "manual_calib", ros::init_options::AnonymousName);
  
     manual_calib calib(file,  method);
     
     //ros::spin ();
     ros::spin();
     
   }
   
   return 0;
   
 }
 
