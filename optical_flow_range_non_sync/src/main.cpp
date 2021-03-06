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
 #include <pcl/segmentation/segment_differences.h>
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
 #include <pcl/visualization/interactor_style.h>
 
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
 #include <pcl/pcl_base.h>
#include "pcl/kdtree/kdtree.h"

 
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
 
 #include "image_utilities.h"
 //#include "image_utilities.h"
 ///EIGEN
 
 #include <Eigen/Dense>

 
 #include <visualization_msgs/Marker.h>
 
  #include <pcl/range_image/range_image_planar.h>
  
  #include <pcl/common/transforms.h>

 using namespace pcl;
 using namespace std;
 using namespace Eigen;
 using namespace message_filters;
 
 using namespace cv;
 namespace enc = sensor_msgs::image_encodings;
 
 
 ///////////////////////////////////////////////////
 /// subscribers 
 /////////////////////////////////////////////////
 typedef message_filters::Subscriber<sensor_msgs::PointCloud2 > CloudSubscriber;
  //////////////////////////////////////////////////////

 ///publishers

 /////////////////////////////////////////////////////
 ros::Publisher pub_im_range_flow,pub_im_range_color_flow;

 ros::Publisher pub_im_range;
 ros::Publisher pub_cloud_flow,pub_cloud_normal,pub_cloud;
 ros::Publisher pub_markers;
 
 

 
 ///calibration parameters
 string file;
 Matrix3f K_;
 MatrixXf P_;
 MatrixXf P_ext;
 Matrix3f Trans_;
 Matrix3f R_;

 Vector3f T_;

 
 ///images range  flow
  bool firstFrame=1;
  Mat im_range_current;
  Mat im_range_previous;
  Mat im_range_current_norm;
  Mat im_range_previous_norm;
  Mat flow,cflow,color_flow;
  Mat falseColorsMap;
   
 ///range image
 boost::shared_ptr<pcl::RangeImage> range_image(new pcl::RangeImage);
 boost::shared_ptr<pcl::RangeImagePlanar> range_image_planar(new pcl::RangeImagePlanar);
 
 ///////////////////////////////////////////////////////////////////
 ///Syncronize objects  only for clouds
 //////////////////////////////////////////////////////////////////
 void Callback(const sensor_msgs::PointCloud2ConstPtr& cloudI)
 {
  
   
   ROS_INFO("cloud timestamps %i.%i ", cloudI->header.stamp.sec,cloudI->header.stamp.nsec);
      
   //////////////////////////////////////////////////////////////////
   ///for range images
   /////////////////////////////////////////////////////////////////
   
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

     
   ///segment data only the part in front of
   SegmentData(cloud_ptr,cloud_filtered,cloud_nonPlanes,cloud_Plane,cloud_projected);
    
   ///////////////////////////////////////////////
   ///image range images
   //////////////////////////////////////////////
   range_image->createFromPointCloud (*cloud_filtered, pcl::deg2rad(0.2f),
				     pcl::deg2rad (360.f), pcl::deg2rad (180.0f),
				     sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
   
   /////////////////////////////////////////////////////////////////////////////////////////////
   ///planar range image TESTING
   //////////////////////////////////////////////////////////////////////////////////////////
   
   Eigen::Affine3f trans,P_affine;//does not include K
   

     trans (0,0)= 0.0f; trans (0,1)= 0.0f; trans (0,2)=-1.0f; trans(0,3)=0.0f;       

     trans (1,0)= 0.0f; trans (1,1)= 1.0f; trans (1,2)=0.0f; trans (1,3)=0.0f;

     trans (2,0)= 1.0f; trans (2,1)= 0.0f; trans (2,2)=0.0f; trans (2,3)=0.0f;

     trans (3,0)= 0.0f; trans (3,1)= 0.0f; trans (3,2)=0.0f; trans (3,3)=1.0f;
     
     
     P_affine (0,0)= 0.0187f; P_affine (0,1)= -0.8024f; P_affine (0,2)= -0.5965f; P_affine(0,3)=-0.7813f;       

     P_affine (1,0)= -0.9998f; P_affine (1,1)= -0.0168f; P_affine (1,2)=-0.0088f; P_affine (1,3)=0.1818f;

     P_affine (2,0)= -0.0030f; P_affine (2,1)= 0.5965f; P_affine (2,2)=-0.8026f; P_affine (2,3)=0.7670f;

     P_affine (3,0)= 0.0f; P_affine (3,1)= 0.0f; P_affine (3,2)=0.0f; P_affine (3,3)=1.0f;
     
     
//    0.0187   -0.8024   -0.5965   -0.7813
//    -0.9998   -0.0168   -0.0088    0.1818
//    -0.0030    0.5965   -0.8026    0.7670

   for(unsigned int i=0; i<cloud_filtered->size();i++)
   {
      PointXYZ p1,p2;
      p1=cloud_filtered->points[i];
      
      p2.x=-p1.z;
      p2.y=p1.y;
      p2.z=p1.x;
      
      cloud_trans->push_back(p2);
   }
     
  // pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, trans);
   Eigen::Affine3f pose(Eigen::Affine3f::Identity());
  // pose=pose*Translation3f(T_);
   pose=P_affine;
   ///
   range_image_planar->createFromPointCloudWithFixedSize(*cloud_filtered,768, 1024,
			//K_(0,2),K_(1,2),K_(0,0),K_(1,1),P_affine
			K_(0,2),K_(1,2),K_(0,0),K_(1,1),  Eigen::Affine3f::Identity()
                             ,pcl::RangeImage::LASER_FRAME, noiseLevel, minRange);
 
   //GenerarImagenOpenCV(range_image,im_range_current, im_range_current_norm);
   //TESTING PART with planar range image
  GenerarImagenOpenCV(range_image_planar,im_range_current, im_range_current_norm);

  applyColorMap( im_range_current_norm, falseColorsMap, COLORMAP_HOT);


   if(!firstFrame)

   {
    Mat im2 = Mat(im_range_previous_norm.rows, im_range_previous_norm.cols, im_range_previous_norm.type());
    cv::resize(im_range_current_norm, im2,im2.size(), INTER_LINEAR );
    cv::calcOpticalFlowFarneback(im2,im_range_previous_norm,flow,0.5, 3, 15, 3, 5, 1.2, 0);
   
    ///show data

    cv::cvtColor( im2 ,cflow, CV_GRAY2BGR);
   
    drawOptFlowMap(flow, cflow, 10, 1.5, Scalar(0, 255, 0));
    drawOpticalFlow_color(flow,color_flow, -1);
  
   
    
     
    //Set_range_flow(range_image,im2,flow,cloud_flow,cloud_p3d,normals,cloud_rgb);
    ///planar range image TESTING
    Set_range_flow(range_image_planar,im2,flow,cloud_flow,cloud_p3d,normals,cloud_rgb);
  
   ////////////////////////////////////////////////////////////////////////////////////
    ///publishing
    //////////////////////////////////////////////////////////////////////////////////
  
    cv_bridge::CvImagePtr  cv_send(new cv_bridge::CvImage);

    cv_send->image=Mat(falseColorsMap.rows, falseColorsMap.cols, CV_8UC3);
    cv_send->encoding=enc::RGB8;

   ///range image 

    falseColorsMap.copyTo(cv_send->image);

    pub_im_range.publish(cv_send->toImageMsg());


   //flow arrows

   
   cv_send->image=Mat(cflow.rows, cflow.cols, CV_8UC3);
   cflow.copyTo(cv_send->image);
   pub_im_range_flow.publish(cv_send->toImageMsg());

   
   ///color flow

   cv_send->image=Mat( color_flow.rows,  color_flow.cols, CV_8UC3);
   color_flow.copyTo(cv_send->image);

     pub_im_range_color_flow.publish(cv_send->toImageMsg());

     
   ///cloud flow
    sensor_msgs::PointCloud2 output_cloud;
   pcl::toROSMsg(*cloud_rgb,output_cloud);
   output_cloud.header.frame_id = cloudI->header.frame_id;
   pub_cloud_flow.publish(output_cloud);

   
    ////////////////////////////////////
   ///cloud complete
   ///////////////////////////////////////
   pub_cloud.publish(cloudI);
    ////////////////////////////////////////////
   ///point normal
   /////////////////////////////////////////////
   sensor_msgs::PointCloud2 output_cloud2;  
   pcl::concatenateFields(*cloud_p3d,* normals,*cloud_normals);

   

   pcl::toROSMsg(*cloud_normals,output_cloud2);
   output_cloud2.header.frame_id = cloudI->header.frame_id;
   pub_cloud_normal.publish(output_cloud2);
   
   //////////////////////////////////////////////////////////////
   ///markers
   ///////////////////////////////////////////////////////////
   visualization_msgs::Marker points,line_list;
   Publish_Marks(cloud_flow,cloud_p3d, points,line_list);
   pub_markers.publish(points);
   pub_markers.publish(line_list);
   
  }
  
    

  //create the images
  firstFrame=0;// we have read the first frame

  im_range_current_norm.copyTo(im_range_previous_norm);
  flow=Mat(im_range_current_norm.rows, im_range_current_norm.cols,CV_32FC2);
  cflow=Mat(im_range_current_norm.rows, im_range_current_norm.cols, CV_8UC3);
  
  
  
    
 }
 void help(){
   
   
   
   ROS_ERROR("Error please try : \n\n"   );
   
   ROS_ERROR("rosrun optical_flow_range_non_sync optical_flow_range_non_sync  --file /home/aortega/Experiments/velodyne_32E/camera-calibration/calib1.yaml clouds:=/velodyne_points\n"   );
   ROS_ERROR("try with--> rosbag play  /home/aortega/Experiments/velodyne_32E/sequences/sequence_me.bag\n"   );
   
 }
 int read_arguments(int argc, char **argv,
		    
		    string & file_calib)
 {
   if(pcl::console::find_argument (argc, argv, "--file") >= 1){
     
     
     
     console::parse_argument (argc, argv, "--file", file_calib);
     
     
     
      ROS_INFO("File camera calibration %s\n\n",file_calib.c_str());
     
   }else
     
   {
     
     ROS_INFO("File %s\n\n",file_calib.c_str());
     
     help();
     
     return(-1);

   }
 }
 
 void read_calib()
 {
   
   K_=Matrix3f::Zero();
   P_=MatrixXf::Zero(3,4);
   
   //openfile
   OpenYalm(file, K_,P_);
   
   ROS_INFO("Laser-Camera calibration parameters read \n");
   
   
   P_ext=MatrixXf::Zero(4,4);
   
   P_ext<< P_(0,0),P_(0,1),P_(0,2),P_(0,3),
           P_(1,0),P_(1,1),P_(1,2),P_(1,3),
           P_(2,0),P_(2,1),P_(2,2),P_(2,3),
           0, 0, 0, 1;
           
   R_<< 0.0187,   -0.8024,   -0.5965,
       -0.9998,   -0.0168,   -0.0088,
       -0.0030 ,   0.5965,   -0.8026;

   T_<<-0.7813,
    0.1818,
      0.7670;
      
   cout<<"K->\n"<<K_<<endl;
   cout<<"P->\n"<< P_<<endl;
   cout<<"Pext->\n"<< P_ext<<endl;
   ///transformation cloud
    Trans_<<0 ,0 ,-1,
   0, 1 ,0,
   1, 0 ,0;
 }
 
 int main (int argc, char** argv){
   
   
   
   if(read_arguments(argc, argv,file)==-1)
   {
     
     
     
     return (-1);
     
   }
   
   else
   {
     //////////////////////////////////////////////////
     /// read calibration
     //////////////////////////////////////////////////
     read_calib();

     
     ros::init(argc, argv, "Syncronization_node");
      ros::Subscriber sub_; 
     ros::NodeHandle nh_; 
     string cloud_topic_;
     CloudSubscriber cloud_sub;   
  
     
    cloud_topic_= nh_.resolveName("clouds");
    sub_=nh_.subscribe (cloud_topic_, 1,  Callback );
     
     
     
     
     /////////////////////////////////////////////////////
     ///publishers
     ///////////////////////////////////////////////////
    
     pub_im_range= nh_.advertise<sensor_msgs::Image>("image_range_out", 1);
     pub_im_range_flow= nh_.advertise<sensor_msgs::Image>("image__range_flow", 1);
     pub_im_range_color_flow=nh_.advertise<sensor_msgs::Image>("image_range_color_flow", 1);


     pub_cloud_flow= nh_.advertise<sensor_msgs::PointCloud2>("cloud_flow", 1);
     pub_cloud= nh_.advertise<sensor_msgs::PointCloud2>("cloud_complete", 1);
     pub_cloud_normal= nh_.advertise<sensor_msgs::PointCloud2 >("cloud_normal", 1);
     pub_markers= nh_.advertise<visualization_msgs::Marker >("Markers", 1);
      
     ros::spin();
     
   }
   return 0;
   
 }
 
 
