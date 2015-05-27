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

 using namespace pcl;
 using namespace std;
 using namespace Eigen;
 using namespace message_filters;
 
 using namespace cv;
 namespace enc = sensor_msgs::image_encodings;
 
 
 ///////////////////////////////////////////////////
 /// subscribers 
 /////////////////////////////////////////////////
 typedef message_filters::Subscriber< sensor_msgs::Image > ImageSubscriber;
 typedef message_filters::Subscriber<sensor_msgs::PointCloud2 > CloudSubscriber;
  //////////////////////////////////////////////////////

 ///publishers

 /////////////////////////////////////////////////////

 ros::Publisher pub_im_flow,pub_im_color_flow;
 ros::Publisher pub_im;
 ros::Publisher pub_cloud_flow,pub_cloud_normal,pub_cloud;
 ros::Publisher pub_markers;
 
 ///viewer
 boost::shared_ptr<visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

 
 ///range image visualizer
 boost::shared_ptr<pcl::visualization::RangeImageVisualizer> range_image_widget(new pcl::visualization::RangeImageVisualizer("Range image"));
 pcl::visualization::PCLVisualizerInteractorStyle vis_style;
 
 ///calibration parameters
 string file;
 Matrix3f K_;
 MatrixXf P_;
 MatrixXf P_ext;
 Matrix3f Trans_;
 
 ///images optical flow
 bool im_firstFrame=1;
 Mat im_current;
 Mat im_previous;
 Mat im_current_norm;
 Mat im_previous_norm;
 Mat im_flow,im_cflow,im_color_flow;
 Mat im_falseColorsMap;
 Mat im_current_color;
 
 ///images range 
  bool firstFrame=1;

   Mat im_range_current;

   Mat im_range_previous;

   Mat im_range_current_norm;

   Mat im_range_previous_norm;

   Mat flow,cflow,color_flow;

   Mat falseColorsMap;
   
 ///range image
 boost::shared_ptr<pcl::RangeImage> range_image(new pcl::RangeImage);
 
 
 ///////////////////////////////////////////////////////////////////
 ///Syncronize objects cloud projected and cloud in image
 //////////////////////////////////////////////////////////////////
 void Callback(const sensor_msgs::PointCloud2ConstPtr& cloudI,
	       const sensor_msgs::ImageConstPtr& msg)
 {
   ROS_INFO("image2 timestamps %i.%i ", msg->header.stamp.sec,msg->header.stamp.nsec);
   
   ROS_INFO("cloud timestamps %i.%i ", cloudI->header.stamp.sec,cloudI->header.stamp.nsec);
   
   
   //////////////////////////////////////////////////////////////////////////////////
   ///initialize objects visualization 
   //////////////////////////////////////////////////////////////////////////////////
   
   viewer->setBackgroundColor (0, 0, 0);
   //  viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters ();
   viewer->setCameraPose (-5,-5,1,1,0,0,0,0,1);
   
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
   
  
   
   ////////////////////////////////////////////////////////////////
   ///image
   ////////////////////////////////////////////////////////////////
   cv_bridge::CvImageConstPtr cv_ptr;
   
   try
   
   {
     
//      if (enc::isColor(msg->encoding))
       
       cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
     
//      else
//        
//        cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
//      
     ROS_INFO("Image data size (%d, %d)",cv_ptr->image.rows, cv_ptr->image.cols);
     
   }
   
   catch (cv_bridge::Exception& e)
   
   {
     
     ROS_ERROR("cv_bridge exception: %s", e.what());
     
     
     return ;
     
   }
   
   
   PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
    
   PointCloud<pcl::PointXYZ>::Ptr cloud_flow(new PointCloud<PointXYZ> ());

   PointCloud<pcl::PointXYZ>::Ptr cloud_p3d (new PointCloud<pcl::PointXYZ>);

   PointCloud<pcl::Normal>::Ptr normals (new PointCloud<pcl::Normal>);
   
   PointCloud<PointXYZ>::Ptr cloud_nonPlanes (new PointCloud<PointXYZ> ());


   PointCloud<pcl::PointXYZ>::Ptr cloud_Plane(new PointCloud<PointXYZ> ());


   PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new PointCloud<pcl::PointXYZ>);
   PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new PointCloud<pcl::PointXYZRGB>);
   PointCloud<pcl::PointNormal>::Ptr cloud_normals (new PointCloud<pcl::PointNormal>);
     
   ///segment data only the part in front of
   SegmentData(cloud_ptr,cloud_filtered,cloud_nonPlanes,cloud_Plane,cloud_projected);
    
   ///////////////////////////////////////////////
   ///image range images
   //////////////////////////////////////////////
   
   range_image->createFromPointCloud (*cloud_filtered, pcl::deg2rad(0.2f),

				     pcl::deg2rad (360.f), pcl::deg2rad (180.0f),

				     sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
   
 
   GenerarImagenOpenCV(range_image,im_range_current, im_range_current_norm);
  

   applyColorMap( im_range_current_norm, falseColorsMap, COLORMAP_HOT);


   if(!firstFrame)

   {
    Mat im2 = Mat(im_range_previous_norm.rows, im_range_previous_norm.cols, im_range_previous_norm.type());
    cv::resize(im_range_current_norm, im2,im2.size(), INTER_LINEAR );
    cv::calcOpticalFlowFarneback(im2,im_range_previous_norm,flow,0.5, 3, 15, 3, 5, 1.2, 0);
   
    ///show data

    cv::cvtColor( im2 ,cflow, CV_GRAY2BGR);
    //drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));

    drawOptFlowMap(flow, cflow, 10, 1.5, Scalar(0, 255, 0));
    drawOpticalFlow_color(flow,color_flow, -1);
  
   
    // ros::Rate loop_rate(10);
    
       
       //loop_rate.sleep();
//      }
     
    Set_range_flow(range_image,im2,flow,cloud_flow,cloud_p3d,normals,cloud_rgb);
//     ROS_ERROR("cloud flow %d %d",cloud_flow->width, cloud_flow->height);
//     ROS_ERROR("cloud flow %d %d",normals->width, normals->height);
    

      visualization::PointCloudColorHandlerCustom <PointXYZ> point_cloud_color_handler (cloud_ptr, 150, 150, 150);   



   viewer->addPointCloud<PointXYZ> (cloud_ptr, point_cloud_color_handler, "original point cloud");  
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original point cloud");
// 
//    viewer->addPointCloud<PointXYZ> (cloud_p3d, point_cloud_color_handler, "cloud  flow"); 
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,"cloud  flow");

   viewer->addPointCloudNormals<PointXYZ, Normal> (cloud_p3d, normals, 10, 0.2, "normals");
//    
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0,"normals");
   
   /*pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
   viewer->addPointCloud<PointXYZRGB> (cloud_rgb,rgb,"cloud rgb"); 
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud rgb");*/
   
   
   ///////////////////////////////////////////////////
   ///test with lines
   ////////////////////////////////////////////////////
   
   char str[512];


//   int step=50;
//   for (int k=0 ;k<cloud_flow->size();k=k+step)
//    {
//   
//      sprintf(str, "line_%d",k);
// 
// 
//     // viewer->addarrow<PointXYZ,PointXYZ>(cloud_p3d->points[k], cloud_flow->points[k],1.0, 1.0, 0,str,0);
//      
//      
//      viewer->addLine<PointXYZ>(cloud_p3d->points[k], cloud_flow->points[k],str,0);
//  
// 
//     //   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str);
// 
//     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0,str);
//      viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, str);
//      
// //      sprintf(str, "point_%d",k);
// 
// //      viewer->addPointCloud<PointXYZ>(P1,str,0);
// 
// //      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,str);
// 
// //      
//    }
//   
   
//    while (!viewer->wasStopped ())
//   {
//     viewer->spinOnce (100);
//     pcl_sleep(1.0);
//   }
//   
   viewer->spinOnce ();

  

   pcl_sleep(1.0);
   viewer->removePointCloud("original point cloud");
   //viewer->removePointCloud("cloud  flow");
   viewer->removePointCloud("normals");
 //  viewer->removePointCloud("cloud rgb");
   
//     imshow( "Optical flow",  cflow);  
//     imshow( "Optical flow 2",  color_flow);  
//     imshow("Range Image jet", falseColorsMap);
    
//      waitKey(30);
//    for (int k=0 ;k<cloud_flow->size();k=k+step)
//    {
// 
//      sprintf(str, "line_%d",k);
//   
//      viewer->removeShape(str);
//    
//    }
   ///////////////////////////////////////////////////////////////////
   ///publishing 
     /////////////////////////////////////////////////////////////////
   cv_bridge::CvImagePtr  cv_send(new cv_bridge::CvImage);

   
   cv_send->image=cv_ptr->image;

   cv_send->header=cv_ptr->header;

   cv_send->encoding=cv_ptr->encoding;
   
   ///range image 
   falseColorsMap.copyTo(cv_send->image);
   pub_im.publish(cv_send->toImageMsg());
   
 
   ///flow arrows
   cflow.copyTo(cv_send->image);
   pub_im_flow.publish(cv_send->toImageMsg());
   
   ///color flow
   color_flow.copyTo(cv_send->image);
   pub_im_color_flow.publish(cv_send->toImageMsg());
   
   ///clouds
    sensor_msgs::PointCloud2 output_cloud;
   
   ///cloud flow

   pcl::toROSMsg(*cloud_rgb,output_cloud);

   output_cloud.header.frame_id = cloudI->header.frame_id;

   pub_cloud_flow.publish(output_cloud);

   

   ///cloud complete

   pub_cloud.publish(cloudI);

   

   ///point normal

    sensor_msgs::PointCloud2 output_cloud2;  

   pcl::concatenateFields(*cloud_p3d,* normals,*cloud_normals);

   //ROS_INFO("cloud info %d %d---->",cloud_normals->height,cloud_normals->width);

   pcl::toROSMsg(*cloud_normals,output_cloud2);

   output_cloud2.header.frame_id = cloudI->header.frame_id;

   pub_cloud_normal.publish(output_cloud2);
   
   ///markers
   
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
  
  
  //////////////////////////////////////////////////
  //publishing
  
//   cv_bridge::CvImagePtr cv_send(new cv_bridge::CvImage);
//   sensor_msgs::PointCloud2 output;
//   //  pcl::toROSMsg<>();
//   pcl::toROSMsg<pcl::PointXYZRGB>(*cloud_rgb,output);
//   //      while (ros::ok())
//   //      {
//     //sensor_msgs::ImagePtr msg_im=sensor_msgs::CvBridge::cvToImgMsg(&ipl_img,  "bgr8");
//   
//  
//   cv_send->image=Mat(im_range_current.rows, im_range_current.cols, CV_8UC3);
//   im_range_current.copyTo(cv_send->image);
//  // cv_send->image.convertTo(cv_send->image, CV_8UC1);
//   pub_flow.publish(cv_send->toImageMsg());
//   pub_cloud.publish(output);
//        
      // ros::spinOnce();
  
    
 }
 void help(){
   
   
   
   ROS_ERROR("Error please try : \n\n"   );
   
   ROS_ERROR("rosrun optical_flow_range optical_flow_range  --file /home/aortega/Experiments/velodyne_32E/camera-calibration/calib1.yaml clouds:=/velodyne_points images:=/camera/image_raw\n"   );
   ROS_ERROR("try with rosbag rosbag play  /home/aortega/Experiments/velodyne_32E/sequences/sequence_me.bag\n"   );
   
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
           
   cout<<"K->\n"<<K_<<endl;
   cout<<"P->\n"<< P_<<endl;
   cout<<"Pext->\n"<< P_ext<<endl;
   ///transformation cloud
    Trans_<<0 ,0 ,-1,
   0, 1 ,0,
   1, 0 ,0;
 }
 
 int main (int argc, char** argv){
   
   
   typedef sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
   typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy_Aprox;
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
     
     ros::NodeHandle nh_; 
     string cloud_topic_;
     
     string image_topic_;
     
     
     
     CloudSubscriber cloud_sub;   
     ImageSubscriber image_sub;
     
     
     ROS_INFO("Registering data");
     
     
     cloud_sub.subscribe(nh_,"clouds",1);
     image_sub.subscribe(nh_ ,"images" ,1);
     
     
     Synchronizer<MySyncPolicy_Aprox> sync(MySyncPolicy_Aprox(10), cloud_sub, image_sub);
     
     sync.registerCallback(boost::bind(&Callback,_1,_2));
     
     
     
      /////////////////////////////////////////////////////

     ///publishers

     ///////////////////////////////////////////////////

     pub_im= nh_.advertise<sensor_msgs::Image>("image_out", 1);

     

     pub_im_flow= nh_.advertise<sensor_msgs::Image>("image_flow", 1);

     pub_im_color_flow=nh_.advertise<sensor_msgs::Image>("image_color_flow", 1);

     pub_cloud_flow= nh_.advertise<sensor_msgs::PointCloud2>("cloud_flow", 1);

     pub_cloud= nh_.advertise<sensor_msgs::PointCloud2>("cloud_complete", 1);

     pub_cloud_normal= nh_.advertise<sensor_msgs::PointCloud2 >("cloud_normal", 1);
     
     pub_markers= nh_.advertise<visualization_msgs::Marker >("Markers", 1);
      
     ros::spin();
     
   }
   return 0;
   
 }
 
 
