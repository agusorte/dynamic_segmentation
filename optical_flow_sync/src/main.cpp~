 #include <string>
 #include <iostream>
 #include <rosbag/bag.h>
 #include <rosbag/view.h>
 #include "ros/ros.h"
 #include <ros/names.h>
 #include <pcl_ros/point_cloud.h>
 
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
 // PCL specific includes
 #include <pcl/ros/conversions.h>
 #include <pcl/point_cloud.h>
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
 #include <visualization_msgs/Marker.h>
 #include "image_utilities.h"
 
 ///EIGEN
 
 #include <Eigen/Dense>
 
 
 
 using namespace pcl;
 using namespace std;
 using namespace Eigen;
 using namespace message_filters;
 
 using namespace cv;
 namespace enc = sensor_msgs::image_encodings;
 
 
 ///range image

 boost::shared_ptr<pcl::RangeImage> range_image(new pcl::RangeImage);
 
 ///////////////////////////////////////////////////

 /// subscribers 
 /////////////////////////////////////////////////

 typedef message_filters::Subscriber< sensor_msgs::Image > ImageSubscriber;
 typedef message_filters::Subscriber<sensor_msgs::PointCloud2 > CloudSubscriber;
 
 //////////////////////////////////////////////////////////////////////////////////////
 ///publishers
 /////////////////////////////////////////////////////////////////////////////////////
///FOR IMAGES

 ros::Publisher pub_im_flow,pub_im_color_flow;

 ros::Publisher pub_im;

 ros::Publisher pub_cloud_im_flow,pub_cloud_im_normal,pub_cloud,pub_cloud_rgb;//<--this is for the real cloud

 ros::Publisher pub_im_markers;

 ///FOR RANGE IMAGES

 /////////////////////////////////////////////////////



 ros::Publisher pub_im_range_flow,pub_im_range_color_flow;

 ros::Publisher pub_im_range;

 ros::Publisher pub_cloud_range_flow,pub_cloud_range_normal;

 ros::Publisher pub_range_markers;

 ros::Publisher pub_arrows_image;

 ros::Publisher pub_arrows_range;

 

 ///publisher rgb  clusters

 ros::Publisher pub_clusters_image;

 ros::Publisher pub_clusters_range;

  ros::Publisher pub_planes;

  ros::Publisher pub_non_planes;

 ////////////////////////////////////////////////////////////////////////////////////
 ///calibration parameters
 ////////////////////////////////////////////////////////////////////////////////////
 string file;
 Matrix3f K_;
 MatrixXf P_;
 MatrixXf P_ext;
 Matrix3f Trans_;
 Matrix3f R_;
 Vector3f T_;
 /////////////////////////////////////////////////////////////////////////
 ///images for optical flow
 ////////////////////////////////////////////////////////////////////////
 bool im_firstFrame=1;
  Mat im_current;

 Mat im_previous;

 Mat im_current_norm;

 Mat im_previous_norm;

 Mat im_flow,im_cflow,im_color_flow;

 Mat im_falseColorsMap;

 Mat im_current_color;

 Mat im_previous_color;

 Mat im_previous_color_aux;
 
 ////////////////////////////////////////////////////////////////////////
 ///images range  flow
 ////////////////////////////////////////////////////////////////////////
 bool firstFrame=1;

 Mat im_range_current;

 Mat im_range_previous;

 Mat im_range_current_norm;

 Mat im_range_previous_norm;

 Mat flow,cflow,color_flow;

 Mat falseColorsMap;
 
 
 ///////////////////////////////////////////////////////////////////
 ///Syncronize objects cloud projected and cloud in image
 //////////////////////////////////////////////////////////////////
 void Callback(const sensor_msgs::PointCloud2ConstPtr& cloudI,
	       const sensor_msgs::ImageConstPtr& msg)
 {
   ROS_INFO("IMAGE timestamps %i.%i ", msg->header.stamp.sec,msg->header.stamp.nsec);
   ROS_INFO("cloud timestamps %i.%i ", cloudI->header.stamp.sec,cloudI->header.stamp.nsec);

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
     
     cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
     
     ROS_INFO("Image data size (%d, %d)",cv_ptr->image.rows, cv_ptr->image.cols);
   }
   catch (cv_bridge::Exception& e)   
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());     
     return ;    
   }
   
   
   ///////////////////////////////////////////////////////
   /// point clouds project in image
   ///////////////////////////////////////////////////////
   ///variables
   MatrixXf projected_cloud,  projected_cloud_ext;
   MatrixXf cloud_on_image;
   MatrixXf projected_cloud_non_floor;
   MatrixXf cloud_on_image_non_floor;
   MatrixXf cloud_in_imagecontour;
   MatrixXf flow_projected;
   MatrixXf flow_3d,P3D;
   
   ///////////////////////////////////////////////////////////////////////
   ///cloud variables 
   //////////////////////////////////////////////////////////////////////
   PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
   PointCloud<PointXYZRGB>::Ptr cloud_RGB(new PointCloud<PointXYZRGB>);
   PointCloud<PointXYZRGB>::Ptr cloud_RGB_flow(new PointCloud<PointXYZRGB>);
   
   /////////////////////////////////////////////////////////////
   ///find planes and non planes
   //////////////////////////////////////////////////////
   pcl::PointCloud<PointXYZ>::Ptr cloud_nonPlanes (new PointCloud<PointXYZ> ());     
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Plane(new PointCloud<PointXYZ> ());
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new PointCloud<pcl::PointXYZ>);
   
   ///For flow
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_flow(new PointCloud<PointXYZ> ());
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p3d(new PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::Normal>::Ptr normals (new PointCloud<pcl::Normal>);
   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new PointCloud<pcl::PointNormal>);
   PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new PointCloud<pcl::PointXYZRGB>);
   
   /////////////////////////////////////////////////////////////////////////
   ///for debbuging transformin pointclouds
   PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new PointCloud<pcl::PointXYZ>);
  PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB_im (new PointCloud<pcl::PointXYZRGB>);
  PointCloud<pcl::PointXYZRGB>::Ptr  Clusters(new PointCloud<pcl::PointXYZRGB>);
  PointCloud<pcl::PointXYZRGB>::Ptr  Clusters_plane(new PointCloud<pcl::PointXYZRGB>);
     
   
   /////////////////////////////////////////////////////////////////////////////
   ///IMAGE FOR OPTICAL FLOW
   /////////////////////////////////////////////////////////////////////////////
   ROS_INFO("IMAGE FLOW ------------\n\n");
   Mat im_current=cv_ptr->image.clone();
   im_current.copyTo( im_current_color);//eliminate to depure
   //im_current_color_aux.copyTo( im_current_color);//eliminate to depure
   cvtColor(im_current, im_current_norm, CV_RGB2GRAY);
   
   if(!im_firstFrame)
   {
     
     ///IMPORTAN optical FLOW is gray scale
     cv::calcOpticalFlowFarneback(im_previous_norm,im_current_norm,im_flow,0.5, 3, 15, 3, 5, 1.2, 0);
     
     //////////////////////////////////////////////////
     ///segment data 
     ///////////////////////////////////////////////////
     PCL_INFO("Segmenting data \n");
    SegmentData(cloud_ptr,cloud_filtered,cloud_nonPlanes,cloud_Plane,cloud_projected);
     
     //////////////////////////////////////////////////////////////////////////////////////////   
     ///here change the cloud filtered if dont want to include the segmented data
     //Eigen::MatrixXf cloud_1=cloud_filtered->getMatrixXfMap();<<<-------------------------
     
     Eigen::MatrixXf cloud_1= cloud_filtered->getMatrixXfMap();
     Eigen::MatrixXf cloud_2= cloud_nonPlanes->getMatrixXfMap();//non planes
     
     ////////////////////////////////////////////
     ///project points
     PCL_INFO("Project  data \n");

     project_cloud2image( cloud_1,cloud_2,im_previous_color,Trans_,P_,projected_cloud, projected_cloud_ext,			  
			  cloud_on_image,projected_cloud_non_floor, cloud_on_image_non_floor,cloud_RGB);
      
     ///////////////////////////////////////////////////////////////////
     /// recover 3d point and flow in 3d
     //////////////////////////////////////////////////////////////
     PCL_INFO("Recover 3d point and flow in 3d \n");
     Set_projected_flow(projected_cloud_ext,im_flow,flow_projected);
      
     ///DEBUG WORKING well
//      Set_flow2d23d(projected_cloud_ext,im_flow,flow_projected, P_ext,Trans_,
// 		   flow_3d,P3D,cloud_flow,cloud_p3d,normals,cloud_RGB_flow);
     
//      
      ////////////////////////////////////////////
           ///transform optical flow to 3d
	   ////////////////////////////////////////////
       Set_flow2d23d(projected_cloud_ext,
				 im_flow,
			  im_previous_color_aux,
			  flow_projected, 
			  P_ext,
			  flow_3d,
			  P3D,
			  cloud_flow,
			  cloud_p3d,
			  normals,
			  cloud_RGB_flow,
			  cloud_RGB_im);
// 	     
     ////////////////////////////////////////////////////////////////////////
     ///Put flow on images flow
     ///////////////////////////////////////////////////////////////////////
     PCL_INFO("flow on images \n");
     cv::cvtColor(im_current_norm ,im_cflow, CV_GRAY2BGR);
     drawOptFlowMap(im_flow, im_cflow, 10, 1.5, Scalar(0, 255, 0));
     drawOpticalFlow_color(im_flow,im_color_flow, -1);
     
     
     /////////////////////////////////////////////////////////////////
     ///publishing for images flow
     //////////////////////////////////////////////////////////////////
     ///Current image
     
     PCL_INFO("Publish data \n");
     
     pub_im.publish(cv_ptr->toImageMsg());
     
     cv_bridge::CvImagePtr  cv_send(new cv_bridge::CvImage);
     cv_send->image=Mat(im_previous_norm.rows, im_previous_norm.cols, CV_8UC3);
     cv_send->encoding=enc::RGB8;
     im_previous_color.copyTo(cv_send->image);
     pub_im.publish(cv_send->toImageMsg());
     
     // 	    cv_send->image=Mat(im_current_norm.rows, im_current_norm.cols, CV_8UC3);
     // 	    cv_send->encoding=enc::RGB8;
     ///image flow
     
     im_cflow.copyTo(cv_send->image);
     pub_im_flow.publish(cv_send->toImageMsg());
     
     ///image color
     im_color_flow.copyTo(cv_send->image);
     pub_im_color_flow.publish(cv_send->toImageMsg());
     
     ///////////////////////////////////////////////////////////
     ///clouds 
     ////////////////////////////////////////////////////////////
     ///////////////////////////////////////////////////////////
     sensor_msgs::PointCloud2 output_cloud;
     
     ///complete cloud
     
     
     pcl::toROSMsg(*cloud_ptr,output_cloud);
     output_cloud.header.frame_id = "/teo/velodyne";
     pub_cloud.publish(output_cloud);
   
     
     ///cloud flow
     
     pcl::toROSMsg(*cloud_RGB_flow,output_cloud);
     output_cloud.header.frame_id =" /teo/velodyne";
     pub_cloud_im_flow.publish(output_cloud);
	    
     
     ///point normal
     
     
     
     sensor_msgs::PointCloud2 output_cloud2; 
     pcl::concatenateFields(*cloud_p3d,* normals,*cloud_normals);
     
     
     
     // 	    ///
     
     // 	    
     
     // 	    ROS_INFO("cloud clusters planes---> received with %d points ",Clusters_plane->size());
     
     
     
     //ROS_INFO("cloud info %d %d---->",cloud_normals->height,cloud_normals->width);
     
     ///publish normals    
     pcl::toROSMsg(*cloud_normals,output_cloud2);
     output_cloud2.header.frame_id ="/teo/velodyne";
     pub_cloud_im_normal.publish(output_cloud2);
     

     ///cloud rgb of image
     
     pcl::toROSMsg(*cloud_RGB_im,output_cloud2);
     output_cloud2.header.frame_id ="/teo/velodyne";
     pub_cloud_rgb.publish(output_cloud2);
     
     
     
     ///publish NON PLANES 
     
     pcl::toROSMsg(*cloud_filtered,output_cloud2);
     output_cloud2.header.frame_id ="/teo/velodyne";
     pub_non_planes.publish(output_cloud2);
     
     
     
     ///publish PLANES ---NOTE
     
     pcl::toROSMsg(*cloud_Plane,output_cloud2);
     output_cloud2.header.frame_id ="/teo/velodyne";
     pub_planes.publish(output_cloud2);
     
     // 	    
     
     ///publish markers
     
     visualization_msgs::Marker points,line_list;
 
     Publish_Marks(cloud_flow,cloud_p3d, points,line_list,1);
     
     //   pub_im_markers.publish(points); NOTE DONT SHOW POINTS
     
     pub_im_markers.publish(line_list);
     
     ///publish markers
     // visualization_msgs::Marker points,line_list;
     //      Publish_Marks(cloud_flow,cloud_p3d, points,line_list);
     //      pub_im_markers.publish(points);
     //      pub_im_markers.publish(line_list);
     
     visualization_msgs::Marker points2,lines2;
     visualization_msgs::MarkerArray arrows;
     cluster_pointcloud(cloud_normals,  Clusters,points,points2,lines2,arrows);
     ROS_INFO("cloud clusters ---> received with %d points ",  Clusters->size());
     
     pcl::toROSMsg(*Clusters,output_cloud2);
     output_cloud2.header.frame_id ="/teo/velodyne";
     pub_clusters_image.publish(output_cloud2);
     
     pub_arrows_image.publish(points2);///DONE after put a new publiser for the centers
     pub_arrows_image.publish(lines2);///DONE directions*/
	     //  pub_arrows.publish(arrows);
   }
   ///////////////////////////////////////////////////////////////
   ///create the images for optical flow the first time
   //////////////////////////////////////////////////////////////
   
   im_firstFrame=0;// we have read the first frame 
   im_previous_norm=Mat(im_current_norm.rows, im_current_norm.cols,im_current_norm.type());

   im_current_norm.copyTo(im_previous_norm);

   im_flow=Mat(im_current_norm.rows, im_current_norm.cols,CV_32FC2);

   im_cflow=Mat(im_current_norm.rows, im_current_norm.cols, CV_8UC3);

   im_current_color.copyTo(im_previous_color);

   im_current_color.copyTo(im_previous_color_aux);
   
//    /////////////////////////////////////////////////////////////
//    ///RANGE FLOW
//    ////////////////////////////////////////////////////////////
//    ROS_INFO("RANGE FLOW ------------\n\n");
//    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
// 
//    
//    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
// 
//    
//    float noiseLevel=0.00;
// 
//    
//    float minRange = 0.0f;
// 
//    
//    int borderSize = 1;
// 
//    
//    ///segment data only the part in front of
//    SegmentData(cloud_ptr,cloud_filtered,cloud_nonPlanes,cloud_Plane,cloud_projected);
//    
//    ///////////////////////////////////////////////
// 
//    ///image range images
// 
//    //////////////////////////////////////////////
// 
//    range_image->createFromPointCloud (*cloud_filtered, pcl::deg2rad(0.2f),
// 				      pcl::deg2rad (360.f), pcl::deg2rad (180.0f),
// 				      sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
//    
//    ///create image opencv to work with
//    GenerarImagenOpenCV(range_image,im_range_current, im_range_current_norm);
//    
//    ///apply color to the range image
//    applyColorMap( im_range_current_norm, falseColorsMap, COLORMAP_HOT);
//    
//    if(!firstFrame)
//    {
//      
//      Mat im2 = Mat(im_range_previous_norm.rows, im_range_previous_norm.cols, im_range_previous_norm.type());
//      cv::resize(im_range_current_norm, im2,im2.size(), INTER_LINEAR );
//      cv::calcOpticalFlowFarneback(im2,im_range_previous_norm,flow,0.5, 3, 15, 3, 5, 1.2, 0);
//      
//      /////////////////////////////////////////////////////////////////
//      ///show data put optical flow
//      ///////////////////////////////////////////////////////////////
//      cv::cvtColor( im2 ,cflow, CV_GRAY2BGR);
// 
//      drawOptFlowMap(flow, cflow, 10, 1.5, Scalar(0, 255, 0));
//      drawOpticalFlow_color(flow,color_flow, -1);
// 
//      
//      Set_range_flow(range_image,im2,flow,cloud_flow,cloud_p3d,normals,cloud_rgb);
//      ///planar range image TESTING for planar images
//      // Set_range_flow(range_image_planar,im2,flow,cloud_flow,cloud_p3d,normals,cloud_rgb);
//      
//      ////////////////////////////////////////////////////////////////////////////////////
// 
//      ///publishing
// 
//      //////////////////////////////////////////////////////////////////////////////////
// 
//      cv_bridge::CvImagePtr  cv_send(new cv_bridge::CvImage);
//      cv_send->image=Mat(falseColorsMap.rows, falseColorsMap.cols, CV_8UC3);
//      cv_send->encoding=enc::RGB8;
//      
//      ///range image 
//      falseColorsMap.copyTo(cv_send->image);
// 
//      pub_im_range.publish(cv_send->toImageMsg());
//      ///flow arrows for range image
//      cv_send->image=Mat(cflow.rows, cflow.cols, CV_8UC3);
//      cflow.copyTo(cv_send->image);
// 
//      pub_im_range_flow.publish(cv_send->toImageMsg());
//      ///color flow
//      
//      cv_send->image=Mat( color_flow.rows,  color_flow.cols, CV_8UC3);
//      color_flow.copyTo(cv_send->image);
//      
//      pub_im_range_color_flow.publish(cv_send->toImageMsg());
//      
//      ////////////////////////////////////////////////////////////////
//      ///cloud flow
//      ////////////////////////////////////////////////////////////////
//      sensor_msgs::PointCloud2 output_cloud;
//      pcl::toROSMsg(*cloud_rgb,output_cloud);
//      output_cloud.header.frame_id = cloudI->header.frame_id;
//      pub_cloud_range_flow.publish(output_cloud);
//      
//      //////////////////////////////////////////////////////////////
//      ///point normal
// 
//      ///////////////////////////////////////////////////////////////
//      sensor_msgs::PointCloud2 output_cloud2;  
//      pcl::concatenateFields(*cloud_p3d,* normals,*cloud_normals);
//      
//      pcl::toROSMsg(*cloud_normals,output_cloud2);
//      output_cloud2.header.frame_id = cloudI->header.frame_id;
//      pub_cloud_range_normal.publish(output_cloud2);
//      
//      //////////////////////////////////////////////////////////////
// 
//      ///markers
// 
//      ///////////////////////////////////////////////////////////
// 
//      visualization_msgs::Marker points,line_list;
//      Publish_Marks(cloud_flow,cloud_p3d, points,line_list);
//      pub_range_markers.publish(points);
//      pub_range_markers.publish(line_list);
//      
//    }
//    
//    //create the images
// 
//    firstFrame=0;// we have read the first frame
// 
//    
//    im_range_current_norm.copyTo(im_range_previous_norm);
// 
//    flow=Mat(im_range_current_norm.rows, im_range_current_norm.cols,CV_32FC2);
// 
//    cflow=Mat(im_range_current_norm.rows, im_range_current_norm.cols, CV_8UC3);
   
 }
 void help(){
   
   ROS_ERROR("Error please try : \n\n"   );
   ROS_ERROR("rosrun optical_flow_sync optical_flow_sync  --file /home/aortega/Experiments/velodyne_32E/camera-calibration/calib1.yaml clouds:=/velodyne_points images:=/camera/image_raw\n"   );
   ROS_ERROR("rosrun optical_flow_sync optical_flow_sync --file /home/aortega/Experiments/calibration_iri_velodyne/exp2/laser_calib_left.yaml clouds:=/teo/velodyne_points images:=/teo/sensors/stereo/left/image_raw\n"   );

   ROS_ERROR("try with:-----> rosbag play  /home/aortega/Experiments/velodyne_32E/sequences/sequence_me.bag\n"   );
   
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
     return (-1);
   else
   {
     //////////////////////////////////////////////////
     /// read calibration
     //////////////////////////////////////////////////
     read_calib();
     
     
     ros::init(argc, argv, "Syncronization_node_optica_flow");
     
     ros::NodeHandle nh_; 
     
     CloudSubscriber cloud_sub;
     ImageSubscriber image_sub;
     
     
     
     cloud_sub.subscribe(nh_,"clouds",1);
     image_sub.subscribe(nh_ ,"images" ,1);
     
     ///////////////////////////////////////////////////////
     ///Syncronize topics
     //////////////////////////////////////////////////////////////////////////////
     Synchronizer<MySyncPolicy_Aprox> sync(MySyncPolicy_Aprox(10), cloud_sub, image_sub);
     sync.registerCallback(boost::bind(&Callback,_1,_2));
     
     /////////////////////////////////////////////////////
     ///publishers IMAGE
     ///////////////////////////////////////////////////
      pub_im= nh_.advertise<sensor_msgs::Image>("image_out", 1);  

   pub_im_flow= nh_.advertise<sensor_msgs::Image>("image_flow", 1);

   pub_im_color_flow=nh_.advertise<sensor_msgs::Image>("image_color_flow", 1);

   pub_cloud_im_flow= nh_.advertise<sensor_msgs::PointCloud2>("cloud_image_flow", 1);

   pub_cloud= nh_.advertise<sensor_msgs::PointCloud2>("cloud_complete", 1);

   pub_cloud_rgb= nh_.advertise<sensor_msgs::PointCloud2>("cloud_image_rgb", 1);

   pub_cloud_im_normal= nh_.advertise<sensor_msgs::PointCloud2 >("cloud_normal_image", 1);  

   pub_im_markers= nh_.advertise<visualization_msgs::Marker >("Markers_image_cloud", 1);
     
      ///pubclusters
   pub_clusters_image= nh_.advertise<sensor_msgs::PointCloud2 >("cloud_clusters_image", 1);
   pub_clusters_range= nh_.advertise<sensor_msgs::PointCloud2 >("cloud_clusters_range", 1);
   pub_planes= nh_.advertise<sensor_msgs::PointCloud2 >("cloud_planes", 1);
   pub_non_planes= nh_.advertise<sensor_msgs::PointCloud2 >("cloud_non_planes", 1);
   pub_arrows_image= nh_.advertise<visualization_msgs::Marker>("Direction_line_image", 1);
    //pub_arrows_range= nh_.advertise<visualization_msgs::Marker>("Direction_line_range", 1);
     
     ////////////////////////////////////////////////////
     //*/*/*//publishers RANGE IMAGE
     ///////////////////////////////////////////////////
//      pub_im_range= nh_.advertise<sensor_msgs::Image>("image_range_out", 1);
// 
//      pub_im_range_flow= nh_.advertise<sensor_msgs::Image>("image_range_flow", 1);
// 
//      pub_im_range_color_flow=nh_.advertise<sensor_msgs::Image>("image_range_color_flow", 1);
// 
//      pub_cloud_range_flow= nh_.advertise<sensor_msgs::PointCloud2>("cloud_range_flow", 1);
// 
//      pub_cloud_range_normal= nh_.advertise<sensor_msgs::PointCloud2 >("cloud_range_normal", 1);
// 
//      pub_range_markers= nh_.advertise<visualization_msgs::Marker >("Markers_range_cloud", 1);*/*/*/
      ros::spin();
     
   }
   return 0;
   
 }
 
 