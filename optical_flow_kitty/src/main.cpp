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
 #include <visualization_msgs/MarkerArray.h>
 
 using namespace std;
 using namespace pcl;
 using namespace cv;
 using namespace Eigen;
 namespace enc = sensor_msgs::image_encodings;
 ///range image
 boost::shared_ptr<pcl::RangeImage> range_image(new pcl::RangeImage);
 boost::shared_ptr<pcl::RangeImage> range_image_prev(new pcl::RangeImage);
 
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
 
int main(int argc, char **argv) {
  
   char *path;
   
   string dir,filename_vel,file_im_gray1,file_im_gray2,file_im_color1,file_im_color2;
   Mat im_gray1,im_gray2,im_color1,im_color2;
   
   vector<string> files_pcd = vector<string>();
   
   
   
   /////////////////////////////////////////////////////////////////////////
   ///ROS variables
   /////////////////////////////////////////////////////////////////////////
   ros::init(argc, argv, "Node_Kitty_datase");
   ros::NodeHandle nh_; 
   ros::Rate loop_rate(5);
   
   ///Initialize publishers
   ////////////////////////////////////////////////////

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
    pub_arrows_range= nh_.advertise<visualization_msgs::Marker>("Direction_line_range", 1);
   ////////////////////////////////////////////////////

   ///publishers RANGE IMAGE

   ///////////////////////////////////////////////////

   pub_im_range= nh_.advertise<sensor_msgs::Image>("image_range_out", 1);
   pub_im_range_flow= nh_.advertise<sensor_msgs::Image>("image_range_flow", 1);
   pub_im_range_color_flow=nh_.advertise<sensor_msgs::Image>("image_range_color_flow", 1);
   pub_cloud_range_flow= nh_.advertise<sensor_msgs::PointCloud2>("cloud_range_flow", 1);
   pub_cloud_range_normal= nh_.advertise<sensor_msgs::PointCloud2 >("cloud_range_normal", 1);
   pub_range_markers= nh_.advertise<visualization_msgs::Marker >("Markers_range_cloud", 1);
   
   ///principal cloud
   pub_cloud= nh_.advertise<sensor_msgs::PointCloud2>("cloud_complete", 1);
    ///range image

   boost::shared_ptr<pcl::RangeImage> range_image(new pcl::RangeImage); 
   
   ifstream myReadFile;
   string dir_calib,strLine;
   
   
   MatrixXf Tr_velo_to_cam(4,4);
  
  //////////////////////////////////////////////////////////////////////////////////////
  /// read arguments
  ////////////////////////////////////////////////////////////////////////////////////
  if (argc == 3) {
    
    dir = argv[1];
    dir_calib = argv[2];
    
  }
  else{
    ROS_ERROR("Missing argument try ./read_dataset <PATH_DATASET> <PATH_CALIBRATION>\n");
    ROS_ERROR("or  try rosrun optical_flow_kitty  optical_flow_kitty /home/aortega/proyectos/Dynamic_segmentation/data/KITTI_dataset/sec_48/2011_09_26_drive_0048 /home/aortega/proyectos/Dynamic_segmentation/data/KITTI_dataset/sec_48/2011_09_26_calib\n");
    ROS_ERROR("or example : \n");
    ROS_ERROR("rosrun optical_flow_kitty  optical_flow_kitty /home/aortega/proyectos/Dynamic_segmentation/data/KITTI_dataset/sec_71/2011_09_29_drive_0071 /home/aortega/proyectos/Dynamic_segmentation/data/KITTI_dataset/sec_71/2011_09_29_calib\n");
    return(0);
  }
  
  ROS_INFO("Status read paths: \n");
  ROS_INFO("Path selected -> \n");
   
  getdir(dir+"/velodyne_points/data",files_pcd);
  
  //////////////////////////////////////////////////////////
  ///variables for RANGE images
  /////////////////////////////////////////////////////////
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
  float noiseLevel=0.00;
  float minRange = 0.0f;
  int borderSize = 1;
  
  ///POINTCLOUDS variables
  PointCloud<PointXYZRGB>::Ptr cloud_RGB(new PointCloud<PointXYZRGB>);
  
  PointCloud<PointXYZRGB>::Ptr cloud_RGB_flow(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZ>::Ptr cloud_ptr(new PointCloud<PointXYZ> ());
  PointCloud<PointXYZ>::Ptr cloud_previous_ptr(new PointCloud<PointXYZ> ());
  PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr cloud_filtered_prev(new PointCloud<PointXYZ>);
  
  PointCloud<pcl::PointXYZ>::Ptr cloud_flow(new PointCloud<PointXYZ> ());
  PointCloud<pcl::PointXYZ>::Ptr cloud_p3d (new PointCloud<pcl::PointXYZ>);
  PointCloud<pcl::Normal>::Ptr normals (new PointCloud<pcl::Normal>);
  PointCloud<PointXYZ>::Ptr cloud_nonPlanes (new PointCloud<PointXYZ> ());
  PointCloud<PointXYZ>::Ptr cloud_nonPlanes_prev (new PointCloud<PointXYZ> ());
  PointCloud<pcl::PointXYZ>::Ptr cloud_Plane(new PointCloud<PointXYZ> ());
  PointCloud<pcl::PointXYZ>::Ptr cloud_Plane_prev(new PointCloud<PointXYZ> ());
  PointCloud<pcl::PointXYZ>::Ptr cloud_non_floor(new PointCloud<PointXYZ> ());
  PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new PointCloud<pcl::PointXYZ>);
  PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new PointCloud<pcl::PointXYZRGB>);
  PointCloud<pcl::PointNormal>::Ptr cloud_normals (new PointCloud<pcl::PointNormal>);
  PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new PointCloud<pcl::PointXYZ>);
  PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB_im (new PointCloud<pcl::PointXYZRGB>);
  PointCloud<pcl::PointXYZRGB>::Ptr  Clusters(new PointCloud<pcl::PointXYZRGB>);
  PointCloud<pcl::PointXYZRGB>::Ptr  Clusters_plane(new PointCloud<pcl::PointXYZRGB>);
  ///////////////////////////////////////////////////////

   /// point clouds project in image

   ///////////////////////////////////////////////////////

   ///variables

   MatrixXf projected_cloud,  projected_cloud_ext,projected_cloud_ext_prev;
   MatrixXf cloud_on_image;
   MatrixXf projected_cloud_non_floor;
   MatrixXf cloud_on_image_non_floor;
   MatrixXf cloud_in_imagecontour;
   MatrixXf flow_projected;
   MatrixXf flow_3d,P3D;
 
   //////////////////////////////////////////////////////////////////////////////
   /// read calibration velodyne camera
   /////////////////////////////////////////////////////////////////////////////
   
   ////////////////////////////////////////////
   /// read calib cameras
   ////////////////////////////////////////////
   //  - S_xx: 1x2 size of image xx before rectification
   //   - K_xx: 3x3 calibration matrix of camera xx before rectification
   //   - D_xx: 1x5 distortion vector of camera xx before rectification
   //   - R_xx: 3x3 rotation matrix of camera xx (extrinsic)
   //   - T_xx: 3x1 translation vector of camera xx (extrinsic)
   //   - S_rect_xx: 1x2 size of image xx after rectification
   //   - R_rect_xx: 3x3 rectifying rotation to make image planes co-planar
   //   - P_rect_xx: 3x4 projection matrix after rectification
   vector<Matrix3f> K(4);
   vector<Vector2f> S(4);
   vector<Matrix3f> R(4);
   vector<Vector2f> S_rect(4);
   vector<Matrix3f> R_rect(4);
   vector<Vector3f> T(4);
   vector< MatrixXf> P_rect(4);
   vector<VectorXf> D(4);
   
   if( read_calib_kitty(dir_calib,Tr_velo_to_cam,K,S,R,S_rect,R_rect,T,P_rect,D)==0)
   {
     ROS_INFO("Error reading the calibration \n");
   }
   

   
   ///////////////////////////////
   ///new matrices for projection
   ///////////////////////////////////////
   MatrixXf R_cam_to_rect(4,4);
   MatrixXf P_velo_to_img(3,4);
   
 
   
   //sort data
   std::sort(files_pcd.begin(),files_pcd.end());
   
  
   for (unsigned int i = 0;i < files_pcd.size();i++) {
     
     
     ////////////////////////////////////////////////////
     /// read pcd
     //////////////////////////////////////////////////
     if(files_pcd[i].substr(files_pcd[i].find_last_of(".") + 1) == "pcd"){//find pcd files
        
	filename_vel=dir + "/velodyne_points/data/" + files_pcd[i]; //<<----read file
	/////////////////////////////////////////////////////////////////////
	///open pcd READ POINTCLOUD
	//////////////////////////////////////////////////////////////////
	if (io::loadPCDFile (filename_vel.c_str(), *cloud_ptr) == -1)
	{
	  ROS_ERROR( "Was not able to open file %s  \n");
	  return -1;
	}
	
	else {
	  
	  //////////////////////////////////////////////////////////////////////
	  /// read images <----
	  ////////////////////////////////////////////////////////////////////////
	  file_im_gray1=dir + "/image_00/data/" + files_pcd[i].substr(0,files_pcd[i].find_last_of("."))+".png"; 
	  im_gray1 = imread(file_im_gray1,CV_LOAD_IMAGE_GRAYSCALE);
	  
	  file_im_gray2=dir + "/image_01/data/" + files_pcd[i].substr(0,files_pcd[i].find_last_of("."))+".png"; 
	  im_gray2 = imread(file_im_gray2,CV_LOAD_IMAGE_GRAYSCALE); 
	  
	  file_im_color1=dir + "/image_02/data/" + files_pcd[i].substr(0,files_pcd[i].find_last_of("."))+".png"; 
	  im_color1 = imread(file_im_color1,CV_LOAD_IMAGE_COLOR);
	  
	  file_im_color2=dir + "/image_03/data/" + files_pcd[i].substr(0,files_pcd[i].find_last_of("."))+".png"; 
	  im_color2 = imread(file_im_color2,CV_LOAD_IMAGE_COLOR); 
	  
	  ///////////////////////////////////////////////////////////////////////////////////////////////////////
	  ///INFO or received data
	  ///////////////////////////////////////////////////////////////////////////////////////////////////////
	  ROS_INFO("cloud received with %d points ", cloud_ptr->size());
	  ROS_INFO("Images with resolution (%d,%d) ", im_color1.cols,im_color2.rows);
	  
	  /////////////////////////////////////////////////////////////////////////////
	  ///OPTICAL FLOW for RANGE IMAGES
	  ////////////////////////////////////////////////////////////////////////////
	  ///
	  ///segment_data 
	 
	    filter_cloud_kitty(cloud_ptr,cloud_filtered,  cloud_nonPlanes,
               cloud_Plane,
              cloud_projected);
	
	  ///////////////////////////////////////////////////////////////////////////
	  ///image range images
	  //////////////////////////////////////////////////////////////////////////
	  range_image->createFromPointCloud (*cloud_nonPlanes, pcl::deg2rad(0.2f),
					     pcl::deg2rad (360.f), pcl::deg2rad (180.0f),
					     sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	  ///Convert to opencv to work with
	  GenerarImagenOpenCV(range_image,im_range_current, im_range_current_norm);
	  ///just to visualize
	  applyColorMap( im_range_current_norm, falseColorsMap, COLORMAP_HOT);
	  
	  if(!firstFrame)
	  {
	   
	    Mat im2 = Mat(im_range_previous_norm.rows, im_range_previous_norm.cols, im_range_previous_norm.type());
	    cv::resize(im_range_current_norm, im2,im2.size(), INTER_LINEAR );
	    cv::calcOpticalFlowFarneback(im_range_previous_norm,im2,flow,0.5, 3, 15, 3, 5, 1.2, 0);
	    
	    ///show data
	    cv::cvtColor( im_range_previous_norm ,cflow, CV_GRAY2BGR);
	    drawOptFlowMap(flow, cflow, 10, 1.5, Scalar(0, 255, 0));
	    drawOpticalFlow_color(flow,color_flow, -1);
	    
	    ///planar range image TESTING
	    
	    
	    range_image_prev->createFromPointCloud (*cloud_nonPlanes_prev , pcl::deg2rad(0.2f),
						    pcl::deg2rad (360.f), pcl::deg2rad (180.0f),
						    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	    
	    ROS_INFO("Resolution Range image (%d %d)", range_image->width,range_image->height);
	    ROS_INFO("Resolution Range image (%d %d)", range_image_prev->width,range_image_prev->height);
	    // 	     cout<<"pass here"<<endl;
	    Set_range_flow(range_image,range_image_prev,im_range_previous_norm,flow,cloud_flow,cloud_p3d,normals,cloud_rgb);
	    
	    ///////////////////////////////////////////////////////////////////////////////
	    /// Publishing Range data
	    //////////////////////////////////////////////////////////////////////////////
	    
	    Publish_Range_Data(cflow, 
			       color_flow,
			falseColorsMap,  
			cloud_previous_ptr,///changed
			cloud_rgb,
			normals,
			cloud_normals,
			cloud_p3d,
			cloud_flow,
			Clusters,
			pub_im_range,
			pub_im_range_flow,
			pub_im_range_color_flow,
			pub_cloud,
			pub_cloud_range_flow,
			pub_cloud_range_normal,
			pub_range_markers,
			 pub_clusters_range,
			pub_arrows_range
			      );
	    
	    loop_rate.sleep();
	    
	 
	     
	    
	    cloud_flow->clear();
	    cloud_p3d->clear();
	    cloud_rgb->clear();
	    normals->clear();
	    cloud_normals->clear();
	    cloud_filtered->clear();
	    cloud_RGB_flow->clear();
	    cloud_normals->clear();
	    cloud_flow->clear();
	    cloud_p3d->clear();
	    cloud_filtered->clear();
	    normals->clear();
	    cloud_RGB_im->clear();
	    Clusters->clear();
	    Clusters_plane->clear();
	  }
	  
	  //create the images
	  
	  firstFrame=0;// we have read the first frame
	  
	  im_range_current_norm.copyTo(im_range_previous_norm);
	  
	  flow=Mat(im_range_current_norm.rows, im_range_current_norm.cols,CV_32FC2);
	  cflow=Mat(im_range_current_norm.rows, im_range_current_norm.cols, CV_8UC3);
	  
	
	  ////////////////////////////////////////////////////////////////
	  ///Image optical flow
	  ////////////////////////////////////////////////////////////////
	  cvtColor(im_color1, im_current_norm, CV_RGB2GRAY);
	  ///cvtColor(im_color2, im_current_norm, CV_RGB2GRAY);<<--- we chose the firt color image
	  im_color1.copyTo(im_current_color);
	  
	  
	  if(!im_firstFrame)

	  {

	    ///IMPORTAN optical FLOW is gray scale
            ///YOU HAVE TO compute PREVIOUS less CURRENT
	    cv::calcOpticalFlowFarneback(im_previous_norm,im_current_norm,im_flow,0.5, 3, 15, 3, 5, 1.2, 0);
 
	    //////////////////////////////////////////////////////////////////////////////////////////   
	    ///here change the cloud filtered if dont want to include the segmented data
	    /////////////////////////////////////////////////////////////////////////////////////////
	    
	    
	    ////////////////////////////////////////////
	    ///project points     
	    //////////////////////////////////////////////
	    // 	    //NOTE N_camara is the cara of 4 
	    ///0 ---> im_gray1
	    /// 1 --->im_gray2
	    /// 2 --->im_color1
	    /// 3 --->im_color2 
	    int N_camara=2;
	 
	    
	    ///proyect the current point clouds 
// 	   Eigen::MatrixXf cloud_2= cloud_nonPlanes->getMatrixXfMap();///<------change the pointcloud NOT consider FLOOR
// 	   Eigen::MatrixXf  cloud_1= cloud_nonPlanes->getMatrixXfMap();
// 	  /* 
// 	   project_cloud2image_kitty(cloud_1, cloud_2, im_current_color, R_rect,P_rect,
// 			   Tr_velo_to_cam,Trans_,projected_cloud,projected_cloud_ext, 
// 			  cloud_on_image, projected_cloud_non_floor, 
// 			  cloud_on_image_non_floor,cloud_RGB, N_camara,P_ext);
// 	      */
	   /// we asing the actual cloud with the before.     
	    Eigen::MatrixXf  cloud_2= cloud_nonPlanes_prev->getMatrixXfMap();///<------change the pointcloud NOT consider FLOOR
	    Eigen::MatrixXf  cloud_1= cloud_nonPlanes_prev->getMatrixXfMap();
	      ROS_INFO("PassHERe ");
	    
	    // 	   cloud_RGB->clear();
	    // 		     
	    Trans_<<1,0,0,
	    0,1,0,
	    0,0,1;
	    /// project the previus 3d with the privous image 
	    ///then we get the point in 2d
	    project_cloud2image_kitty(cloud_1, cloud_2, im_previous_color, R_rect,P_rect,
				      Tr_velo_to_cam,Trans_,projected_cloud,projected_cloud_ext, 
			       cloud_on_image, projected_cloud_non_floor, 
			       cloud_on_image_non_floor,cloud_RGB, N_camara,P_ext);
	      ROS_INFO("PassHERe ");
	    ///NOTE DEBUGING

	    
	    ///////////////////////////////////////////////////////////////////
	    /// recover 3d point and flow in 3d
	    //////////////////////////////////////////////////////////////       
	    ///HERE we have to CHANGE for have the reak 3d point
	    ///THIS PART is WORKING WELL
	    //Set_projected_flow(projected_cloud_ext_prev,
		//		im_flow,flow_projected);
// 	     ROS_INFO("previous (%d %d)", projected_cloud_ext_prev.rows(), projected_cloud_ext_prev.cols());
// 	     ROS_INFO("current (%d %d)", projected_cloud_ext.rows(), projected_cloud_ext.cols());
	     
	     Set_projected_flow( projected_cloud_ext,
			im_flow,flow_projected);
	      ROS_INFO("Pass--HERe ");
	    ////////////////////////////////////////////
           ///transform optical flow to 3d
	   ////////////////////////////////////////////
	     Set_flow2d23d_kitty(projected_cloud_ext,
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
	     
	     ROS_INFO("Point normals size %d ", cloud_flow->size());
	     ROS_INFO("Point 3d size %d ", cloud_p3d->size());
	     ROS_INFO("Point rgb %d ", cloud_RGB_im->size());
	    
	    ////////////////////////////////////////////////////////////////////////
	    ///Put flow on images flow
	    ///////////////////////////////////////////////////////////////////////
	    cv::cvtColor(im_previous_norm ,im_cflow, CV_GRAY2BGR);
	    drawOptFlowMap(im_flow, im_cflow, 10, 1.5, Scalar(0, 255, 0));
	    drawOpticalFlow_color(im_flow,im_color_flow, -1);

	    ///Comment just for paper draft
// 	      string path="/home/aortega/Desktop/kitti_out/";
// 	     stringstream ss;
// 	     ss << i;
//             ///save images
// 	     imwrite( path+"out_"+ss.str()+".jpg", im_previous_color);
// 	     imwrite( path+"out_flow"+ss.str()+".jpg", im_cflow);
// 	     imwrite( path+"out_pub_im_range"+ss.str()+".jpg", im_range_previous_norm);
// 	     imwrite( path+"out_pub_range_flow"+ss.str()+".jpg", cflow);
	    
	    /////////////////////////////////////////////////////////////////
	    ///publishing for images flow
	    //////////////////////////////////////////////////////////////////
            ///
	    ///Current image
	    cv_bridge::CvImagePtr  cv_send(new cv_bridge::CvImage);
	    cv_send->image=Mat(im_previous_norm.rows, im_previous_norm.cols, CV_8UC3);
	    cv_send->encoding=enc::RGB8;
	    im_previous_color_aux.copyTo(cv_send->image);
	    pub_im.publish(cv_send->toImageMsg());
	    
// 	    cv_send->image=Mat(im_current_norm.rows, im_current_norm.cols, CV_8UC3);
// 	    cv_send->encoding=enc::RGB8;
	    ///image flow

	    im_cflow.copyTo(cv_send->image);
	    pub_im_flow.publish(cv_send->toImageMsg());

	    ///image color
	    im_color_flow.copyTo(cv_send->image);
	    pub_im_color_flow.publish(cv_send->toImageMsg());
	    
// 	    ///////////////////////////////////////////////////////////
// 	    ///clouds 
// 	    ////////////////////////////////////////////////////////////
	    sensor_msgs::PointCloud2 output_cloud;

	    ///cloud flow
	    pcl::toROSMsg(*cloud_RGB_flow,output_cloud);
	    output_cloud.header.frame_id = "/velodyne";
	    pub_cloud_im_flow.publish(output_cloud);
           
// 	    
// 	    
	    ///point normal

	    sensor_msgs::PointCloud2 output_cloud2;  
	    pcl::concatenateFields(*cloud_p3d,* normals,*cloud_normals);
	    
// 	    ///
// 	    
// 	    ROS_INFO("cloud clusters planes---> received with %d points ",Clusters_plane->size());

	    //ROS_INFO("cloud info %d %d---->",cloud_normals->height,cloud_normals->width);
            ///publish normals
	    pcl::toROSMsg(*cloud_normals,output_cloud2);
	    output_cloud2.header.frame_id ="/velodyne";
	    pub_cloud_im_normal.publish(output_cloud2);

	    
	    ///cloud rgb of image
	    pcl::toROSMsg(*cloud_RGB_im,output_cloud2);

	    output_cloud2.header.frame_id ="/velodyne";
	    pub_cloud_rgb.publish(output_cloud2);
	    
	    ///publish NON PLANES 
	     pcl::toROSMsg(*cloud_nonPlanes_prev,output_cloud2);
	    output_cloud2.header.frame_id ="/velodyne";
	    pub_non_planes.publish(output_cloud2);
	    
	    ///publish PLANES ---NOTE
	    pcl::toROSMsg(*cloud_Plane,output_cloud2);
	    output_cloud2.header.frame_id ="/velodyne";
	    pub_planes.publish(output_cloud2);
// 	    
	    ///publish markers
	    visualization_msgs::Marker points,line_list;

	    Publish_Marks(cloud_flow,cloud_p3d, points,line_list,1);
	 //   pub_im_markers.publish(points); NOTE DONT SHOW POINTS
	    pub_im_markers.publish(line_list);
	    
	    
	    ///NOTE comment the data 
	    //*//CLUSTERS IMAGE
	    ///////////////////////////////////////////////
	    ///clusters normal 
	   /* 
	    visualization_msgs::Marker points2,lines2;
	    visualization_msgs::MarkerArray arrows;
            cluster_pointcloud(cloud_normals,  Clusters,points,points2,lines2,arrows);
	    ROS_INFO("cloud clusters ---> received with %d points ",  Clusters->size());
	    
	    pcl::toROSMsg(*Clusters,output_cloud2);
	    output_cloud2.header.frame_id ="/velodyne";
	    pub_clusters_image.publish(output_cloud2);
	    
	    
	   
	    
	     pub_arrows_image.publish(points2);///DONE after put a new publiser for the centers
	     pub_arrows_image.publish(lines2);///DONE directions*/
	     //  pub_arrows.publish(arrows);
	    
	  }
	  /////////////////////////////////////////////////////////////
	  ///create the images for optical flow the first time
	  ////////////////////////////////////////////////////////////
	  
	  
	  im_firstFrame=0;// we have read the first frame 
	  im_previous_norm=Mat(im_current_norm.rows, im_current_norm.cols,im_current_norm.type());
	  im_current_norm.copyTo(im_previous_norm);
	  im_flow=Mat(im_current_norm.rows, im_current_norm.cols,CV_32FC2);
	  im_cflow=Mat(im_current_norm.rows, im_current_norm.cols, CV_8UC3);
	  im_current_color.copyTo(im_previous_color);
	  im_current_color.copyTo(im_previous_color_aux);
	  
	  
	  pcl::copyPointCloud<PointXYZ>(*cloud_filtered, *cloud_filtered_prev);
	  pcl::copyPointCloud<PointXYZ>(*cloud_nonPlanes,* cloud_nonPlanes_prev );
	  pcl::copyPointCloud<PointXYZ>(*cloud_ptr,*cloud_previous_ptr);
	  pcl::copyPointCloud<PointXYZ>(*cloud_Plane,*cloud_Plane_prev);
	  
	  cloud_filtered->clear();
	  cloud_RGB_flow->clear();
	  cloud_normals->clear();
	  cloud_flow->clear();
	  cloud_p3d->clear();
	  cloud_filtered->clear();
	  normals->clear();
	  cloud_RGB_im->clear();
	  Clusters->clear();
	  Clusters_plane->clear();
	  cloud_nonPlanes->clear();
	  cloud_Plane->clear();
	//  cloud_nonPlanes_prev->clear();
	  cloud_ptr->clear();
	 // cloud_Plane_prev->clear();
	  
	}
	
	
     }
   }
   
   
   
  
  return 0;
}
