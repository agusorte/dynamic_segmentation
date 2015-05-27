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

  #include "Utilities.h"

  #include "viewer_data.h"
  #include "image_data.h"

  namespace enc = sensor_msgs::image_encodings;
  using namespace cv;
  
  class manual_calib{
    
  protected:
    
    ros::NodeHandle nh_;
    
  private:
    std::string prefix_;
    
  public:
    string cloud_topic_;
    string image_topic_;
    
    ros::Subscriber sub_;
    
    ros::V_Subscriber subs_;
    Matrix3f K;
    MatrixXf P;
  
    string selection_;
    string method_;
   

    Mat image1; //this image for debugging
    
    MatrixXf x3d;//3d points
    MatrixXf x2d;//2d points
    
    MatrixXf R;// rotation

    Vector3f t;// translation

    MatrixXf xcam_; // points of the pose

    
     // class data 3d
    viewer_data objviewer_data;
    
    image_data obj_img_data;
    
    /// cloud global
    PointCloud<PointXYZ>::Ptr cloud_ ;//(new PointCloud<PointXYZ>);
    ////////////////////////////////////////////////
    // this is our contructor
    ///////////////////////////////////////////////
    manual_calib(string file,string method);
   
    void image_Callback(const sensor_msgs::ImageConstPtr& msg);
   
   ////////////////////////////////////////////////////////////////////////////////
   // Callback
   ////////////////////////////////////////////////////////////////////////////////
   void cloud_Callback (const sensor_msgs::PointCloud2ConstPtr& cloudI);
 

   //////////////////////////////////////////////////

   // selection of image points
   
  
 //  void PickingEventOccurred_x3d(const visualization::PointPickingEvent& event, void* viewer_void,MatrixXf &x3d);

 };