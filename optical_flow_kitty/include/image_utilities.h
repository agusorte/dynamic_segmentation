#include "cxcore.h"
#include <cv.h>
#include <highgui.h>
#include "opencv2/contrib/contrib.hpp"
#include<opencv2/opencv.hpp>
#include <unistd.h>
#include <sys/stat.h>
#include <sensor_msgs/image_encodings.h>
//vtk
#include <vtkSmartPointer.h>
#include <vtkImageShiftScale.h>
#include <string>

//visualization  PCL
#include <visualization_msgs/MarkerArray.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>

//segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common.h>

#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>

 #include <rosbag/bag.h>
 #include <rosbag/view.h>
 #include "ros/ros.h"
 #include <ros/names.h>
 #include <Eigen/Dense>
 #include <tf/transform_listener.h>
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

///EIGEN
#include <Eigen/Dense>


using namespace pcl;
using namespace std;
using namespace Eigen;


void GenerarImagenOpenCV(boost::shared_ptr<pcl::RangeImage> range_image_ptr,
			 Mat & ImageOpenCVRange,Mat & Image_normalized);

void drawOptFlowMap(Mat flow, Mat cflowmap, int step,
		    double scale, Scalar color);

inline bool isFlowCorrect(Point2f u);

Vec3b computeColor(float fx, float fy);

void drawOpticalFlow_color(const Mat_<Point2f>& flow, Mat& dst, float maxmotion);

void Set_projected_flow(const MatrixXf projected_cloud, const Mat_<Point2f> flow,MatrixXf &flow_projected);

void Set_projected_flow(const MatrixXf projected_cloud_prev,
			const MatrixXf projected_cloud,
			const Mat_<Point2f> flow,MatrixXf &flow_projected);


void Set_flow2d23d(MatrixXf projected_cloud_ext, MatrixXf flow_projected, MatrixXf P_ext,Matrix3f Trans_ ,
		   MatrixXf & flow_3d,MatrixXf & P3D,  const PointCloud<PointXYZ>::Ptr&  cloud_flow,
		    const PointCloud<PointXYZ>::Ptr&  cloud_p3d,
		    const PointCloud<Normal>::Ptr & Normals
		  );

void project_cloud2image( Eigen::MatrixXf cloud_1, Eigen::MatrixXf cloud_non_floor, 
			  Mat & im_current_color,  Matrix3f Trans_ , MatrixXf P_,  
			  MatrixXf &projected_cloud, MatrixXf &projected_cloud_ext, MatrixXf &cloud_on_image, 
			  MatrixXf &projected_cloud_non_floor, MatrixXf &cloud_on_image_non_floor,
			  const PointCloud<PointXYZRGB>::Ptr &  cloud_RGB);

void SegmentData(PointCloud<PointXYZ>::ConstPtr cloud_,
		const PointCloud<PointXYZ>::Ptr &  cloud_filtered ,
		 PointCloud<PointXYZ>::Ptr& cloud_nonPlanes,
   PointCloud<PointXYZ>::Ptr &cloud_Plane,
   PointCloud<PointXYZ>::Ptr& cloud_projected );

void Gaussian_images(BackgroundSubtractorMOG2 &bg_im, Mat& im_current_color, Mat& im_fore, Mat& im_back,
		     std::vector<std::vector<cv::Point> > &im_contours , Scalar color );
   

void Region2Cloud(vector<vector<cv::Point> > contours,
		    boost::shared_ptr<pcl::RangeImage> range_image_ptr, 
		    const PointCloud<PointXYZ>::Ptr &  cloud);

void Region2image_point_proj(vector<vector<cv::Point> > contours,
			     MatrixXf projected_cloud, MatrixXf cloud_on_image,
			     const PointCloud<PointXYZ>::Ptr&  cloud_in_contour,
			     Mat &image_point_on_region);

void setRGB(float col, float row, Mat image, PointCloud<PointXYZRGB> & PointAuxRGB,
	    float x, float y, float z);

void Regionfused(const PointCloud<PointXYZ>::Ptr  cloud_range_image, 
		 const PointCloud<PointXYZ>::Ptr  cloud_image,
		 const PointCloud<PointXYZ>::Ptr&  cloud_fused);

void saveimages(Mat im_current_color,Mat  im_current_gm,
		Mat im_current_color_region,Mat falseColorsMap,Mat range_color_im,
	       boost::shared_ptr<visualization::PCLVisualizer> viewer,
		boost::shared_ptr<visualization::PCLVisualizer> viewer2,int n_frame);

void saveimages(Mat im_current_color,Mat  im_current_gm,
		Mat im_current_color_region,Mat falseColorsMap,Mat range_color_im,
	       boost::shared_ptr<visualization::PCLVisualizer> viewer,int n_frame);

void OpenYalm(const string Name, Matrix3f &K);

void OpenYalm(const string Name, Matrix3f &K, MatrixXf &P);

void  Set_range_flow(const boost::shared_ptr<pcl::RangeImage> range_image_ptr,
		      Mat im_range,
	              const Mat flow,
		      const PointCloud<PointXYZ>::Ptr&  cloud_flow,
		      const PointCloud<PointXYZ>::Ptr&  cloud_p3d,
		      const PointCloud<Normal>::Ptr & Normals, 
		      const PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB);

void Set_range_flow(const boost::shared_ptr<pcl::RangeImage> range_image_ptr,
		    const boost::shared_ptr<pcl::RangeImage> range_image_ptr_prev,
		             Mat im_range,
			   const Mat flow,
			  const PointCloud<PointXYZ>::Ptr&  cloud_flow,
		         const PointCloud<PointXYZ>::Ptr&  cloud_p3d,
		    const PointCloud<Normal>::Ptr & Normals, 
		    const PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB);

void Publish_Marks(const PointCloud<PointXYZ>::Ptr  cloud_flow,
		    const PointCloud<PointXYZ>::Ptr  cloud_p3d, 
		   visualization_msgs::Marker& points,
		   visualization_msgs::Marker& line_list);
void Publish_Marks(const PointCloud<PointXYZ>::Ptr  cloud_flow,
		    const PointCloud<PointXYZ>::Ptr  cloud_p3d, 
		   visualization_msgs::Marker& points,
		   visualization_msgs::Marker& line_list,int step);
///////////////////////////////////////////////////////////////////////////////////////////////
///Kitty
///////////////////////////////////////////////////////////////////////////////////////////////

int getdir (string dir, vector<string> &files);

int read_calib_kitty(string dir_calib,
		     MatrixXf &Tr_velo_to_cam, 
		     vector<Matrix3f> &K,
		    vector<Vector2f> &S,
		    vector<Matrix3f> &R,
		    vector<Vector2f> &S_rect,
		    vector<Matrix3f> &R_rect,
		     vector<Vector3f> &T,
		    vector< MatrixXf> &P_rect,
		    vector<VectorXf> &D);

void filter_cloud_kitty(PointCloud<PointXYZ>::ConstPtr cloud_,
		const PointCloud<PointXYZ>::Ptr &  cloud_filtered,
		   PointCloud<PointXYZ>::Ptr& cloud_nonPlanes,
   PointCloud<PointXYZ>::Ptr &cloud_Plane,
   PointCloud<PointXYZ>::Ptr& cloud_projected);

///
void Publish_Range_Data(Mat cflow, 
			Mat color_flow,
			Mat falseColorsMap,  
			PointCloud<PointXYZ>::Ptr cloud_ptr,
                        PointCloud<PointXYZRGB>::Ptr cloud_rgb,
			PointCloud<Normal>::Ptr normals,
			PointCloud<pcl::PointNormal>::Ptr cloud_normals,
			PointCloud<PointXYZ>::Ptr cloud_p3d,
			PointCloud<PointXYZ>::Ptr cloud_flow,
			PointCloud<PointXYZRGB>::Ptr Clusters,
			 ros::Publisher pub_im_range,
			ros::Publisher pub_im_range_flow,
			ros::Publisher pub_im_range_color_flow,
			ros::Publisher pub_cloud,
			ros::Publisher  pub_cloud_range_flow,
			ros::Publisher pub_cloud_range_normal,
			ros::Publisher pub_range_markers,
			ros::Publisher pub_clusters_range,
			ros::Publisher pub_arrows_range
 		      );

void project_cloud2image_kitty( Eigen::MatrixXf cloud_1, 
				Eigen::MatrixXf cloud_non_floor, 
			        Mat & im_current_color, 
			  vector<Matrix3f> R_rect,
			  vector< MatrixXf> P_rect,
			  MatrixXf Tr_velo_to_cam,
			  MatrixXf Trans,
			  MatrixXf &projected_cloud,
			  MatrixXf &projected_cloud_ext, 
			  MatrixXf &cloud_on_image, 
			  MatrixXf &projected_cloud_non_floor, 
			  MatrixXf &cloud_on_image_non_floor,
			  const PointCloud<PointXYZRGB>::Ptr &  cloud_RGB, 
			  int N_camara, MatrixXf &P_ext);

void Set_flow2d23d_kitty(MatrixXf projected_cloud_ext, const Mat_<Point2f> flow,Mat im_color,
		   MatrixXf flow_projected, MatrixXf P_ext,
		   MatrixXf & flow_3d,MatrixXf & P3D,  
		   const PointCloud<PointXYZ>::Ptr&  cloud_flow,
		    const PointCloud<PointXYZ>::Ptr&  cloud_p3d,
		    const PointCloud<Normal>::Ptr & Normals,
		   const PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB_flow, const PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB);



void cluster_pointcloud(const PointCloud<PointNormal>::Ptr Cloud_Normal, 
			const PointCloud<PointXYZRGB>::Ptr& Clusters,
			visualization_msgs::Marker Point_flows,
			visualization_msgs::Marker& points,
			visualization_msgs::Marker& line_list,
			visualization_msgs::MarkerArray& marker_arrow);

//Transmatrix Nx6 ie [(x1i,y1i,0),(x1i,y1i,1),(x1i,yi,1),(x1i,y1i,2)) 
void Transform_Matrix(MatrixXf projected_cloud_ext,MatrixXf projected_cloud_prev,MatrixXf Trans);