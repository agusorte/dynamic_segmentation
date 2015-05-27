#include "cxcore.h"
#include <cv.h>
#include <highgui.h>
#include "opencv2/contrib/contrib.hpp"
#include<opencv2/opencv.hpp>


//vtk
#include <vtkSmartPointer.h>
#include <vtkImageShiftScale.h>
#include <string>

//visualization  PCL

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


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
 #include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace cv;


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

void Set_flow2d23d(MatrixXf projected_cloud_ext, const Mat_<Point2f> flow,
		   MatrixXf flow_projected, MatrixXf P_ext,Matrix3f Trans_ ,
		   MatrixXf & flow_3d,MatrixXf & P3D,  
		   const PointCloud<PointXYZ>::Ptr&  cloud_flow,
		    const PointCloud<PointXYZ>::Ptr&  cloud_p3d,
		    const PointCloud<Normal>::Ptr & Normals,
		   const PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB);

void project_cloud2image( Eigen::MatrixXf cloud_1, Eigen::MatrixXf cloud_non_floor, 
			  Mat & im_current_color,  Matrix3f Trans_ , MatrixXf P_,  
			  MatrixXf &projected_cloud, MatrixXf &projected_cloud_ext, MatrixXf &cloud_on_image, 
			  MatrixXf &projected_cloud_non_floor, MatrixXf &cloud_on_image_non_floor,
			  const PointCloud<PointXYZRGB>::Ptr &  cloud_RGB);

void Set_flow2d23d(MatrixXf projected_cloud_ext, 
			 const Mat_<Point2f> flow,
			 Mat im_color,
			 MatrixXf flow_projected, 
		         MatrixXf P_ext,
		         MatrixXf & flow_3d,
			 MatrixXf & P3D,  
		    const PointCloud<PointXYZ>::Ptr&  cloud_flow,
		    const PointCloud<PointXYZ>::Ptr&  cloud_p3d,
		    const PointCloud<Normal>::Ptr & Normals,
		    const PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB_flow, 
		    const PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB);

void SegmentData(PointCloud<PointXYZ>::ConstPtr cloud_,
		const PointCloud<PointXYZ>::Ptr &  cloud_filtered ,
		 PointCloud<PointXYZ>::Ptr& cloud_nonPlanes,
   PointCloud<PointXYZ>::Ptr &cloud_Plane,
   PointCloud<PointXYZ>::Ptr& cloud_projected );

void Gaussian_images(BackgroundSubtractorMOG2 &bg_im, Mat& im_current_color, Mat& im_fore, Mat& im_back,
		     std::vector<std::vector<cv::Point> > &im_contours , Scalar color );
   

void cluster_pointcloud(const PointCloud<PointNormal>::Ptr Cloud_Normal, 
			const PointCloud<PointXYZRGB>::Ptr& Clusters,
			visualization_msgs::Marker Point_flows,
			visualization_msgs::Marker& points,
			visualization_msgs::Marker& line_list,
			visualization_msgs::MarkerArray& marker_arrow);

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

void   Set_range_flow(const boost::shared_ptr<pcl::RangeImage> range_image_ptr,Mat im_range,
			   const Mat flow,
			  const PointCloud<PointXYZ>::Ptr&  cloud_flow,
		         const PointCloud<PointXYZ>::Ptr&  cloud_p3d,
		    const PointCloud<Normal>::Ptr & Normals, const PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB);


void Publish_Marks(const PointCloud<PointXYZ>::Ptr  cloud_flow,
		    const PointCloud<PointXYZ>::Ptr  cloud_p3d, 
		   visualization_msgs::Marker& points,
		   visualization_msgs::Marker& line_list);
void Publish_Marks(const PointCloud<PointXYZ>::Ptr  cloud_flow,

		    const PointCloud<PointXYZ>::Ptr  cloud_p3d, 

		   visualization_msgs::Marker& points,

		   visualization_msgs::Marker& line_list,int step);