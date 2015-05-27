
#include <string>
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <fstream>
#include <string>
#include <cstdlib>

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

//convexhull
#include "pcl/surface/convex_hull.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/segmentation/extract_clusters.h"

#include <pcl/pcl_base.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>



#include <pcl/surface/mls.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


#include <pcl/registration/registration.h>

#include <Eigen/Eigen>
#include <Eigen/SVD> 


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>


#include "Lu.h"

using namespace Eigen;
using namespace cv;
using namespace std;
using namespace pcl;


 ////////////////////////////////
 /// for the event to select 3d point



int Get3d_2dPoints(PointCloud<PointXYZ>::ConstPtr  cloud_p, 
	            PointCloud<PointXYZ>::ConstPtr  cloud_p2, 
		    MatrixXf &x3d, 
		    MatrixXf &x2d,   
		   vector <Point3f>& points3d,
		    vector <Point2f>& imagePoints);

void Lu_method(MatrixXf x3d, MatrixXf x2d, MatrixXf &R,Vector3f &t, MatrixXf &xcam_, MatrixXf K,
  const PointCloud<PointXYZRGB>::Ptr& cloudRGB,//projected data
  const PointCloud<PointXYZ>::Ptr& cloud_laser_cord,//laser coordinates
   MatrixXf& points_projected, Mat image, PointCloud<PointXYZ>::ConstPtr cloud );


void Lu_method(MatrixXf x3d, MatrixXf x2d, MatrixXf &R,Vector3f &t, MatrixXf &xcam_, MatrixXf K);


void SolvePNP_method(vector <Point3f> points3d,
		    vector <Point2f> imagePoints, 
		     MatrixXf &R,Vector3f &t, MatrixXf &xcam_, 
		     MatrixXf K,
                     const PointCloud<PointXYZRGB>::Ptr& cloudRGB,//projected data
                    const PointCloud<PointXYZ>::Ptr& cloud_laser_cord,//laser coordinates
                     MatrixXf& points_projected,Mat image,
		     PointCloud<PointXYZ>::ConstPtr cloud);

void SortPoints(MatrixXf &x2d);
void SortPointsAngle(MatrixXf &x2d);

void SaveCalibrationData(MatrixXf r,Vector3f  t_2);

void OpenYalm(const string Name, Matrix3f &K);
void OpenYalm(const string Name, Matrix3f &K, MatrixXf &P);
void PickingEventOccurred_(const visualization::PointPickingEvent& event, void* viewer_void);
