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
#include <pcl/range_image/range_image.h>

#include <pcl/registration/registration.h>

#include <Eigen/Eigen>
#include <Eigen/SVD> 


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>


using namespace Eigen;
using namespace cv;
using namespace std;
using namespace pcl;


class FindChessBoard3D {
  
public:
  
  FindChessBoard3D();
  ~FindChessBoard3D();
  
  
  ////////////////////////////////////////////////////////////////////
  //data segmentated of the interest region
  ////////////////////////////////////////////////////////////////////
  
 void  SegmentData(PointCloud<PointXYZ>::ConstPtr cloud_,
		const PointCloud<PointXYZ>::Ptr &  cloud_filtered ); 
  
 int SegmentData_by_planes(PointCloud<PointXYZ>::ConstPtr cloud_filtered,
			const PointCloud<PointXYZ>::Ptr &  cloud_Plane,
			const PointCloud<PointXYZ>::Ptr & cloud_nonPlanes,
			const PointCloud<PointXYZ>::Ptr & cloud_projected);
  
 void DrawClouds( visualization::PCLVisualizer viewer,
	      PointCloud<PointXYZ>::ConstPtr  cloud,
	      PointCloud<PointXYZ>::ConstPtr  cloud_filtered,
	      PointCloud<PointXYZ>::ConstPtr  cloud_Plane,
	      PointCloud<PointXYZ>::ConstPtr  cloud_nonPlanes,
	      PointCloud<PointXYZ>::ConstPtr  cloud_projected,
	      PointCloud<PointXYZRGB>::ConstPtr  cloud_RGB,
	      PointCloud<PointXYZ>::ConstPtr  cloud_chessboard,
	      PointCloud<PointXYZ>::ConstPtr  cloud_plane_cluster,
	      PointCloud<PointXYZ>::ConstPtr  cloud_plane_projected,
	      PointCloud<PointXYZ>::ConstPtr  cloud_hull,
	      PointCloud<PointXYZ>::ConstPtr  cloud_chessboard_square);
 
 void Clustering(PointCloud<PointXYZ>::ConstPtr cloud_nonPlanes,
                const PointCloud<PointXYZRGB>::Ptr& cloudRGB_cluster,
		const PointCloud<PointXYZ>::Ptr& cloud_chessboard);
 
  void chessboard_segment(PointCloud<PointXYZ>::ConstPtr cloud_chessboard,
		const PointCloud<PointXYZ>::Ptr& cloud_plane_cluster ,
		const PointCloud<PointXYZ>::Ptr& cloud_nonplane_cluster,
		const PointCloud<PointXYZ>::Ptr& cloud_plane_projected,
		const PointCloud<PointXYZ>::Ptr&   cloud_chessboard_square,
		const PointCloud<PointXYZ>::Ptr&   cloud_hull);
 
 void FindPointsChessboard3D(PointCloud<PointXYZ>::ConstPtr  cloud,
	      const PointCloud<PointXYZ>::Ptr&  cloud_filtered,
	      const PointCloud<PointXYZ>::Ptr&  cloud_Plane,
	      const PointCloud<PointXYZ>::Ptr&  cloud_nonPlanes,
	      const PointCloud<PointXYZ>::Ptr& cloud_nonplane_cluster,
	      const PointCloud<PointXYZ>::Ptr& cloud_projected,
	      const PointCloud<PointXYZRGB>::Ptr& cloudRGB_cluster,
	      const PointCloud<PointXYZ>::Ptr&  cloud_chessboard,
	      const PointCloud<PointXYZ>::Ptr&  cloud_plane_cluster,
	      const PointCloud<PointXYZ>::Ptr&  cloud_plane_projected,
	      const PointCloud<PointXYZ>::Ptr&  cloud_hull,
	      const PointCloud<PointXYZ>::Ptr&  cloud_chessboard_square);
 
 void AssociatePoints(Mat &image, PointCloud<PointXYZ>::ConstPtr  cloud, 
		      MatrixXf &x3d);
 void SortPoints3d(MatrixXf &x2d, PointCloud<PointXYZ>::ConstPtr  cloud, MatrixXf & x3d);
 
  

};