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

using namespace std;
using namespace cv;


///EIGEN
#include <Eigen/Dense>

class image_utilities{
  
  
private:
  
public:
  
  ///constructor
  
  image_utilities();
  //
  
  void GenerarImagenOpenCV(boost::shared_ptr<pcl::RangeImage> range_image_ptr,
			   Mat & ImageOpenCVRange,Mat & Image_normalized);
  
  void drawOptFlowMap(Mat flow, Mat cflowmap, int step,
		      double scale, Scalar color);
  
  inline bool isFlowCorrect(Point2f u);
  
  Vec3b computeColor(float fx, float fy);
  
   void drawOpticalFlow_color(const Mat_<Point2f>& flow, Mat& dst, float maxmotion);
  
 
};
    