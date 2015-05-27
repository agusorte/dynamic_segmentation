#include "image_utilities.h"

image_utilities::image_utilities()
{
}


void image_utilities:: GenerarImagenOpenCV(boost::shared_ptr<pcl::RangeImage> range_image_ptr,
			 Mat & ImageOpenCVRange,Mat & Image_normalized)
{
  int cols = range_image_ptr->width;
  int rows = range_image_ptr->height;
  
  float min_range;
  float max_range;
  range_image_ptr->getMinMaxRanges(min_range, max_range);
  
//   MatrixXf mat_=range_image_ptr->getMgetMatrixXfMap();
//   cout<<mat_.rows()<<" ,"<<mat_.cols()<<endl;
//  cout<<mat_.minCoeff()<<"-- "<<mat_.maxCoeff()<<endl;
  //cout<<"max "<<max_range<<"min "<<min_range<<endl;
//  cout<<"size range image "<<range_image_ptr->points.size()<<endl;
 // cout<<rows<<" -"<<cols<<endl;
   ImageOpenCVRange=Mat(range_image_ptr->height,range_image_ptr->width,CV_32F);
   ImageOpenCVRange=Mat::zeros(range_image_ptr->height,range_image_ptr->width,CV_32F);
  for (size_t i=0; i<(range_image_ptr->points.size()); i++)
  {
      float z = range_image_ptr->getPoint(i).z;
                if (isnan(z) ||  isinf(z))
                        ImageOpenCVRange.at<float>(i)=0;
                else
		{
                      ImageOpenCVRange.at<float>(i)=z;
		}
               
                //std::cout<<i<<' '<<ImageOpenCVRange.at<float>(i)<<' '<<std::endl; 
  }
  
  double max,min;
  cv::minMaxIdx(ImageOpenCVRange, &min, &max);
//ImageOpenCVRange.convertTo(Image_normalized,CV_8UC1, 255 / (max-min), -min); 
cv::convertScaleAbs(ImageOpenCVRange, Image_normalized, 255 / max);
 //cv::normalize(ImageOpenCVRange,Image_normalized,0,255, NORM_MINMAX);
 // cv::normalize(ImageOpenCVRange,Image_normalized, 0, 255, NORM_INF, CV_8UC3);
	  
	
} 
void image_utilities:: drawOptFlowMap(Mat flow, Mat cflowmap, int step,
		    double scale, Scalar color)
{
  int x, y;
  for( y = 0; y < cflowmap.rows; y += step)
    for( x = 0; x < cflowmap.cols; x += step)
    {
      // CvPoint2D32f fxy = CV_MAT_ELEM(*flow, CvPoint2D32f, y, x);
      Point2f fxy=flow.at< Point2f>(y,x);
      cv::line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
	       color, 1, 8, 0);
      cv::circle(cflowmap, cvPoint(x,y), 2, color, -1, 8, 0);
    }
}

inline bool image_utilities:: isFlowCorrect(Point2f u)
{
  return !cvIsNaN(u.x) && !cvIsNaN(u.y) && fabs(u.x) < 1e9 && fabs(u.y) < 1e9;
}

 Vec3b image_utilities::computeColor(float fx, float fy)
{
  static bool first = true;
  
  // relative lengths of color transitions:
  // these are chosen based on perceptual similarity
  // (e.g. one can distinguish more shades between red and yellow
  //  than between yellow and green)
  const int RY = 15;
  const int YG = 6;
  const int GC = 4;
  const int CB = 11;
  const int BM = 13;
  const int MR = 6;
  const int NCOLS = RY + YG + GC + CB + BM + MR;
  static Vec3i colorWheel[NCOLS];
  
  if (first)
  {
    int k = 0;
    
    for (int i = 0; i < RY; ++i, ++k)
      colorWheel[k] = Vec3i(255, 255 * i / RY, 0);
    
    for (int i = 0; i < YG; ++i, ++k)
      colorWheel[k] = Vec3i(255 - 255 * i / YG, 255, 0);
    
    for (int i = 0; i < GC; ++i, ++k)
      colorWheel[k] = Vec3i(0, 255, 255 * i / GC);
    
    for (int i = 0; i < CB; ++i, ++k)
      colorWheel[k] = Vec3i(0, 255 - 255 * i / CB, 255);
    
    for (int i = 0; i < BM; ++i, ++k)
      colorWheel[k] = Vec3i(255 * i / BM, 0, 255);
    
    for (int i = 0; i < MR; ++i, ++k)
      colorWheel[k] = Vec3i(255, 0, 255 - 255 * i / MR);
    
    first = false;
  }
  
  const float rad = sqrt(fx * fx + fy * fy);
  const float a = atan2(-fy, -fx) / (float)CV_PI;
  
  const float fk = (a + 1.0f) / 2.0f * (NCOLS - 1);
  const int k0 = static_cast<int>(fk);
  const int k1 = (k0 + 1) % NCOLS;
  const float f = fk - k0;
  
  Vec3b pix;
  
  for (int b = 0; b < 3; b++)
  {
    const float col0 = colorWheel[k0][b] / 255.f;
    const float col1 = colorWheel[k1][b] / 255.f;
    
    float col = (1 - f) * col0 + f * col1;
    
    if (rad <= 1)
      col = 1 - rad * (1 - col); // increase saturation with radius
      else
	col *= .75; // out of range
	
	pix[2 - b] = static_cast<uchar>(255.f * col);
  }
  
  return pix;
}

 void image_utilities:: drawOpticalFlow_color(const Mat_<Point2f>& flow, Mat& dst, float maxmotion = -1)
{
  dst.create(flow.size(), CV_8UC3);
  dst.setTo(Scalar::all(0));
  
  // determine motion range:
  float maxrad = maxmotion;
  
  if (maxmotion <= 0)
  {
    maxrad = 1;
    for (int y = 0; y < flow.rows; ++y)
    {
      for (int x = 0; x < flow.cols; ++x)
      {
	Point2f u = flow(y, x);
	
	if (!isFlowCorrect(u))
	  continue;
	
	maxrad = max(maxrad, sqrt(u.x * u.x + u.y * u.y));
      }
    }
  }
  
  for (int y = 0; y < flow.rows; ++y)
  {
    for (int x = 0; x < flow.cols; ++x)
    {
      Point2f u = flow(y, x);
      
      if (isFlowCorrect(u))
	dst.at<Vec3b>(y, x) = computeColor(u.x / maxrad, u.y / maxrad);
    }
  }
}

