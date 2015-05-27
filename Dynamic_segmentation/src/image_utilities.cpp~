#include "image_utilities.h"



void  GenerarImagenOpenCV(boost::shared_ptr<pcl::RangeImage> range_image_ptr,
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
void  drawOptFlowMap(Mat flow, Mat cflowmap, int step,
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

inline bool isFlowCorrect(Point2f u)
{
  return !cvIsNaN(u.x) && !cvIsNaN(u.y) && fabs(u.x) < 1e9 && fabs(u.y) < 1e9;
}

Vec3b computeColor(float fx, float fy)
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

void drawOpticalFlow_color(const Mat_<Point2f>& flow, Mat& dst, float maxmotion = -1)
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

void project_cloud2image( Eigen::MatrixXf cloud_1, Eigen::MatrixXf cloud_non_floor, 
			  Mat & im_current_color,  Matrix3f Trans_ , MatrixXf P_,  
			  MatrixXf &projected_cloud, MatrixXf &cloud_on_image, 
			  MatrixXf &projected_cloud_non_floor, MatrixXf &cloud_on_image_non_floor,
			  const PointCloud<PointXYZRGB>::Ptr &  cloud_RGB)

{
  
  Eigen::MatrixXf cloud_matrix(cloud_1.cols(),3);//quit intensity
  
  Eigen::MatrixXf cloud_matrix_non_floor(cloud_1.cols(),3);//quit intensity
  
  //not intensity
  for (int i=0; i<cloud_1.cols();i++)
  {
    cloud_matrix(i,0)=cloud_1(0,i);
    cloud_matrix(i,1)=cloud_1(1,i);
    cloud_matrix(i,2)=cloud_1(2,i);
  }
  
   for (int i=0; i<cloud_non_floor.cols();i++)
  {
    cloud_matrix_non_floor(i,0)=cloud_non_floor(0,i);
    cloud_matrix_non_floor(i,1)=cloud_non_floor(1,i);
    cloud_matrix_non_floor(i,2)=cloud_non_floor(2,i);
  }
 
  
  /// now we transform
  Eigen::MatrixXf cloud_trans=cloud_matrix*Trans_;
  Eigen::MatrixXf cloud_trans_non_floor=cloud_matrix_non_floor*Trans_;
  
 // cout<<cloud_trans.rows()<<" "<<cloud_trans.cols() <<endl;//matrix ix 4Nx$
  
  
  
  Eigen::MatrixXf cloud_matrix_trans(cloud_trans.rows(),4);//quit intensity
  Eigen::MatrixXf cloud_matrix_trans_non_floor(cloud_trans_non_floor.rows(),4);//quit intensity
  
  
  for (int i=0; i<cloud_trans.rows();i++)
  {
    cloud_matrix_trans(i,0)=cloud_trans(i,0);
    cloud_matrix_trans(i,1)=cloud_trans(i,1);
    cloud_matrix_trans(i,2)=cloud_trans(i,2);
    cloud_matrix_trans(i,3)=1;
  }
  
  for (int i=0; i<cloud_trans_non_floor.rows();i++)
  {
    cloud_matrix_trans_non_floor(i,0)=cloud_trans_non_floor(i,0);
    cloud_matrix_trans_non_floor(i,1)=cloud_trans_non_floor(i,1);
    cloud_matrix_trans_non_floor(i,2)=cloud_trans_non_floor(i,2);
    cloud_matrix_trans_non_floor(i,3)=1;
  }
  
  /// now proyect to image 
  projected_cloud=(P_*cloud_matrix_trans.transpose()).transpose();
  projected_cloud_non_floor=(P_*cloud_matrix_trans_non_floor.transpose()).transpose();
  
  cloud_on_image=MatrixXf::Zero(cloud_matrix_trans.rows(),3); //
  cloud_on_image_non_floor=MatrixXf::Zero(cloud_matrix_trans_non_floor.rows(),3);
  Mat im_color_aux;
  
  im_current_color.copyTo(im_color_aux);
  PointCloud<PointXYZRGB>  PointAuxRGB;
  for(int k=0;k<projected_cloud.rows();k++)
  {
    projected_cloud(k,0)=projected_cloud(k,0)/projected_cloud(k,2);
    projected_cloud(k,1)=projected_cloud(k,1)/projected_cloud(k,2);
    cloud_on_image(k,0)=  cloud_matrix(k,0);
    cloud_on_image(k,1)=  cloud_matrix(k,1);
    cloud_on_image(k,2)=  cloud_matrix(k,2);
    
    ///add point with RGB
    setRGB(projected_cloud(k,0), projected_cloud(k,1), im_color_aux, PointAuxRGB,
	   cloud_on_image(k,0),cloud_on_image(k,1),cloud_on_image(k,2));
    
    cloud_RGB->points.push_back (PointAuxRGB.points[0]);
    
    circle(im_current_color, Point(projected_cloud(k,0),projected_cloud(k,1)), 1, cv::Scalar(255, 0, 0), CV_FILLED, CV_AA);
     
  }
  
   for(int k=0;k<projected_cloud_non_floor.rows();k++)
  {
    projected_cloud_non_floor(k,0)=projected_cloud_non_floor(k,0)/projected_cloud_non_floor(k,2);
    projected_cloud_non_floor(k,1)=projected_cloud_non_floor(k,1)/projected_cloud_non_floor(k,2);
    cloud_on_image_non_floor(k,0)=  cloud_matrix_non_floor(k,0);
    cloud_on_image_non_floor(k,1)=  cloud_matrix_non_floor(k,1);
    cloud_on_image_non_floor(k,2)=  cloud_matrix_non_floor(k,2);
    
   // circle(im_current_color, Point(projected_cloud(k,0),projected_cloud(k,1)), 1, cv::Scalar(255, 0, 0), CV_FILLED, CV_AA);
    
  }
  
}


void SegmentData(PointCloud<PointXYZ>::ConstPtr cloud_,
		const PointCloud<PointXYZ>::Ptr &  cloud_filtered ,
		 PointCloud<PointXYZ>::Ptr& cloud_nonPlanes,
   PointCloud<PointXYZ>::Ptr &cloud_Plane,
   PointCloud<PointXYZ>::Ptr& cloud_projected ){
  
  
    //first are removed sporius points
   PointCloud<PointXYZ>::Ptr cloud_stastic (new PointCloud<PointXYZ>);
   
  //segment in Z
    //now the cloud is filtered in heigh and
   PassThrough<PointXYZ> pass;
   pass.setInputCloud (cloud_);
   pass.setFilterFieldName ("z");// here we have to see where x and z
   pass.setFilterLimits (-1.0, 1.5);
   pass.filter (*cloud_filtered);
   PCL_INFO ("Removed x points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
   
     
     //now the cloud is filtered x

   pass.setInputCloud (cloud_filtered);
   pass.setFilterFieldName ("x");
   pass.setFilterLimits (0, 10.0);//maximum 7 meters;
   pass.filter (*cloud_filtered);
   PCL_INFO ("Removed x points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
   
   
   
   //filter in y
   pass.setInputCloud (cloud_filtered);
   pass.setFilterFieldName ("y");
   pass.setFilterLimits (-1, 3);
   pass.filter (*cloud_filtered);
   PCL_INFO ("Removed y points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
   
   StatisticalOutlierRemoval<PointXYZ> sor;
   sor.setInputCloud (cloud_filtered);
   sor.setMeanK (10);
   sor.setStddevMulThresh (1.0);
   sor.filter (*cloud_filtered);
   
   PCL_INFO ("Removed sporious points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
   
   
   
   ///////////////////////////////////////////////////////////////
   /// get plane
   ///////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
   ///segmentacion of planes 
   //////////////////////////////////////////////////////////////////////////////////////////////
   
    PointIndices::Ptr inliers (new PointIndices);
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
   pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
   seg.setOptimizeCoefficients (true);
   // Mandatory
   seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setMaxIterations (1000);
   seg.setDistanceThreshold (0.04);


   seg.setInputCloud (cloud_filtered);
   seg.segment (*inliers, *coefficients);

   if (inliers->indices.size () == 0)
   {
 //    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
     //return (-1);
   }


  
 //  ROS_INFO ("Model inliers with %d points ",inliers->indices.size());

   /////////////////////////////////////////////////////////////
  //find planes and non planes
  
   
     
     
   pcl::ExtractIndices<pcl::PointXYZ> extract;
   extract.setInputCloud (cloud_filtered);
   extract.setIndices (inliers);
   extract.setNegative (false);
   
   extract.filter (*cloud_Plane); //*plane found
   
  
  //  ROS_INFO ("PointCloud representing the planar component: %d data points",cloud_Plane->points.size ());
        
   // Write the planar inliers to disk
     //extract no planes
   extract.setNegative (true);
   extract.filter (*cloud_nonPlanes); //*
   
  // ROS_INFO ("PointCloud representing the non planar component: %d data points",cloud_nonPlanes->points.size ());
       
       
   //project points to the plane
   pcl::ProjectInliers<PointXYZ> proj;
  proj.setModelType (SACMODEL_PLANE);
  proj.setInputCloud (cloud_Plane);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);    
   
}

void Gaussian_images(BackgroundSubtractorMOG2 &bg_im, Mat& im_current_color, Mat& im_fore, Mat& im_back,
		     std::vector<std::vector<cv::Point> > &im_contours , Scalar color )
{
  im_current_color.copyTo(im_fore);
  //back=Mat(frame.rows, frame.cols, CV_8UC1);
  im_back=Mat(im_current_color.rows, im_current_color.cols, im_current_color.type());
  im_current_color.copyTo(im_back);
  bg_im.operator ()(im_current_color,im_fore);
  bg_im.getBackgroundImage(im_back);
  cv::erode(im_fore,im_fore,cv::Mat());
  cv::dilate(im_fore,im_fore,cv::Mat());
  cv::findContours(im_fore,im_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
  cv::drawContours(im_current_color,im_contours,-1,color,CV_FILLED);
//   imshow("Frame image ",im_current_color);
//   imshow("Background image",im_back);
//   imshow("Fore image",im_fore);
//   waitKey(30);
  
}

///function that transform the contourns to 3d points
void Region2Cloud(vector<vector<cv::Point> > contours,
		  boost::shared_ptr<pcl::RangeImage> range_image_ptr, 
		  const PointCloud<PointXYZ>::Ptr&  cloud)
{
  
  PointCloud<PointXYZ> PointAux;
  PointAux.points.resize(1);
  for(vector<vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ++it)
  {
    vector<cv::Point>  vec2=*it;
    
    for(vector<cv::Point>::iterator it2 = vec2.begin(); it2 != vec2.end(); ++it2)
    {
      cv::Point P_=*it2;
     
      PointWithRange Pr=range_image_ptr->getPoint(P_.x ,P_.y);
     
      
      PointAux.points[0].x=Pr.x;
      PointAux.points[0].y=Pr.y;
      PointAux.points[0].z=Pr.z;
      
      //cout<<PointAux.points[0]<<endl;
      cloud->points.push_back (PointAux.points[0]);
      
    }
  }



//   PointCloud<PointXYZ> PointAux;
//   PointAux.points.resize(1);
//   int value;
//   Vector3f pt;
//   
//   for(int k=0;k<range_image_ptr->size();k++)
//   {
//     float x = range_image_ptr->getPoint(k).x;
//         float y = range_image_ptr->getPoint(k).y;
//     for(int i=0;i<contours.size();i++)
//     {
// 
//       value=cv::pointPolygonTest(contours[i],Point2f(x,y),false);
//       
//       /*cout<<value<<endl;*/ 
//       if(0<=value)//inside of contourn
//       {
//       
//         PointWithRange Pr=range_image_ptr->getPoint(x,y);
//         PointAux.points[0].x=Pr.x;
//       PointAux.points[0].y=Pr.y;
//       PointAux.points[0].z=Pr.z;
//       
//       
//         cloud->points.push_back (PointAux.points[0]);
//       
//       
//        // circle(image_point_on_region, Point(projected_cloud(k,0),projected_cloud(k,1)), 1, cv::Scalar(0, 255, 0), CV_FILLED, CV_AA);
//       
//       }
//     }
//   }
  
}
///this function project the cloud points and compare
void Region2image_point_proj(vector<vector<cv::Point> > contours,
			     MatrixXf projected_cloud, MatrixXf cloud_on_image,
			     const PointCloud<PointXYZ>::Ptr&  cloud_in_contour,
			     Mat &image_point_on_region)

{
  
  PointCloud<PointXYZ> PointAux;
  PointAux.points.resize(1);
  int value;
  for(int k=0;k<projected_cloud.rows();k++)
  {
    
    for(int i=0;i<contours.size();i++)
    {
      value=cv::pointPolygonTest(contours[i],Point2f(projected_cloud(k,0),projected_cloud(k,1)),false);
      
      /*cout<<value<<endl;*/ 
      if(0<=value)//inside of contourn
      {
      
        PointAux.points[0].x=cloud_on_image(k,0);
        PointAux.points[0].y=cloud_on_image(k,1);
        PointAux.points[0].z=cloud_on_image(k,2);
      
        cloud_in_contour->points.push_back (PointAux.points[0]);
      
      
        circle(image_point_on_region, Point(projected_cloud(k,0),projected_cloud(k,1)), 1, cv::Scalar(0, 255, 0), CV_FILLED, CV_AA);
      
      }
    }
    
    
  }
  
}

///this function project the cloud points and compare
void Regionfused(const PointCloud<PointXYZ>::Ptr  cloud_range_image, 
		 const PointCloud<PointXYZ>::Ptr  cloud_image,
		 const PointCloud<PointXYZ>::Ptr&  cloud_fused
                
)

{
  for (int i=0 ;i<cloud_range_image->size();i++)
  {
    for (int j=0 ;j<cloud_image->size();j++)
    {
      
      if(cloud_range_image->points[i].x==cloud_image->points[j].x &&
	 cloud_range_image->points[i].y==cloud_image->points[j].y &&
	 cloud_range_image->points[i].z==cloud_image->points[j].z
      )
      {
	cout<<"point"<<endl;
	cloud_fused->points.push_back(cloud_range_image->points[i]);
      }
      
    }
  }

   
}

//set pixel value to cloud
//col x row y in the 3d point x y z a pixel
void setRGB(float col, float row, Mat image, PointCloud<PointXYZRGB> & PointAuxRGB,
	    float x, float y, float z)
{
  
  int b,r,g;
  
    PointAuxRGB.points.resize(1);

    if ((col<0 || row<0) || (col>image.cols || row>image.rows)){


      r=0;


      g=0;


      b=0;



    }else{


   
      r=image.at<cv::Vec3b>(row,col)[0];


      g=image.at<cv::Vec3b>(row,col)[1];


      b=image.at<cv::Vec3b>(row,col)[2];


    }

//    uint8_t r_i = r;
// 
// 
//     uint8_t g_i = g;
// 
// 
//     uint8_t b_i = b;


    

    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);




    PointAuxRGB.points[0].x=x;

    PointAuxRGB.points[0].y=y;

    PointAuxRGB.points[0].z=z;

    PointAuxRGB.points[0].rgb=*reinterpret_cast<float*>(&rgb);  


    
  
}

/// save images

void saveimages(Mat im_current_color,Mat  im_current_gm,
		Mat im_current_color_region,Mat falseColorsMap,Mat range_color_im,
	       boost::shared_ptr<visualization::PCLVisualizer> viewer, 
		boost::shared_ptr<visualization::PCLVisualizer> viewer2,int n_frame)
{
  string path="/home/aortega/Experiments/velodyne_32E/results/";
   stringstream ss;
   ss << n_frame;
   ///save images
   imwrite(  path+"projected" +ss.str()+".jpg", im_current_color );
   imwrite(  path+"gaussian" +ss.str()+".jpg", im_current_gm );
   imwrite(  path+"region" +ss.str()+".jpg", im_current_color_region );
   
//    imwrite(  path+"range" +ss.str()+".jpg", falseColorsMap );
//    imwrite(  path+"rangeprojected" +ss.str()+".jpg", range_color_im );
    imwrite(  path+"range_completed" +ss.str()+".jpg", falseColorsMap );
   imwrite(  path+"range_projected_completed" +ss.str()+".jpg", range_color_im );
   
   // imwrite(  path+"region" +ss+".jpg", iim_current_color_region );
   //  vis_style =viewer->getInteractorStyle();
   viewer->saveScreenshot(path+"cloud3_segmented_fusion_3_" +ss.str()+".png");
   viewer2->saveScreenshot(path+"cloud_seg_" +ss.str()+".png");
}

void saveimages(Mat im_current_color,Mat  im_current_gm,
		Mat im_current_color_region,Mat falseColorsMap,Mat range_color_im,
	       boost::shared_ptr<visualization::PCLVisualizer> viewer,int n_frame)
{
  string path="/home/aortega/Experiments/velodyne_32E/results/";
   stringstream ss;
   ss << n_frame;
   ///save images
   imwrite(  path+"projected" +ss.str()+".jpg", im_current_color );
   imwrite(  path+"gaussian" +ss.str()+".jpg", im_current_gm );
   imwrite(  path+"region" +ss.str()+".jpg", im_current_color_region );
   
//    imwrite(  path+"range" +ss.str()+".jpg", falseColorsMap );
//    imwrite(  path+"rangeprojected" +ss.str()+".jpg", range_color_im );
    imwrite(  path+"range_completed" +ss.str()+".jpg", falseColorsMap );
   imwrite(  path+"range_projected_completed" +ss.str()+".jpg", range_color_im );
   
   // imwrite(  path+"region" +ss+".jpg", iim_current_color_region );
   //  vis_style =viewer->getInteractorStyle();
   viewer->saveScreenshot(path+"cloud3_segmented_fusion_3_" +ss.str()+".png");
   //viewer2->saveScreenshot(path+"cloud_seg_" +ss.str()+".png");
}
  

