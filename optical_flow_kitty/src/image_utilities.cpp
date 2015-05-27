#include "image_utilities.h"




void  GenerarImagenOpenCV(boost::shared_ptr<pcl::RangeImage> range_image_ptr,
			  Mat & ImageOpenCVRange,Mat & Image_normalized)
{
//   int cols = range_image_ptr->width;
//   int rows = range_image_ptr->height;
//   
  float min_range;
  float max_range;
  range_image_ptr->getMinMaxRanges(min_range, max_range);
  
  //   int cols = range_image_ptr->width;
//   int rows = range_image_ptr->height;
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
			  MatrixXf &projected_cloud,MatrixXf &projected_cloud_ext, MatrixXf &cloud_on_image, 
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
  
  projected_cloud_ext=MatrixXf::Ones(projected_cloud.rows(),4);
  
  int in=0;
  for(int k=0;k<projected_cloud.rows();k++)
  {
   
    cloud_on_image(k,0)=  cloud_matrix(k,0);
    cloud_on_image(k,1)=  cloud_matrix(k,1);
    cloud_on_image(k,2)=  cloud_matrix(k,2);
    
    projected_cloud_ext(k,0)=projected_cloud(k,0);
    projected_cloud_ext(k,1)=projected_cloud(k,1);
     projected_cloud_ext(k,2)=projected_cloud(k,2);
     
      projected_cloud(k,0)=projected_cloud(k,0)/projected_cloud(k,2);
    projected_cloud(k,1)=projected_cloud(k,1)/projected_cloud(k,2);
     
    ///add point with RGB
    setRGB(projected_cloud(k,0), projected_cloud(k,1), im_color_aux, PointAuxRGB,
	   cloud_on_image(k,0),cloud_on_image(k,1),cloud_on_image(k,2));
    
    cloud_RGB->points.push_back (PointAuxRGB.points[0]);
    
    if ( (0<projected_cloud(k,0) && projected_cloud(k,0)<im_current_color.cols) &&
         (0<projected_cloud(k,1) && projected_cloud(k,1)<im_current_color.rows))
    {
      circle(im_current_color, Point(projected_cloud(k,0),projected_cloud(k,1)), 1, cv::Scalar(255, 0, 0), CV_FILLED, CV_AA);
      in=in+1;
    }
     
  }
  
//   cout<<"point in"<<in<<endl;
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
    
    for(unsigned int i=0;i<contours.size();i++)
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
  for (unsigned int i=0 ;i<cloud_range_image->size();i++)
  {
    for (unsigned int j=0 ;j<cloud_image->size();j++)
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
  
void OpenYalm(const string Name, Matrix3f &K){

  
  FileStorage fs;

   
  fs.open(Name, FileStorage::READ);

  
  Mat K2=Mat_<double > (3, 3);

 
  if (!fs.isOpened()){
            cerr << "Failed to open " << Name << endl;

            
            return ;
   }
        

    double* _r = K2.ptr<double>();

  

  K(0,0)=_r[0];

  K(0,1)=_r[1];

  K(0,2)=_r[2];

  K(1,0)=_r[3];

  K(1,1)=_r[4];

  K(1,2)=_r[5];

  K(2,0)=_r[6];

  K(2,1)=_r[7];

  K(2,2)=_r[8];

  
  fs.release();     

}

void OpenYalm(const string Name, Matrix3f &K,MatrixXf &P)
{
  
  FileStorage fs;
  
  fs.open(Name, FileStorage::READ);

  
  
  P=MatrixXf::Zero(3,4);

  
  Mat K2=Mat_<double > (3, 3);
  Mat K3=Mat_<double > (3, 4);
  
  
  if (!fs.isOpened()){

    cerr << "Failed to open " << Name << endl;

    
    return ;
  }
  
  fs["camera_matrix"] >> K2; 

  fs["projection_matrix"] >> K3; 

  
  
  double* _r = K2.ptr<double>();

  double* _r2 = K3.ptr<double>();

  
  K(0,0)=_r[0];
  K(0,1)=_r[1];
  K(0,2)=_r[2];
  K(1,0)=_r[3];
  K(1,1)=_r[4];
  K(1,2)=_r[5];
  K(2,0)=_r[6];
  K(2,1)=_r[7];
  K(2,2)=_r[8];
  

  P(0,0)=_r2[0];
  P(0,1)=_r2[1];
  P(0,2)=_r2[2];
  P(0,3)=_r2[3];
  P(1,0)=_r2[4];
  P(1,1)=_r2[5];
  P(1,2)=_r2[6];
  P(1,3)=_r2[7];
  P(2,0)=_r2[8];
  P(2,1)=_r2[9];
  P(2,2)=_r2[10];
  P(2,3)=_r2[11];
  
  fs.release();     
}


void Set_projected_flow(const MatrixXf projected_cloud_ext, 
			const Mat_<Point2f> flow,
			MatrixXf &flow_projected)
{
  ///
  int x, y;
  flow_projected=MatrixXf::Ones( projected_cloud_ext.rows(),4);
  
  //
 

  for(int k = 0; k < projected_cloud_ext.rows(); k ++)
  {
    // CvPoint2D32f fxy = CV_MAT_ELEM(*flow, CvPoint2D32f, y, x);
    x=round(projected_cloud_ext(k,0)/projected_cloud_ext(k,2));
    y=round(projected_cloud_ext(k,1)/projected_cloud_ext(k,2));
    
    
    
    Point2f fxy=flow.at< Point2f>(y,x);
    
    ///THIS PART IS WORKING WELL
    flow_projected(k,0)=(x+(fxy.x))*projected_cloud_ext(k,2);
    flow_projected(k,1)=(y+(fxy.y))*projected_cloud_ext(k,2);
    flow_projected(k,2)=projected_cloud_ext(k,2);
    
  }
}


void Set_projected_flow(const MatrixXf projected_cloud_prev,
			const MatrixXf projected_cloud,
			const Mat_<Point2f> flow,MatrixXf &flow_projected)
{
   int x, y;
   int x2, y2;
   
   int min_pts;//minum point projected
  
  
  //
  min_pts=min(projected_cloud_prev.rows(),projected_cloud.rows());
  
  flow_projected=MatrixXf::Ones(min_pts,4);

  for(int k = 0; k < min_pts; k ++)
  {
    // CvPoint2D32f fxy = CV_MAT_ELEM(*flow, CvPoint2D32f, y, x);
    x=round(projected_cloud_prev(k,0)/projected_cloud_prev(k,2));
    y=round(projected_cloud_prev(k,1)/projected_cloud_prev(k,2));
    
    
    
    Point2f fxy=flow.at< Point2f>(y,x);
    ///
    ///THIS PART IS WORKING WELL
    flow_projected(k,0)=(x+(fxy.x))*projected_cloud(k,2);
    flow_projected(k,1)=(y+(fxy.y))*projected_cloud(k,2);
    flow_projected(k,2)=1*projected_cloud(k,2);
    
    
  }
  
}

void Set_flow2d23d(MatrixXf projected_cloud_ext, MatrixXf flow_projected, MatrixXf P_ext,Matrix3f Trans_ ,
		   MatrixXf & flow_3d,MatrixXf & P3D,  const PointCloud<PointXYZ>::Ptr&  cloud_flow,
		    const PointCloud<PointXYZ>::Ptr&  cloud_p3d,
		    const PointCloud<Normal>::Ptr & Normals
		  )
{
  
  PointCloud<PointXYZ> P1,P2;
  
  P1.points.resize(1);
  P2.points.resize(1);
  
  PointCloud<Normal> N_;
  N_.points.resize(1);
  
  /// transform clouds like in the before
  //   for (int i=0;i< flow_projected.rows();i++)
  //   {
    //     flow_projected(i,0)=flow_projected(i,2)
  //     flow_projected(i,2)=flow_projected(i,1);
  //   
  
  //   flow_projected=flow_projected*Trans_;
  //   projected_cloud_ext=flow_projected*Trans_;
  //   
  flow_3d=(P_ext.inverse()*flow_projected.transpose()).transpose();
  P3D=(P_ext.inverse()*projected_cloud_ext.transpose()).transpose();
  
  //   cout<<"point flow size "<< flow_3d.rows()<<" "<< flow_3d.cols()<<endl;
  //    cout<<"point P3d "<< flow_3d.rows()<<" "<< flow_3d.cols()<<endl;
  //   
  //    
  //   flow_3d=flow_3d*Trans_.inverse();
  //     P3D=P3D*Trans_.inverse();
  
  /*  
   * cout<<"point flow size "<< flow_3d.rows()<<" "<< flow_3d.cols()<<endl;
   * cout<<"point P3d "<< flow_3d.rows()<<" "<< flow_3d.cols()<<endl;*/
  
  
  for (int i=0;i< flow_3d.rows();i++)
  {
    //we use trans 
    P1.points[0].z=P3D(i,0);
    P1.points[0].y=P3D(i,1);
    P1.points[0].x=-P3D(i,2);
    
    cloud_p3d->push_back(P1.points[0]);
    
    P2.points[0].z=flow_3d(i,0)+1;
    P2.points[0].y=flow_3d(i,1);
    P2.points[0].x=-flow_3d(i,2);
    
  
    cloud_flow->push_back(P2.points[0]);
    
    float norm_points=sqrt((P1.points[0].x-P2.points[0].x)*(P1.points[0].x-P2.points[0].x)+
    (P1.points[0].y-P2.points[0].y)*(P1.points[0].y-P2.points[0].y)+
    (P1.points[0].z-P2.points[0].z)*(P1.points[0].z-P2.points[0].z));
 
    
    N_.points[0].normal_x=(P2.points[0].x-P1.points[0].x)/norm_points;
    N_.points[0].normal_y=(P2.points[0].y-P1.points[0].y)/norm_points;
    N_.points[0].normal_z=(P2.points[0].z-P1.points[0].z)/norm_points;
    
    Normals->push_back(N_.points[0]);
  }
  
//   cout<<"point flow size "<< cloud_p3d->size()<<endl;
//    cout<<"point P3d "<< cloud_flow->size()<<endl;
//    cout<<"normals "<< Normals->size()<<endl;
}
void Set_range_flow(const boost::shared_ptr<pcl::RangeImage> range_image_ptr,
		    const boost::shared_ptr<pcl::RangeImage> range_image_ptr_prev,
		             Mat im_range,
			   const Mat flow,
			  const PointCloud<PointXYZ>::Ptr&  cloud_flow,
		         const PointCloud<PointXYZ>::Ptr&  cloud_p3d,
		    const PointCloud<Normal>::Ptr & Normals, 
		    const PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB)
{
  int x, y;
  
  PointXYZ p1,p2;
  PointXYZRGB prgb;
  Normal N_;
  

  float maxrad = -1;
  
  if (maxrad <= 0)
  {
    maxrad = 1;
    for (int y = 0; y < im_range.rows; ++y)
    {
      for (int x = 0; x < im_range.cols; ++x)
      {
	Point2f u = flow.at< Point2f>(y,x);
	
	if (!isFlowCorrect(u))
	  continue;
	
	maxrad = max(maxrad, sqrt(u.x * u.x + u.y * u.y));
      }
    }
  }
  
  for( y = 0; y < im_range.rows; y ++)
    for( x = 0; x < im_range.cols; x ++)
    {
      
      Point2f fxy=flow.at< Point2f>(y,x);
     
      if (isFlowCorrect(fxy))
      {

	
	PointWithRange Pr;
	//range_image_ptr->calculate3DPoint (x, y, Pr);
	Pr=range_image_ptr_prev->getPoint(x ,y);
	//cout<<"P1-> "<<endl;
	p1.x=Pr.x;
	p1.y=Pr.y;
	p1.z=Pr.z;
	
	PointWithRange Pr2;
	range_image_ptr->calculate3DPoint (x+fxy.x ,y+fxy.y,Pr.range, Pr2);
        p2.x=Pr2.x;
	p2.y=Pr2.y;
	p2.z=Pr2.z;

// 	
	float norm_=sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y)+(p2.z-p1.z)*(p2.z-p1.z));
// 	float norm_=1;
	
	if (!isnan(norm_) && norm_!=0 && range_image_ptr->isValid(x+fxy.x ,y+fxy.y))
	{
	  N_.normal_x=(p2.x-p1.x)/norm_;
	  N_.normal_y=(p2.y-p1.y)/norm_;
	  N_.normal_z=(p2.z-p1.z)/norm_;

	  
	 if (N_.normal_x !=0 && N_.normal_y !=0 && N_.normal_z !=0)
	 {
	  cloud_flow->push_back(p2);
	  cloud_p3d->push_back(p1);
	   
	  Normals->push_back(N_);
	  
	  Vec3b  value=computeColor(fxy.x / maxrad, fxy.y / maxrad);
          //cout<<"vec3b "<<(int)value[0]<<" " <<(int)value[1]<<" "<<(int)value[2]<<endl;
	  
	  prgb.x=Pr.x;
	  prgb.y=Pr.y;
	  prgb.z=Pr.z;
	  
	  uint32_t rgb = (static_cast<uint32_t>(value[0]) << 16 
	   | static_cast<uint32_t>(value[1]) << 8 | static_cast<uint32_t>(value[2]));
	   
	 //  cout<<"rgb ->"<<rgb<<endl;
           prgb.rgb = *reinterpret_cast<float*>(&rgb);
	   
	   cloudRGB->push_back(prgb);
	 }
	
	}
      }
    }
  

}
  

void Set_range_flow(const boost::shared_ptr<pcl::RangeImage> range_image_ptr,
		             Mat im_range,
			   const Mat flow,
			  const PointCloud<PointXYZ>::Ptr&  cloud_flow,
		         const PointCloud<PointXYZ>::Ptr&  cloud_p3d,
		    const PointCloud<Normal>::Ptr & Normals, 
		    const PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB)
{
  int x, y;
  
  PointXYZ p1,p2;
  PointXYZRGB prgb;
  Normal N_;
  
//   cout<<"Resolution range image x->"<<range_image_ptr->width <<" y ->"<<range_image_ptr->height<<endl;
//   cout<<"Resolution range image created x->"<<im_range.cols <<" y ->"<<im_range.rows<<endl;
//   cout<<"Resolution flow image x->"<<flow.cols <<" y ->"<<flow.rows<<endl;
  // determine motion range:
  float maxrad = -1;
  
  if (maxrad <= 0)
  {
    maxrad = 1;
    for (int y = 0; y < im_range.rows; ++y)
    {
      for (int x = 0; x < im_range.cols; ++x)
      {
	Point2f u = flow.at< Point2f>(y,x);
	
	if (!isFlowCorrect(u))
	  continue;
	
	maxrad = max(maxrad, sqrt(u.x * u.x + u.y * u.y));
      }
    }
  }
  
  for( y = 0; y < im_range.rows; y ++)
    for( x = 0; x < im_range.cols; x ++)
    {
      // CvPoint2D32f fxy = CV_MAT_ELEM(*flow, CvPoint2D32f, y, x);
      Point2f fxy=flow.at< Point2f>(y,x);
     
      if (isFlowCorrect(fxy))
      {
// 	cout<<"x-> "<<x << " y-> " << y <<endl;
// 	cout<<"fx.x-> "<<fxy.x << " fxy.y-> " <<fxy.y <<endl;
// 	
	
	PointWithRange Pr;
	//range_image_ptr->calculate3DPoint (x, y, Pr);
	Pr=range_image_ptr->getPoint(x ,y);
	//cout<<"P1-> "<<endl;
	p1.x=Pr.x;
	p1.y=Pr.y;
	p1.z=Pr.z;
	
	PointWithRange Pr2;
	range_image_ptr->calculate3DPoint (x+fxy.x ,y+fxy.y,Pr.range, Pr2);
        p2.x=Pr2.x;
	p2.y=Pr2.y;
	p2.z=Pr2.z;
// 	if(range_image_ptr->isValid(x+fxy.x ,y+fxy.y))
// 	{
// 	  Pr2=range_image_ptr->getPoint(x+fxy.x ,y+fxy.y);
// 	//cout<<"P2-> "<<endl;
// 	p2.x=Pr2.x;
// 	p2.y=Pr2.y;
// 	p2.z=Pr2.z;
// 	}
// 	else
// 	{
// 	  p2.x=0.0;
// 	  p2.y=0.0;
// 	  p2.z=0.0;
// 	}
// 	
	float norm_=sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y)+(p2.z-p1.z)*(p2.z-p1.z));
// 	float norm_=1;
	
	if (!isnan(norm_) && norm_!=0 && range_image_ptr->isValid(x+fxy.x ,y+fxy.y))
	{
	  N_.normal_x=(p2.x-p1.x)/norm_;
	  N_.normal_y=(p2.y-p1.y)/norm_;
	  N_.normal_z=(p2.z-p1.z)/norm_;
	  
// 	  cout<<"x-> "<<x << " y-> " << y <<endl;
// 	cout<< "p1"<<p1<<endl;
// 	cout<<" x+fx.x-> "<<cvRound(x+fxy.x) << " y+fxy.y-> " << cvRound(y+fxy.y) <<endl;
// 	cout<< "p2"<<p2<<endl;
	//  cout<< "N_"<<N_<<endl;
// 	  
// 	 cout<<"normal sqrt ->"<<norm_ <<endl;
	  
	 if (N_.normal_x !=0 && N_.normal_y !=0 && N_.normal_z !=0)
	 {
	  cloud_flow->push_back(p2);
	  cloud_p3d->push_back(p1);
	   
	  Normals->push_back(N_);
	  
	  Vec3b  value=computeColor(fxy.x / maxrad, fxy.y / maxrad);
          //cout<<"vec3b "<<(int)value[0]<<" " <<(int)value[1]<<" "<<(int)value[2]<<endl;
	  
	  prgb.x=Pr.x;
	  prgb.y=Pr.y;
	  prgb.z=Pr.z;
	  
	  uint32_t rgb = (static_cast<uint32_t>(value[0]) << 16 
	   | static_cast<uint32_t>(value[1]) << 8 | static_cast<uint32_t>(value[2]));
	   
	 //  cout<<"rgb ->"<<rgb<<endl;
           prgb.rgb = *reinterpret_cast<float*>(&rgb);
	   
	   cloudRGB->push_back(prgb);
	 }
	
	}
      }
    }
  
//   for(int k=0;k<range_image_ptr->size();k++)
//   {

//     
//    
//       x =round( range_image_ptr->getPoint(k).x);
//       y = round(range_image_ptr->getPoint(k).y);
//       
//        cout<<"x "<<x << "y " << y <<endl;
//       // CvPoint2D32f fxy = CV_MAT_ELEM(*flow, CvPoint2D32f, y, x);
//       Point2f fxy=flow.at< Point2f>(y,x);
// 
//      PointWithRange Pr=range_image_ptr->getPoint(x ,y);
//     
//       p1.x=Pr.x;
//       p1.y=Pr.y;
//       p1.z=Pr.z;
//       
//       PointWithRange Pr2=range_image_ptr->getPoint(x+fxy.x ,y+fxy.y);
//       p2.x=Pr2.x;
//       p2.y=Pr2.y;
//       p2.z=Pr2.z;
//       
//       float norm_=sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y)+(p2.y-p1.y)*(p2.y-p1.y));
//       
//       N_.normal_x=(p2.x-p1.x)/norm_;
//       N_.normal_y=(p2.y-p1.y)/norm_;
//       N_.normal_z=(p2.z-p1.z)/norm_;
//       
//       cloud_flow->push_back(p1);
//       
//       Normals->push_back(N_);
//       
// //       cv::line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
// // 	       color, 1, 8, 0);
//       
//       
//       
//      
//     }
}

void Publish_Marks(const PointCloud<PointXYZ>::Ptr  cloud_flow,

		    const PointCloud<PointXYZ>::Ptr  cloud_p3d, 

		   visualization_msgs::Marker& points,

		   visualization_msgs::Marker& line_list)

		  

{



    points.header.frame_id =  line_list.header.frame_id = "/velodyne";

    points.header.stamp =line_list.header.stamp = ros::Time::now();

    points.ns =  line_list.ns = "points_and_lines";

    points.action  = line_list.action = visualization_msgs::Marker::ADD;

    points.pose.orientation.w =line_list.pose.orientation.w = 1.0;

    

     points.id = 0;

    line_list.id = 1;







    points.type = visualization_msgs::Marker::POINTS;

     line_list.type = visualization_msgs::Marker::LINE_LIST;




    // POINTS markers use x and y scale for width/height respectively

    points.scale.x = 0.01;

    points.scale.y = 0.01;



    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width

    line_list.scale.x =line_list.scale.y=line_list.scale.z=0.01;



    // Points are blue

    points.color.b = 1.0f;

    points.color.a = 1.0;





    // Line list is green

    line_list.color.g = 1.0;

    line_list.color.a = 1.0;

    
    int step=1;
    for (uint32_t i = 0; i < cloud_p3d->size(); i=i+step)

    {

      

      geometry_msgs::Point p1,p2;

      p1.x = cloud_p3d->points[i].x;

      p1.y = cloud_p3d->points[i].y;

      p1.z = cloud_p3d->points[i].z;

      

      p2.x = cloud_flow->points[i].x;

      p2.y = cloud_flow->points[i].y;

      p2.z = cloud_flow->points[i].z;



      points.points.push_back(p1);





      // The line list needs two points for each line

      line_list.points.push_back(p1);

      line_list.points.push_back(p2);

    }



   

}


void Publish_Marks(const PointCloud<PointXYZ>::Ptr  cloud_flow,

		    const PointCloud<PointXYZ>::Ptr  cloud_p3d, 

		   visualization_msgs::Marker& points,

		   visualization_msgs::Marker& line_list,int step)	  
{



    points.header.frame_id =  line_list.header.frame_id = "/velodyne";

    points.header.stamp =line_list.header.stamp = ros::Time::now();

    points.ns =  line_list.ns = "points_and_lines";

    points.action  = line_list.action = visualization_msgs::Marker::ADD;

    points.pose.orientation.w =line_list.pose.orientation.w = 1.0;

    

     points.id = 0;

    line_list.id = 1;







    points.type = visualization_msgs::Marker::POINTS;

     line_list.type = visualization_msgs::Marker::LINE_LIST;




    // POINTS markers use x and y scale for width/height respectively

    points.scale.x = 0.01;

    points.scale.y = 0.01;



    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width

    line_list.scale.x =line_list.scale.y=line_list.scale.z=0.01;



    // Points are blue

    points.color.b = 1.0f;

    points.color.a = 1.0;





    // Line list is green

    line_list.color.g = 1.0;

    line_list.color.a = 1.0;

    
    
    for (uint32_t i = 0; i < cloud_p3d->size(); i=i+step)

    {


      geometry_msgs::Point p1,p2;

      p1.x = cloud_p3d->points[i].x;
      p1.y = cloud_p3d->points[i].y;
      p1.z = cloud_p3d->points[i].z;

      

      p2.x = cloud_flow->points[i].x;
      p2.y = cloud_flow->points[i].y;
      p2.z = cloud_flow->points[i].z;
      float dist=sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));

      if (dist<1)
      {
	points.points.push_back(p1);
	// The line list needs two points for each line
	line_list.points.push_back(p1);	
	line_list.points.push_back(p2);
      }
      

    }



   

}
////////////////////////////////////////////////////////
///kitty
/////////////////////////////////////////////////////////

int getdir (string dir, vector<string> &files)
{
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL) {
    cout << "Error(" << errno << ") opening " << dir << endl;
    return errno;
  }
  
  while ((dirp = readdir(dp)) != NULL) {
    files.push_back(string(dirp->d_name));
  }
  closedir(dp);
  
  return 0;
}

int read_calib_kitty(string dir_calib,
		     MatrixXf &Tr_velo_to_cam, 
		     vector<Matrix3f> &K,
		    vector<Vector2f> &S,
		    vector<Matrix3f> &R,
		    vector<Vector2f> &S_rect,
		    vector<Matrix3f> &R_rect,
		     vector<Vector3f> &T,
		    vector< MatrixXf> &P_rect,
		    vector<VectorXf> &D)
{
  /////////////////////////////////////////////////////////////////////////////
   /// read calibration velodyne camera
   //////////////////////////////////////////////////////////////////////////////
    ifstream myReadFile;
    string strLine;
   string calib_file=dir_calib+"/calib_velo_to_cam.txt";
   myReadFile.open(calib_file.c_str());
   
   string name;
   float a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12;
   
   if(!myReadFile.good()){
     cout<<" error reading file"<<endl;
     return(0);
   }
   
   std::getline(myReadFile,strLine);//<-date
   cout<<strLine<<endl;
   
   myReadFile>>name>>a1>>a2>>a3>>a4>>a5>>a6>>a7>>a8>>a9;
   
   myReadFile>>name>>a10>>a11>>a12;
   
   
   Tr_velo_to_cam(0,0)=a1;
   Tr_velo_to_cam(0,1)=a2;
   Tr_velo_to_cam(0,2)=a3;
   Tr_velo_to_cam(1,0)=a4;
   Tr_velo_to_cam(1,1)=a5;
   Tr_velo_to_cam(1,2)=a6;
   Tr_velo_to_cam(2,0)=a7;
   Tr_velo_to_cam(2,1)=a8;
   Tr_velo_to_cam(2,2)=a9;
   
   Tr_velo_to_cam(0,3)=a10;
   Tr_velo_to_cam(1,3)=a11;
   Tr_velo_to_cam(2,3)=a12;
   Tr_velo_to_cam(3,3)=1.0f;
   
  
   
   myReadFile.close();
   
   Matrix3f K_aux;
   Vector2f S_aux;
   Vector2f S_aux2;
   Matrix3f R_aux;
   Matrix3f R_aux2;
   VectorXf D_aux(5);
   Vector3f T_aux;
   MatrixXf P_aux(3,4);
   
   calib_file=dir_calib+"/calib_cam_to_cam.txt";
   myReadFile.open(calib_file.c_str());
   
   if(!myReadFile.good()){
     cout<<" error reading file"<<endl;
     return(0);
   }
   std::getline(myReadFile,strLine);//<-date
   std::getline(myReadFile,strLine);//<-corner
   ///repeat 4 times for the cameras
   
   for (int j=0;j<4;j++)
   {
     myReadFile>>name>>S_aux(0)>>S_aux(1);
     //cout<<S_aux<<endl;
     S.at(j)=S_aux;
     
     myReadFile>>name>>K_aux(0,0)>>K_aux(0,1)>>K_aux(0,2)
     >>K_aux(1,0)>>K_aux(1,1)>>K_aux(1,2)>>K_aux(2,0)>>K_aux(2,1)>>K_aux(2,2);
     //  cout<<"K "<<K_aux<<endl;
     K.at(j)=K_aux;
     
     myReadFile>>name>>D_aux(0)>>D_aux(1)>>D_aux(2)>>D_aux(3)>>D_aux(4);
     //  cout<<"D "<<D_aux<<endl;
     D.at(j)=D_aux;
     myReadFile>>name>>R_aux(0,0)>>R_aux(0,1)>>R_aux(0,2)
     >>R_aux(1,0)>>R_aux(1,1)>>R_aux(1,2)>>R_aux(2,0)>>R_aux(2,1)>>R_aux(2,2);
     //    cout<<"R "<<R_aux<<endl; 
     R.at(j)=R_aux;
     
     myReadFile>>name>>T_aux(0)>>T_aux(1)>>T_aux(2);
     //  cout<<"T "<<T_aux<<endl;
     T.at(j)=T_aux; 
     myReadFile>>name>>S_aux2(0)>>S_aux2(1);
     //    cout<<"S "<<S_aux2<<endl;
     S_rect.at(j)=S_aux2;
     myReadFile>>name>>R_aux2(0,0)>>R_aux2(0,1)>>R_aux2(0,2)
     >>R_aux2(1,0)>>R_aux2(1,1)>>R_aux2(1,2)>>R_aux2(2,0)>>R_aux2(2,1)>>R_aux2(2,2);
     //   cout<<"R "<<R_aux2<<endl;
     R_rect.at(j)=R_aux2;
     myReadFile>>name>>P_aux(0,0)>>P_aux(0,1)>>P_aux(0,2)>>P_aux(0,3)
     >>P_aux(1,0)>>P_aux(1,1)>>P_aux(1,2)>>P_aux(1,3)>>P_aux(2,0)>>P_aux(2,1)>>P_aux(2,2)>>P_aux(2,3);
     //  cout<<"P "<<P_aux<<endl;
     P_rect.at(j)=P_aux;
     
   }
   
   //   for(int j=0;j<4;j++)
   //     cout<<S.at(j)<<endl;
   
   
   myReadFile.close();
   
   
   return (1);
   
}

void filter_cloud_kitty(PointCloud<PointXYZ>::ConstPtr cloud_,
		const PointCloud<PointXYZ>::Ptr &  cloud_filtered,
		   PointCloud<PointXYZ>::Ptr& cloud_nonPlanes,
   PointCloud<PointXYZ>::Ptr &cloud_Plane,
   PointCloud<PointXYZ>::Ptr& cloud_projected)
{
  //first are removed sporius points
   PointCloud<PointXYZ>::Ptr cloud_stastic (new PointCloud<PointXYZ>);
   
  //segment in Z
    //now the cloud is filtered in heigh and
   PassThrough<PointXYZ> pass;
   pass.setInputCloud (cloud_);
   pass.setFilterFieldName ("z");// here we have to see where x and z
   pass.setFilterLimits (-6.0, 3.0);///WORK well
   pass.filter (*cloud_filtered);
   PCL_INFO ("Removed x points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
   
     
     //now the cloud is filtered x

   pass.setInputCloud (cloud_filtered);
   pass.setFilterFieldName ("x");
   pass.setFilterLimits (2, 5000);///WORK well
   pass.filter (*cloud_filtered);
   PCL_INFO ("Removed x points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
   
   
   
   //filter in y
   pass.setInputCloud (cloud_filtered);
   pass.setFilterFieldName ("y");
   pass.setFilterLimits (-20, 20);///WORK well
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
   seg.setDistanceThreshold (0.1);


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

void Publish_Range_Data(const Mat cflow, 
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
 		      )
{
            cv_bridge::CvImagePtr  cv_send(new cv_bridge::CvImage);
	    cv_send->image=Mat(falseColorsMap.rows, falseColorsMap.cols, CV_8UC3);
	    cv_send->encoding=enc::RGB8;
	    ///////////////////////////////////////////////////////////////////
	    ///range image 
	    ////////////////////////////////////////////////////////////////////
	    falseColorsMap.copyTo(cv_send->image);
	    pub_im_range.publish(cv_send->toImageMsg());
	    ///flow arrows for range image
	    cv_send->image=Mat(cflow.rows, cflow.cols, CV_8UC3);
	    cflow.copyTo(cv_send->image);
	    pub_im_range_flow.publish(cv_send->toImageMsg());
	    
	    ///color flow
	    cv_send->image=Mat( color_flow.rows,  color_flow.cols, CV_8UC3);
	    color_flow.copyTo(cv_send->image);
	    
	    pub_im_range_color_flow.publish(cv_send->toImageMsg());
	    
	    
	    
	    
	    ///////////////////////////////////////////////
	    ///Publish the real cloud
	    sensor_msgs::PointCloud2 output_cloud;
	    
	    pcl::toROSMsg(*cloud_ptr,output_cloud);
	   
	    output_cloud.header.frame_id = "/velodyne";
	 
	    pub_cloud.publish(output_cloud);
	  
	   
	    
	    ////////////////////////////////////////////////////////////////

	    ///cloud flow
	    ////////////////////////////////////////////////////////////////

	    sensor_msgs::PointCloud2 output_cloud3;
	    pcl::toROSMsg(*cloud_rgb,output_cloud3);
	    output_cloud3.header.frame_id = "/velodyne";
	    pub_cloud_range_flow.publish(output_cloud3);
	    
	    //////////////////////////////////////////////////////////////

	    ///point normal
	    //////////////////////////////////////////////////////////////

	    sensor_msgs::PointCloud2 output_cloud2;  
	    pcl::concatenateFields(*cloud_p3d,* normals,*cloud_normals);
	    
	    pcl::toROSMsg(*cloud_normals,output_cloud2);
	    output_cloud2.header.frame_id = "/velodyne";
	    pub_cloud_range_normal.publish(output_cloud2);
	    
	    //////////////////////////////////////////////////////////////


	    ///markers
	    ///////////////////////////////////////////////////////////


	    visualization_msgs::Marker points,line_list;
	    Publish_Marks(cloud_flow,cloud_p3d, points,line_list,5);
	    pub_range_markers.publish(points);
	    pub_range_markers.publish(line_list);
	    
// 	    ///NOTE dont use this part cause is slow  now
// 	     ///CLUSTERS
// 	    ///////////////////////////////////////////////
// 	    ///clusters normal 
// 	    
// 	    visualization_msgs::Marker points2,lines2;
// 	    visualization_msgs::MarkerArray arrows;
//             cluster_pointcloud(cloud_normals,  Clusters,points,points2,lines2,arrows);
// 	    //ROS_INFO("cloud clusters ---> received with %d points ",  Clusters->size());
// 	    
// 	    pcl::toROSMsg(*Clusters,output_cloud2);
// 	    output_cloud2.header.frame_id ="/velodyne";
// 	    pub_clusters_range.publish(output_cloud2);
// 	    
// 	    
// 	   
// 	    
// 	     pub_arrows_range.publish(points2);///DONE after put a new publiser for the centers
// 	     pub_arrows_range.publish(lines2);///DONE directions
}

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
			  int N_camara, MatrixXf &P_ext)
{
  
  ///NOTE N_camara is the cara of 4 
  ///0 ---> im_gray1
  /// 1 --->im_gray2
  /// 2 --->im_color1
  /// 3 --->im_color2 
  MatrixXf R_cam_to_rect(4,4);
  MatrixXf P_velo_to_img(3,4);
  
  R_cam_to_rect=MatrixXf::Identity(4,4);
  Matrix3f R_rect_=R_rect.at(N_camara);
  R_cam_to_rect.block<3,3>(0,0)=R_rect_;
  P_velo_to_img=P_rect.at(N_camara)*R_cam_to_rect*Tr_velo_to_cam;
  
  Eigen::MatrixXf cloud_matrix(cloud_1.cols(),3);//quit intensity
  Eigen::MatrixXf cloud_matrix_non_floor(cloud_1.cols(),3);//quit intensity
  /////////////////////////////////////////////////
  ///not intensity
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
 
  
  /// now we transform ---here is not useful the trenaformation 
  Eigen::MatrixXf cloud_trans=cloud_matrix*Trans;
  Eigen::MatrixXf cloud_trans_non_floor=cloud_matrix_non_floor*Trans;
  
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
  projected_cloud=(P_velo_to_img*cloud_matrix_trans.transpose()).transpose();
  projected_cloud_non_floor=(P_velo_to_img*cloud_matrix_trans_non_floor.transpose()).transpose();
  
  cloud_on_image=MatrixXf::Zero(cloud_matrix_trans.rows(),3); //
  cloud_on_image_non_floor=MatrixXf::Zero(cloud_matrix_trans_non_floor.rows(),3);
  Mat im_color_aux;
  
  im_current_color.copyTo(im_color_aux);
  PointCloud<PointXYZRGB>  PointAuxRGB;
  
  projected_cloud_ext=MatrixXf::Ones(projected_cloud.rows(),4);
  
  int in=0;
  for(int k=0;k<projected_cloud.rows();k++)
  {
    
    cloud_on_image(k,0)=  cloud_matrix(k,0);
    cloud_on_image(k,1)=  cloud_matrix(k,1);
    cloud_on_image(k,2)=  cloud_matrix(k,2);
    
    projected_cloud_ext(k,0)=projected_cloud(k,0);
    projected_cloud_ext(k,1)=projected_cloud(k,1);
    projected_cloud_ext(k,2)=projected_cloud(k,2);
    
    projected_cloud(k,0)=projected_cloud(k,0)/projected_cloud(k,2);
    projected_cloud(k,1)=projected_cloud(k,1)/projected_cloud(k,2);
    
    ///add point with RGB
    setRGB(projected_cloud(k,0), projected_cloud(k,1), im_color_aux, PointAuxRGB,
	   cloud_on_image(k,0),cloud_on_image(k,1),cloud_on_image(k,2));
    
    cloud_RGB->points.push_back (PointAuxRGB.points[0]);
    
//     if ( (0<projected_cloud(k,0) && projected_cloud(k,0)<im_current_color.cols) &&
//       (0<projected_cloud(k,1) && projected_cloud(k,1)<im_current_color.rows))
//     {
      circle(im_current_color, Point(projected_cloud(k,0),projected_cloud(k,1)), 1, cv::Scalar(255, 0, 0), CV_FILLED, CV_AA);
      in=in+1;
  //  }
    
  }
  
  //   cout<<"point in"<<in<<endl;
  for(int k=0;k<projected_cloud_non_floor.rows();k++)
  {
    projected_cloud_non_floor(k,0)=projected_cloud_non_floor(k,0)/projected_cloud_non_floor(k,2);
    projected_cloud_non_floor(k,1)=projected_cloud_non_floor(k,1)/projected_cloud_non_floor(k,2);
    cloud_on_image_non_floor(k,0)=  cloud_matrix_non_floor(k,0);
    cloud_on_image_non_floor(k,1)=  cloud_matrix_non_floor(k,1);
    cloud_on_image_non_floor(k,2)=  cloud_matrix_non_floor(k,2);
    
    // circle(im_current_color, Point(projected_cloud(k,0),projected_cloud(k,1)), 1, cv::Scalar(255, 0, 0), CV_FILLED, CV_AA);
    
  }
  
  P_ext=MatrixXf::Zero(4,4);
  P_ext<< P_velo_to_img(0,0),P_velo_to_img(0,1),P_velo_to_img(0,2),P_velo_to_img(0,3),
   P_velo_to_img(1,0),P_velo_to_img(1,1),P_velo_to_img(1,2),P_velo_to_img(1,3),
   P_velo_to_img(2,0),P_velo_to_img(2,1),P_velo_to_img(2,2),P_velo_to_img(2,3),
   0, 0, 0, 1;
 
}

void Set_flow2d23d_kitty(MatrixXf projected_cloud_ext, 
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
		    const PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB)
{
  
  PointCloud<PointXYZ> P1,P2;
  PointXYZRGB prgb;
  
  PointCloud<PointXYZRGB>  PointAuxRGB;
  PointAuxRGB.points.resize(1);
  
  P1.points.resize(1);
  P2.points.resize(1);
  
  PointCloud<Normal> N_;
  N_.points.resize(1);
  
  flow_3d=MatrixXf::Zero(flow_projected.rows(),flow_projected.cols());
  P3D=MatrixXf::Zero(projected_cloud_ext.rows(),projected_cloud_ext.cols());
 
  float maxrad = -1;
  
  if (maxrad <= 0)
  {
    maxrad = 1;
    for (int y = 0; y < flow.rows; ++y)
    {
      for (int x = 0; x < flow.cols; ++x)
      {
	Point2f u = flow.at< Point2f>(y,x);
	if (!isFlowCorrect(u))
	  continue;
	maxrad = max(maxrad, sqrt(u.x * u.x + u.y * u.y));
      }
    }
  }
   
  int x,y;
  for (int i=0;i< flow_3d.rows();i++)
  {
    MatrixXf P_pse_in=P_ext.transpose()*(P_ext*P_ext.transpose()).inverse();
//      flow_3d.row(i)=(P_ext.inverse()*flow_projected.row(i).transpose()).transpose(); ///WORK WELL but try with pseudoinverse
//      P3D.row(i)=(P_ext.inverse()*projected_cloud_ext.row(i).transpose()).transpose();
     flow_3d.row(i)=(P_pse_in*flow_projected.row(i).transpose()).transpose(); ///WORK WELL but try with pseudoinverse
     P3D.row(i)=(P_pse_in*projected_cloud_ext.row(i).transpose()).transpose();
     
    x=round(projected_cloud_ext(i,0)/projected_cloud_ext(i,2));
    y=round(projected_cloud_ext(i,1)/projected_cloud_ext(i,2));
    
    Point2f fxy=flow.at< Point2f>(y,x);

    if (isFlowCorrect(fxy) )
    {
      P1.points[0].x=P3D(i,0);
      P1.points[0].y=P3D(i,1);
      P1.points[0].z=P3D(i,2);
  
      P2.points[0].x=flow_3d(i,0);
      P2.points[0].y=flow_3d(i,1);
      P2.points[0].z=flow_3d(i,2);   
   
      float norm_points=sqrt((P1.points[0].x-P2.points[0].x)*(P1.points[0].x-P2.points[0].x)+
      (P1.points[0].y-P2.points[0].y)*(P1.points[0].y-P2.points[0].y)+
      (P1.points[0].z-P2.points[0].z)*(P1.points[0].z-P2.points[0].z));
      
      N_.points[0].normal_x=(P2.points[0].x-P1.points[0].x)/norm_points;
      N_.points[0].normal_y=(P2.points[0].y-P1.points[0].y)/norm_points;
      N_.points[0].normal_z=(P2.points[0].z-P1.points[0].z)/norm_points;
      
      if((N_.points[0].normal_z!=0 && N_.points[0].normal_y!=0) || !isnan(norm_points) ){
	 cloud_p3d->push_back(P1.points[0]);
	 cloud_flow->push_back(P2.points[0]);
         Normals->push_back(N_.points[0]);
      }
      ///
      Vec3b  value=computeColor(fxy.x / maxrad, fxy.y / maxrad);
      ///
      prgb.x= P1.points[0].x;
      prgb.y= P1.points[0].y;
      prgb.z= P1.points[0].z;
      ///
      uint32_t rgb = (static_cast<uint32_t>(value[0]) << 16 
      | static_cast<uint32_t>(value[1]) << 8 | static_cast<uint32_t>(value[2]));
      prgb.rgb = *reinterpret_cast<float*>(&rgb);
      ///
      setRGB(x,y,im_color, PointAuxRGB,
	   P1.points[0].x, P1.points[0].y, P1.points[0].z);
      ///
      cloudRGB_flow->push_back(prgb);
      ///
      cloudRGB->push_back(PointAuxRGB.points[0]);
   
    }
    
  }
  

}

void cluster_pointcloud(const PointCloud<PointNormal>::Ptr Cloud_Normal, 
			const PointCloud<PointXYZRGB>::Ptr& Clusters,
			visualization_msgs::Marker Point_flows,
			visualization_msgs::Marker& points,
			visualization_msgs::Marker& line_list,
			visualization_msgs::MarkerArray& marker_arrow)


  {
    points.header.frame_id =  line_list.header.frame_id = "/velodyne";
    points.header.stamp =line_list.header.stamp = ros::Time::now();
    points.ns =  line_list.ns = "points_and_lines";
    points.pose.orientation.w =line_list.pose.orientation.w = 1.0;
    
    
    points.id = 0;
    line_list.id = 1;
    
    
    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;  
    
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x =line_list.scale.y=line_list.scale.z=0.1;
    
    // Points are blue
    points.color.r = 1.0f;
    points.color.a = 1.0;
    
    // Line list is green  
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;
    
    //random numbers
    srand ( time(NULL) );
    pcl::PointXYZRGB  prgb;
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud (Cloud_Normal);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
    //ec.setClusterTolerance (0.02); // 2cm
    ec.setClusterTolerance (1); // 1m
 //   ec.setMinClusterSize (100);///NOTE 100 value -->WORK WELL
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud ( Cloud_Normal);
    ec.extract (cluster_indices);
    
    int j = 0;
    marker_arrow.markers.resize(cluster_indices.size());
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      int r_rand = rand() % 255 + 1;
      int g_rand = rand() % 255 + 1;
      int b_rand = rand() % 255 + 1; 
      ///rand color
      uint8_t r = r_rand;
      uint8_t g = g_rand;
      uint8_t b = b_rand;  
      
      Normal n_mean;
      n_mean.normal_x=n_mean.normal_y=n_mean.normal_z=0;
      PointXYZ P_mean_normal; 
      P_mean_normal.x=P_mean_normal.y=P_mean_normal.z=0;
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
	
	PointXYZ P1_;
	
	P1_.x=Cloud_Normal->points[*pit].x;
	P1_.y=Cloud_Normal->points[*pit].y;
	P1_.z=Cloud_Normal->points[*pit].z;
	
	n_mean.normal_x=Cloud_Normal->points[*pit].normal_x+n_mean.normal_x;
	n_mean.normal_y=Cloud_Normal->points[*pit].normal_y+n_mean.normal_y;
	n_mean.normal_z=Cloud_Normal->points[*pit].normal_z+n_mean.normal_z;
	
	//         P_mean_normal.x=Point_flows.points[*pit].x+P_mean_normal.x;
	// 	P_mean_normal.y=Point_flows.points[*pit].y+P_mean_normal.y;
	// 	P_mean_normal.z=Point_flows.points[*pit].z+P_mean_normal.z;
	
	// 	/*cout<<"( "<<Cloud_Normal->points[*pit].normal_x<<
	// 	", "<<Cloud_Normal->points[*pit].normal_y<<", "<<Cloud_Normal->points[*pit].normal_z<<")"<<endl;
	// 	
	// 	cout<<"(..... "<*/<Cloud_Normal->points[*pit].getNormalVector4fMap()<<")"<<endl;
	
	cloud_cluster->points.push_back (P1_); 
	//  pcl::PointNormal p_;
	prgb.x=Cloud_Normal->points[*pit].x;
	prgb.y=Cloud_Normal->points[*pit].y;
	prgb.z=Cloud_Normal->points[*pit].z;
	
	int32_t rgb = (r << 16) | (g << 8) | b;
	prgb.rgb=rgb;
	
	Clusters->points.push_back(prgb);
	
	
      }
      pcl::PointXYZ  min_pt,max_pt;
      pcl::getMinMax3D (* cloud_cluster, min_pt, max_pt); 
      //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      geometry_msgs::Point p1,p2,p3;
      p1.x = min_pt.x+(max_pt.x-min_pt.x)/2; // Translation along the X axis
      p1.y = min_pt.y+(max_pt.y-min_pt.y)/2; // Translation along the Y axis
      p1.z = min_pt.z+(max_pt.z-min_pt.z)/2; // Translation along the Z axis
      
      p2.x =  n_mean.normal_x/cloud_cluster->points.size (); 
      p2.y =  n_mean.normal_y/cloud_cluster->points.size (); 
      p2.z =  n_mean.normal_z/cloud_cluster->points.size (); 
      
      p2.x=p2.x/sqrt(p2.x*p2.x+p2.y*p2.y+p2.z*p2.z);
      p2.y=p2.y/sqrt(p2.x*p2.x+p2.y*p2.y+p2.z*p2.z);
      p2.z=p2.z/sqrt(p2.x*p2.x+p2.y*p2.y+p2.z*p2.z);
      
      int val=1;
      p3.x=p1.x+p2.x*val;
      p3.y=p1.y+p2.y*val;
      p3.z=p1.z+p2.z*val;
      
//       std::cout << "P1 " << p1.x<<" ,"<<p1.y<<" ," <<p1.z << " ------ "<< std::endl;
//       std::cout << "Normal " << p2.x<<" ,"<<p2.y<<" ," <<p2.z << " ------ "<< std::endl;
//       //multiplication normal
//       std::cout << "Normal squared" << p2.x*p2.x+ p2.y*p2.y+p2.z*p2.z<< " ------ "<< std::endl;
      ///points
      points.points.push_back(p1);
      
      ///lines
      line_list.points.push_back(p1);
      line_list.points.push_back(p3);
      
      
      Vector4f p(p2.x,p2.y,p2.z,0);
      Vector4f q(p1.x,p1.y,p1.z,0);
      double angle_rad=pcl::deg2rad(pcl::getAngle3D(p,q));
      
      
      
      Quaternion<float> qua; 
      qua = AngleAxis<float>(angle_rad, Vector3f(p1.x,p1.y,p1.z));
      
      //       cout<<"angle "<<angle_rad<<""<<endl;
      //       cout<<"quaternion  "<<qua.w()<<""<<endl;
      
      
      
      ///arrows
      marker_arrow.markers[j].header.frame_id= "/velodyne";
      
      marker_arrow.markers[j].header.stamp=ros::Time::now();
      marker_arrow.markers[j].scale.x = 0.2;
      marker_arrow.markers[j].scale.y = 0.2;
      marker_arrow.markers[j].scale.z = 0.2;
      marker_arrow.markers[j].color.r = 1.0f;	      
      marker_arrow.markers[j].color.g = 0.0f;
      marker_arrow.markers[j].color.b = 0.0f;
      marker_arrow.markers[j].color.a = 0.5f;
      marker_arrow.markers[j].pose.position = p1;
      marker_arrow.markers[j].ns="normals";
      marker_arrow.markers[j].action=visualization_msgs::Marker::ADD;
      marker_arrow.markers[j].id = 2;
      marker_arrow.markers[j].type=visualization_msgs::Marker::ARROW;
      marker_arrow.markers[j].points.push_back(p1);
      marker_arrow.markers[j].points.push_back(p2);
      marker_arrow.markers[j].pose.orientation.w=0;
      marker_arrow.markers[j].pose.orientation.x=0;
      marker_arrow.markers[j].pose.orientation.y=0;
      marker_arrow.markers[j].pose.orientation.z=0;
      
      // tf::quaternionTFToMsg(qua, quat_msg);
      //   marker_arrow.markers[j].pose.orientation.w;
      
      // Vector4f r_=p.dot(q);
      
      
      
      //       Vector3f axis(p2.x,p2.y,p2.z);
      //       Vector3f marker_axis(1, 0, 0);
      // //       Vector3f::
      // //       btQuaternion qt(marker_axis.cross(axis.normalize()), marker_axis.angle(axis.normalize()));
      // //       geometry_msgs::Quaternion quat_msg;
      //       Vect
      //       axis.
      //       marker_arrow.markers[j].pose.orientation.w = quat_msg;
      
      
      j++;
  }
  
  
  }
