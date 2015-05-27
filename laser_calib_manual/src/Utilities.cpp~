
#include "Utilities.h"

 #include "viewer_data.h"

#include "image_data.h"

int Get3d_2dPoints(PointCloud<PointXYZ>::ConstPtr  cloud_p, 
	            PointCloud<PointXYZ>::ConstPtr  cloud_p2, 
		    MatrixXf &x3d, 
		    MatrixXf &x2d,   
		   vector <Point3f>& points3d,
		    vector <Point2f>& imagePoints){

 x3d= MatrixXf::Ones(cloud_p->points.size (),4);
  x2d= MatrixXf::Ones(cloud_p2->points.size (),3);
  std::cout<<"points selected \n"<<cloud_p->points.size ()<<std::endl;
  
  
  if (cloud_p->points.size ()!=cloud_p2->points.size ()){
   
    
    return (-1);
  }
  
  int i,j;  
  
   
  for (i=0;i<cloud_p->points.size ();i++){
    
    x3d(i,0)=cloud_p->points[i].x;
    x3d(i,1)=cloud_p->points[i].y;
    x3d(i,2)=cloud_p->points[i].z;
    x3d(i,3)=1;   
    
    points3d.push_back(Point3f(x3d(i,0), x3d(i,1), x3d(i,2)));
    
    
  }
  
  for (i=0;i<cloud_p2->points.size ();i++){
    
    x2d(i,0)=cloud_p2->points[i].x;
    x2d(i,1)=cloud_p2->points[i].y;
    x2d(i,2)=cloud_p2->points[i].z;
    
     imagePoints.push_back(Point2f(x2d(i,0), x2d(i,1)));
  }
  
  return (0);
  
  
}

void Lu_method(MatrixXf x3d, MatrixXf x2d, MatrixXf &R,Vector3f &t, MatrixXf &xcam_, MatrixXf K,
  const PointCloud<PointXYZRGB>::Ptr& cloudRGB,//projected data
  const PointCloud<PointXYZ>::Ptr& cloud_laser_cord,//laser coordinates
  MatrixXf& points_projected, Mat image,PointCloud<PointXYZ>::ConstPtr cloud ){

   
  Lu obpose;
  
  obpose.Initialize(x3d,x2d,K);

  
  //compute method

  obpose.compute();

  
  //getting results

  R=obpose.GetR();// rotation

  t=obpose.Gett();// translation

  xcam_=obpose.Getxcam(); // points of the pose
  
  cout<<"pasa?"<<endl;
  points_projected=MatrixXf::Zero(2,cloud->points.size ());
  cout<<"paso"<<endl;
  for (int j=0;j<cloud->points.size ();j++){


    PointCloud<PointXYZRGB> PointAuxRGB;

    PointAuxRGB.points.resize(1);

    Vector3f xw=Vector3f(cloud->points[j].x,

		cloud->points[j].y,

		cloud->points[j].z);

    

    xw=K*( R*xw+t);

    xw= xw/xw(2);

    
    
    

    int col,row;

    

    col=(int)xw(0);

    row=(int)xw(1);

    
    points_projected(0,j)=(int)xw(0);
    points_projected(1,j)=(int)xw(1);
    

    

    
    
    int b,r,g;

    if ((col<0 || row<0) || (col>image.cols || row>image.rows)){

      r=0;

      g=0;

      b=0;

      

    }else{

      

    //  b = img->imageData[img->widthStep * row+ col * 3];

     // g = img->imageData[img->widthStep * row+ col * 3 + 1];

      //r = img->imageData[img->widthStep * row+ col * 3 + 2];    

      

      r=image.at<cv::Vec3b>(row,col)[0];

      g=image.at<cv::Vec3b>(row,col)[1];

      b=image.at<cv::Vec3b>(row,col)[2];

      //std::cout << image.at<cv::Vec3b>(row,col)[0] << " " << image.at<cv::Vec3b>(row,col)[1] << " " << image.at<cv::Vec3b>(row,col)[2] << std::endl;



    }

    

    

    //std::cout << "( "<<r <<","<<g<<","<<b<<")"<<std::endl;

    uint8_t r_i = r;

    uint8_t g_i = g;

    uint8_t b_i = b;

    

    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

    

    //int32_t rgb = (r_i << 16) | (g_i << 8) | b_i;

    

    

    PointAuxRGB.points[0].x=cloud->points[j].x;

    PointAuxRGB.points[0].y=cloud->points[j].y;

    PointAuxRGB.points[0].z=cloud->points[j].z;

    PointAuxRGB.points[0].rgb=*reinterpret_cast<float*>(&rgb);  

    

    cloudRGB->points.push_back (PointAuxRGB.points[0]);

    //project point to the image

    
		

  }
  
}

void Lu_method(MatrixXf x3d, MatrixXf x2d, MatrixXf &R,Vector3f &t, MatrixXf &xcam_, MatrixXf K){

   
  Lu obpose;
  
  obpose.Initialize(x3d,x2d,K);

  
  //compute method

  obpose.compute();

  
  //getting results

  R=obpose.GetR();// rotation

  t=obpose.Gett();// translation

  xcam_=obpose.Getxcam(); // points of the pose
  
  
}


void SolvePNP_method(vector <Point3f> points3d,
		    vector <Point2f> imagePoints, 
		     MatrixXf &R,Vector3f &t, MatrixXf &xcam_, 
		     MatrixXf K,
                     const PointCloud<PointXYZRGB>::Ptr& cloudRGB,//projected data
                    const PointCloud<PointXYZ>::Ptr& cloud_laser_cord,//laser coordinates
                     MatrixXf& points_projected,Mat image,
		     PointCloud<PointXYZ>::ConstPtr cloud)
{
   
  
  Mat_<float> camera_matrix (3, 3);
  camera_matrix(0,0)=K(0,0);
  camera_matrix(0,1)=K(0,1);
  camera_matrix(0,2)=K(0,2);
  camera_matrix(1,0)=K(1,0);
  camera_matrix(1,1)=K(1,1);
  camera_matrix(1,2)=K(1,2);
  camera_matrix(2,0)=K(2,0);
  camera_matrix(2,1)=K(2,1);
  camera_matrix(2,2)=K(2,2);
  
  
  
  
  vector<double> rv(3), tv(3);

  Mat rvec(rv),tvec(tv);

  Mat_<float> dist_coef (5, 1);
   
  dist_coef = 0;
  
  
  cout <<camera_matrix<<endl;
  cout<< imagePoints<<endl;
  cout<< points3d <<endl;
  
  solvePnP( points3d, imagePoints, camera_matrix, dist_coef, rvec, tvec);
  
  //////////////////////////////////////////////////7
  //convert data
  //////////////////////////////////////////////////7
  
  double rot[9] = {0};

  
  Mat R_2(3,3,CV_64FC1,rot);

  //change results only debugging

  Rodrigues(rvec, R_2);

  
  R=Matrix3f::Zero(3,3);
  t=Vector3f(0,0,0);
  
  double* _r = R_2.ptr<double>();

  double* _t = tvec.ptr<double>();

  
  t(0)=_t[0];

  t(1)=_t[1];

  t(2)=_t[2];   

  
  R(0,0)=_r[0];

  R(0,1)=_r[1];

  R(0,2)=_r[2];

  R(1,0)=_r[3];

  R(1,1)=_r[4];

  R(1,2)=_r[5];

  R(2,0)=_r[6];

  R(2,1)=_r[7];

  R(2,2)=_r[8];

  
  
  points_projected=MatrixXf::Zero(2,cloud->points.size ());
  
  for (int j=0;j<cloud->points.size ();j++){

    

    

    PointCloud<PointXYZRGB> PointAuxRGB;

    PointAuxRGB.points.resize(1);

    

    Vector3f xw=Vector3f(cloud->points[j].x,

		cloud->points[j].y,

		cloud->points[j].z);

    

    xw=K*( R*xw+t);

    xw= xw/xw(2);

    

    int col,row;

    

    col=(int)xw(0);

    row=(int)xw(1);

    

     points_projected(0,j)=(int)xw(0);
    points_projected(1,j)=(int)xw(1);
    

    int b,r,g;

    if ((col<0 || row<0) || (col>image.cols || row>image.rows)){

      r=0;

      g=0;

      b=0;

      

    }else{

      

    //  b = img->imageData[img->widthStep * row+ col * 3];

     // g = img->imageData[img->widthStep * row+ col * 3 + 1];

      //r = img->imageData[img->widthStep * row+ col * 3 + 2];    

      

      r=image.at<cv::Vec3b>(row,col)[0];

      g=image.at<cv::Vec3b>(row,col)[1];

      b=image.at<cv::Vec3b>(row,col)[2];

      //std::cout << image.at<cv::Vec3b>(row,col)[0] << " " << image.at<cv::Vec3b>(row,col)[1] << " " << image.at<cv::Vec3b>(row,col)[2] << std::endl;



    }

    

    

    //std::cout << "( "<<r <<","<<g<<","<<b<<")"<<std::endl;

    uint8_t r_i = r;

    uint8_t g_i = g;

    uint8_t b_i = b;

    

    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

    

    //int32_t rgb = (r_i << 16) | (g_i << 8) | b_i;

    

    

    PointAuxRGB.points[0].x=cloud->points[j].x;

    PointAuxRGB.points[0].y=cloud->points[j].y;

    PointAuxRGB.points[0].z=cloud->points[j].z;

    PointAuxRGB.points[0].rgb=*reinterpret_cast<float*>(&rgb);  

    

    cloudRGB->points.push_back (PointAuxRGB.points[0]);

    //project point to the image

    
		

  }
  
}

void SortPoints(MatrixXf &x2d)
{
  // the array is formed by [x1 y1 1,
//                             x2 y2  ]
  
  
  MatrixXf x2d_aux=MatrixXf::Zero(x2d.rows(),3);

  //////////////////////////////////////////////////////////////
  //first sort point in y
  ///////////////////////////////////////////////////////////////
  
  int i,j;
  for (i =0; i<x2d.rows();i++){
    for(j = i+1; j < x2d.rows(); j ++) {
        if(x2d(j,1) < x2d(i,1)) {
               float temp_x = x2d(i,0);
	       float temp_y = x2d(i,1);
               x2d(0,i) = x2d(0,j);
	       x2d(1,i) = x2d(1,j);
	       
	       x2d(0,j) = temp_x;
	       x2d(1,j) = temp_y;
               
               
	}
    
    }
  }
 
 /////////////////////////////////////////
  //now order in X
 
  for (i =0; i<x2d.rows();i++){
    for(j = i+1; j < x2d.rows(); j ++) {
        if(x2d(j,0) < x2d(i,0)) {
               float temp_x = x2d(i,0);
	       float temp_y = x2d(i,1);
               x2d(0,i) = x2d(0,j);
	       x2d(1,i) = x2d(1,j);
	       
	       x2d(0,j) = temp_x;
	       x2d(1,j) = temp_y;
               
               
	}
    
    }
  }
  
  
}

void SortPointsAngle(MatrixXf &x2d){
  
    // the array is formed by [x1 y1 1,
//                             x2 y2  ]
  
     // compute center point
     Vector3f point=x2d.colwise().sum();
     
     point=point/x2d.rows();
     
     vector<float> angles;
     int i,j;
     // compute angles
     for (i=0; i<x2d.rows();i++){
       
       double x=(point(0)-x2d(i,0));
       double y=(point(1)-x2d(i,1));
       
       angles.push_back(atan2(y,x)*180 / (3.1416));//degree in radians
       
     }
     
     //////////////////////////////////////////////////////
     // now order by angle chossing the minor to mayor
     
       for (i =0; i<x2d.rows();i++){
          for(j = i+1; j < x2d.rows(); j ++) {
                if(angles[j] < angles[i]) {
		  
                     float temp_x = x2d(i,0);
	             float temp_y = x2d(i,1);
                      x2d(i,0) = x2d(j,0);
	              x2d(i,1) = x2d(j,1);
	       
	              x2d(j,0) = temp_x;
	              x2d(j,1) = temp_y;
               
               
	           }
    
           }
       }
     
}

void SaveCalibrationData(MatrixXf r,Vector3f  t_2){
  
  int i;
  //Mat cameraMatrix = (Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
  
  Mat R= (Mat_<double>(3,3) << r(0,0),r(0,1),r(0,2), r(1,0),r(1,1),r(1,2),r(2,0),r(2,1),r(2,2));
  
  Mat t= (Mat_<double>(1,3) << t_2(0),t_2(1),t_2(2));
  
  FileStorage fs("test.yml", FileStorage::WRITE);
  
  
  fs << "Rotation" << R << "Translation" << t;
  
  fs.release();
  
  cout<<" data saved "<<endl;
  
  
}


void OpenYalm(const string Name, Matrix3f &K){
  
  FileStorage fs;
   
  fs.open(Name, FileStorage::READ);
  
  Mat K2=Mat_<double > (3, 3);
  
  
  if (!fs.isOpened()){
            cerr << "Failed to open " << Name << endl;
            
            return ;
   }
        
  fs["camera_matrix"] >> K2; 
  
   cout<<"files**************** "<<Name.c_str()<<endl;
  
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
  
  cout<<"files**************** "<<Name.c_str()<<endl;
  
  cout<<"K--->"<<K2<<endl;
  cout<<"P--->"<<K3<<endl;
  
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

void PickingEventOccurred_(const visualization::PointPickingEvent& event, void* obj_void)

{
  viewer_data* obj=(viewer_data*) obj_void;
  
 
  
//   boost::shared_ptr<visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<
// 
// 
//   visualization::PCLVisualizer> *>(viewer_void);


  obj->indx = event.getPointIndex ();

  
  if (obj->indx == -1)
    return;//there is no point

  vector<int> indices (1);


  vector<float> distances (1);



  pcl::PointXYZ   PointAux;


  event.getPoint ( PointAux.x,  PointAux.y,  PointAux.z);


  char str[512];


  sprintf(str, "sphere_%d",obj->indx);

  std::cout << " (" << PointAux.x << ", " << PointAux.y <<", " << PointAux.z<< "......)"


  << str<<std::endl;


  //save point
   obj->cloud_p->points.push_back (PointAux);


  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(obj->cloud_p, 255, 0, 0);


  //viewer_void->addSphere(PointAux, str);


 obj->viewer->addPointCloud<pcl::PointXYZ>(obj->cloud_p,single_color, str,0);


  obj->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,str);


}

void OnMouse(int evt, int x, int y, int flags, void* param){
  
  image_data* obj=(image_data*) param;
  
    if(evt==CV_EVENT_LBUTTONDOWN){
        cout<<"("<<x <<" "<<y<<")"<<endl;
	circle(obj->img, Point(x,y), 3, cv::Scalar(0, 255, 0), CV_FILLED, CV_AA);
	obj->points.push_back (Point2f(x,y));
	//obj->points.
	imshow("image",obj->img);
    }
    
}

void project2cloud( MatrixXf R,Vector3f t, MatrixXf xcam_, MatrixXf K,
  const PointCloud<PointXYZRGB>::Ptr& cloudRGB,//projected data
  const PointCloud<PointXYZ>::Ptr& cloud_laser_cord,//laser coordinates
  MatrixXf& points_projected,//projected point in the image
  Mat &image,//image with points projected
  PointCloud<PointXYZ>::ConstPtr cloud //cloud to rpoject
)
{
  
  points_projected=MatrixXf::Zero(2,cloud->points.size ());
  cout<<"paso"<<endl;
  for (int j=0;j<cloud->points.size ();j++)
  {
    PointCloud<PointXYZRGB> PointAuxRGB;
    PointAuxRGB.points.resize(1);
    Vector3f xw=Vector3f(cloud->points[j].x,

		cloud->points[j].y,

		cloud->points[j].z);
    xw=K*( R*xw+t);
    xw= xw/xw(2);
    int col,row;

    
    col=(int)xw(0);
    row=(int)xw(1);

    
    points_projected(0,j)=(int)xw(0);
    points_projected(1,j)=(int)xw(1);

    points_projected(0,j)=xw(0);
    points_projected(1,j)=xw(1);
    int b,r,g;

    if ((col<0 || row<0) || (col>image.cols || row>image.rows))
    {

      r=0;
      g=0;
      b=0;
    }else
    {
      r=image.at<cv::Vec3b>(row,col)[0];
      g=image.at<cv::Vec3b>(row,col)[1];
      b=image.at<cv::Vec3b>(row,col)[2];
      cout << "( "<<r <<","<<g<<","<<b<<")"<<std::endl;

    }

   

    uint8_t r_i = r;
    uint8_t g_i = g;
    uint8_t b_i = b;
    
    
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);


    PointAuxRGB.points[0].x=cloud->points[j].x;
    PointAuxRGB.points[0].y=cloud->points[j].y;
    PointAuxRGB.points[0].z=cloud->points[j].z;
    PointAuxRGB.points[0].rgb=*reinterpret_cast<float*>(&rgb);  

    cloudRGB->points.push_back (PointAuxRGB.points[0]);

    circle(image, Point(xw(0),xw(1)), 3, cv::Scalar(255, 0, 0), CV_FILLED, CV_AA);

  }
  
}

void SegmentData(PointCloud<PointXYZ>::ConstPtr cloud_,
		const PointCloud<PointXYZ>::Ptr &  cloud_filtered ){
  
  
   //first are removed sporius points
   PointCloud<PointXYZ>::Ptr cloud_stastic (new PointCloud<PointXYZ>);
   
  //segment in Z
    //now the cloud is filtered in heigh and
   PassThrough<PointXYZ> pass;
   pass.setInputCloud (cloud_);
   pass.setFilterFieldName ("z");// here we have to see where x and z
  // pass.setFilterLimits (-1.0, 10.5);//see for laas
   pass.setFilterLimits (-1.5, 1.5);//for iri lab
   pass.filter (*cloud_filtered);
   PCL_INFO ("Removed x points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
   
     
     //now the cloud is filtered x

   pass.setInputCloud (cloud_filtered);
   pass.setFilterFieldName ("x");
  // pass.setFilterLimits (0, 6.0);//laas
   pass.setFilterLimits (2, 9.0);//iri // 2-7 mts
   pass.filter (*cloud_filtered);
   PCL_INFO ("Removed x points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
   
   
   
   //filter in y
   pass.setInputCloud (cloud_filtered);
   pass.setFilterFieldName ("y");
   //pass.setFilterLimits (0.5, 3);//laas
   pass.setFilterLimits (-2.0, 1.5);//iri
   pass.filter (*cloud_filtered);
   PCL_INFO ("Removed y points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
   
   StatisticalOutlierRemoval<PointXYZ> sor;
   sor.setInputCloud (cloud_filtered);
   sor.setMeanK (10);
   sor.setStddevMulThresh (1.0);
   sor.filter (*cloud_filtered);
   
   PCL_INFO ("Removed sporious points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
   

   
}