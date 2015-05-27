#include "Chessboad_calibration.h" 

Chessboad_calibration::Chessboad_calibration(string file, string selection, string method, int h, int w){
     // Check if a prefix parameter is defined for output file names.
     
     selection_=selection;
     method_=method;
//      h_=h;
//      w_=w;
     
     boardSize.height=h;

     boardSize.width=w;
     
     
     K=Matrix3f::Zero();
     P=MatrixXf::Zero(3,4);
     
     
     //openfile
     OpenYalm(file, K,P);
     
     cout<<"k\n"<<K<<endl;
      cout<<"P\n"<<P<<endl;
     cloud_topic_ = "input";
     image_topic_ = "image";

     
     sub_=nh_.subscribe (cloud_topic_, 1,  & Chessboad_calibration::cloud_Callback , this);
     
     
   } 
   

void Chessboad_calibration::image_Callback(const sensor_msgs::ImageConstPtr& msg){
     
     cv_bridge::CvImageConstPtr cv_ptr;
     
     try
     
     {
       
       if (enc::isColor(msg->encoding))
	 
	 cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
       
       else
	 
	 cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);

       ROS_INFO("Image data size (%d, %d)",cv_ptr->image.rows, cv_ptr->image.cols);
       
     }
     
     catch (cv_bridge::Exception& e)
     
     {
       
       ROS_ERROR("cv_bridge exception: %s", e.what());
       
       return;
       
     }
     
     
       int found;
       
       Mat ima=cv_ptr->image.clone();
    if (selection_.compare("AUTO")==0){
      
       found=calib_Auto_Image(ima);
   
     }

//    if (selection_.compare("MANUAL")==0){
// 
//        calib_Manual();
// 
//      }
     
   
     // Process cv_ptr->image using OpenCV
     
     imshow( "Image2", ima);  

     waitKey(0);
	 
     
     if(found!=-1){
       // calibrate data
       if (method_.compare("LU")==0){ // apply lu method

	    cout<<"x2d\n " <<x2d<<endl;

	    cout<<"x3d\n"<< x3d<<endl;

    
    
    
    
 	  Lu_method(x3d, x2d, R,t, xcam_, K) ;
         //visualize results
	  cout<<"R "<<R<<endl;
	  cout<<"t "<<t<<endl;
	   cout<<"xcam "<<xcam_<<endl;

	      
       }
       
       // we desactivate images and activate laser if they want to continue calibrating
       sub_.shutdown ();
       
       
       string answer;
       cout<<"do you want to calibrate another laser [Y|N]?"<<endl;
       cin>>answer;
       if(answer.compare("Y")==-1)
       sub_=nh_.subscribe (cloud_topic_, 1,  & Chessboad_calibration::cloud_Callback , this);
       else
	 ros::shutdown();
     }
     
     
     
     
     
   }
   
   
   ////////////////////////////////////////////////////////////////////////////////
   
   // Callback
   
 void Chessboad_calibration::cloud_Callback (const sensor_msgs::PointCloud2ConstPtr& cloudI){
     
     
  
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

      
     
     if ((cloudI->width * cloudI->height) == 0)
              return;
     
     
     
     ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
	       
	       (int)cloudI->width * cloudI->height,
	       
	       cloudI->header.frame_id.c_str (),
	       
	       pcl::getFieldsList (*cloudI).c_str ());
     
     
     pcl::fromROSMsg(*cloudI, *cloud);
     
     ////////////////////////////////////////////////////////////////////////
     // here depends of the actions AUTO or MANUAL
     ///////////////////////////////////////////////////////////////////////
    /* 
    
     if (selection_.compare("MANUAL")==0){
       
       
     }*/
    
    
    int found;
    if (selection_.compare("AUTO")==0){
      
      found=calib_Auto_Laser(cloud);
      
     }
    
     
   
    if(found!=-1){ //we disactivate the lase messages and activate images 
     sub_.shutdown();
     
     
     sub_=nh_.subscribe (image_topic_, 1,  & Chessboad_calibration::image_Callback , this);
    }
   
      
   }
   
   
 void Chessboad_calibration::calib_Manual(){
     
//      boost::shared_ptr<visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  
//      viewer->setBackgroundColor (0, 0, 0);
//      viewer->addCoordinateSystem (1.0);
// 
//      
//       viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);
// 
//      viewer->registerPointPickingCallback (PickingEventOccurred_ ,
// 					(void*)&viewer);
// 
//      
//      viewer->addPointCloud<pcl::PointXYZ>(cloud,"sample cloud",0);
// 
//      
//      while (!viewer->wasStopped ()){
// 
//        viewer->spinOnce (100);
// 
//      }
// 
//      

   
 }
 
 
 int Chessboad_calibration::calib_Auto_Laser(PointCloud<PointXYZ>::ConstPtr cloud_){
   
   pcl::PointCloud<PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ> ());
     pcl::PointCloud<PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
     
     
     ///////////////////////////////////////////////////////
     // variables for segmenting data
     ///////////////////////////////////////////////////////
     
     pcl::PointCloud<PointXYZ>::Ptr cloud_nonPlanes (new PointCloud<PointXYZ> ());
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Planes(new PointCloud<PointXYZ> ());
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new PointCloud<pcl::PointXYZ>);
     
     
     ///////////////////////////////////////////////////////
     // Clustering data
     ///////////////////////////////////////////////////////
     
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB_cluster (new PointCloud<PointXYZRGB>);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_chessboard (new PointCloud<PointXYZ>);
     
     
     ///////////////////////////////////////////////////////////7
     // plane extraction
     /////////////////////////////////////////////////////////////////
     PointCloud<PointXYZ>::Ptr cloud_plane_cluster (new PointCloud<PointXYZ>);
     PointCloud<PointXYZ>::Ptr cloud_nonplane_cluster (new PointCloud<PointXYZ>);
     PointCloud<PointXYZ>::Ptr cloud_plane_projected (new PointCloud<PointXYZ>);
     PointCloud<PointXYZ>::Ptr cloud_chessboard_square(new PointCloud<PointXYZ>);
     PointCloud<PointXYZ>::Ptr cloud_hull (new PointCloud<PointXYZ>);

     
     
   
      visualization::PCLVisualizer visualizer ("Online PointCloud2 Viewer");	   
     /// first find the 3d chessboard

     objfindboard.FindPointsChessboard3D(cloud_,

	                                     cloud_filtered,

					     cloud_Planes,

					      cloud_nonPlanes,

					      cloud_nonplane_cluster,

					      cloud_projected,

					      cloudRGB_cluster,

					      cloud_chessboard,

					      cloud_plane_cluster,

					      cloud_plane_projected,

					      cloud_hull,

					      cloud_chessboard_square);

     

     

     /// only debug visualize data

//      pcl::visualization::PCLVisualizer viewer5("3D Viewer");

     objfindboard.DrawClouds(visualizer ,

	     cloud_,

	     cloud_filtered,

	     cloud_Planes,

	     cloud_nonPlanes,

              cloud_projected,

	     cloudRGB_cluster,

	     cloud_chessboard,

	     cloud_plane_cluster,

	     cloud_plane_projected,

	     cloud_hull,

	     cloud_chessboard_square);
     
       while (! visualizer.wasStopped ()){
	visualizer.spin();
      }
      
      
      string answer;
      cout<<"the pattern was recognized automatically [Y|N]\n\n";
      cin>>answer;
      
     
     if(answer.compare("Y")==0){
      objfindboard.AssociatePoints(image1, cloud_chessboard_square, 
		      x3d);
       
      
    }else
       return (-1);
    
    return (0);
		      
		     
 }
 
 int Chessboad_calibration::calib_Auto_Image(Mat &image_){
     
   
   string answer;
   
   cout<<"now checking the new method"<<endl;

   
   if(objfindimageboard.Find_image_chessboard(image_,  boardSize, x2d))
   {

  
     
      string answer;
      cout<<"the pattern was recognized automatically [Y|N]\n\n";
      cin>>answer;
    if(  answer.compare("Y")==0)
       return (0);
    else
       return (-1);
     
     
   }else
{
     cout<<"chessboard was not found"<<endl;

     return(-1);
   } 
 
   
 }
 

 
