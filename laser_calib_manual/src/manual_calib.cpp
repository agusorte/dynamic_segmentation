#include "manual_calib.h"


manual_calib::manual_calib(string file,string method)
{
  
     method_=method;

  
   
     K=Matrix3f::Zero();
     P=MatrixXf::Zero(3,4);
   
     //openfile
     OpenYalm(file, K,P);
     
     cout<<"k\n"<<K<<endl;
      cout<<"P\n"<<P<<endl;
     cloud_topic_ = "input";
     image_topic_ = "image";
     
    
    

     sub_=nh_.subscribe (cloud_topic_, 1,  & manual_calib::cloud_Callback , this);
     
}

void manual_calib::image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
     
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
     
     
     

   ////////////////////////////////////////////////////////////////////////////////
   /// selection manual
   //////////////////////////////////////////////////////////////////////////////// 
   //image_data  obj_im_data;
   
   
   Mat im=cv_ptr->image.clone();
   
   im.copyTo(obj_img_data.img);
   cv::namedWindow( "image", 0 );
   cv::setMouseCallback("image",OnMouse,&obj_img_data);
   cv::imshow("image",im);
   
    cv::waitKey(0);
    
    x2d=MatrixXf::Ones(obj_img_data.points.size(),3);
    for (int j=0; j<obj_img_data.points.size() ; j++)
    {
      x2d(j,0)=obj_img_data.points[j].x;
      x2d(j,1)=obj_img_data.points[j].y;
      
    }
    
   /// save point to x2d  
    
       if (method_.compare("LU")==0){ // apply lu method

	    cout<<"x2d\n " <<x2d<<endl;

	    cout<<"x3d\n"<< x3d<<endl;

  
 	  Lu_method(x3d, x2d, R,t, xcam_, K);
         //visualize results
	  cout<<"R "<<R<<endl;
	  cout<<"t "<<t<<endl;
	   cout<<"xcam "<<xcam_<<endl;

	      
       }
       
       ////////////////////////////////////////////////////////////////////////////////
       /// project 3d point in image
       ///////////////////////////////////////////////////////////////////////////////
        PointCloud<PointXYZRGB>::Ptr cloudRGB (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZ>::Ptr cloud_laser_cord (new PointCloud<PointXYZ>);
	MatrixXf points_projected;
	
       project2cloud(  R, t, xcam_,  K, cloudRGB,//projected data
              cloud_laser_cord,//laser coordinates
               points_projected,//projected point in the image
  im,//image to project 
  cloud_ //cloud to rpoject
);
       
        //////////////////////////////////////////////////////
       /// show image with point projected
       ///////////////////////////////////////////////////////
       imshow("image",im);
       waitKey(0);
       
       //////////////////////////////////////////////////////////////////////////////
       /// show rgb cloud
       ////////////////////////////////////////////////////////////////////////////
      boost::shared_ptr<visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
   

     
     viewer->addPointCloud<PointXYZRGB>(cloudRGB,"cloud",0);
 //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5,"cloud");
     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4, "cloud");
     
     
     while (!viewer->wasStopped ())
     {
       viewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
     } 
       // we desactivate images and activate laser if they want to continue calibrating
       sub_.shutdown ();
       
       
       string answer;
       cout<<"do you want to calibrate another laser [Y|N]?"<<endl;
       cin>>answer;
       if(answer.compare("Y")==-1)
       sub_=nh_.subscribe (cloud_topic_, 1,  & manual_calib::cloud_Callback , this);
       else
	ros::shutdown();
     
  
   }
   
   
   ////////////////////////////////////////////////////////////////////////////////
   
   // Callback
   
 void manual_calib::cloud_Callback (const sensor_msgs::PointCloud2ConstPtr& cloudI)
 {
 
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

      
     
     if ((cloudI->width * cloudI->height) == 0)
              return;
 
     ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
	       
	       (int)cloudI->width * cloudI->height,
	       
	       cloudI->header.frame_id.c_str (),
	       
	       pcl::getFieldsList (*cloudI).c_str ());
     
     pcl::fromROSMsg(*cloudI, *cloud);
  
     ///Segment data ?????????????????/
     SegmentData(cloud,cloud);//to debug after eliminate if you want the complete cloud
     
     ////////////////////////////////////////////////////////////////////
     // selecttion manual
     ////////////////////////////////////////////////////////////////////
     
     pcl::PointCloud<PointXYZ>::Ptr cloud_p (new PointCloud<PointXYZ> ());
   
     boost::shared_ptr<visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
     viewer->setBackgroundColor (0, 0, 0);
 //  viewer->addCoordinateSystem (1.0);
     viewer->initCameraParameters ();
     viewer->setCameraPose (-5,-5,1,1,0,0,0,0,1);
   
     objviewer_data.x3d=&x3d;
     objviewer_data.viewer=viewer;
     objviewer_data.cloud_p=cloud_p;
     
     viewer->registerPointPickingCallback (PickingEventOccurred_ ,& objviewer_data);
     // 					(void*)&viewer);
     
     cloud_=cloud;
     
     viewer->addPointCloud<PointXYZ>(cloud,"cloud",0);
     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.7, 0.7, 0.7,"cloud");
     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2, "cloud");
     PCL_INFO("Press SHIFT+CLICK to select the points \n  and close the window to continue");
     
     while (!viewer->wasStopped ())
     {
       viewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
     } 
     
     PCL_INFO("cloud P size point %d \n ",objviewer_data.cloud_p->size());
     
     ///pass to point x3d
     
     x3d=MatrixXf::Ones(cloud_p->size(),4);
     for(int j=0;j<objviewer_data.cloud_p->size();j++)
     {
       
       x3d(j,0)=objviewer_data.cloud_p->points[j].x;
      x3d(j,1)=objviewer_data.cloud_p->points[j].y;
      x3d(j,2)=objviewer_data.cloud_p->points[j].z;
      
       
     }
     /////////////////////////////////////////////////////////////////////////
     //subscribe image
     /////////////////////////////////////////////////////////////////////////
     sub_.shutdown();
     sub_=nh_.subscribe (image_topic_, 1,  & manual_calib::image_Callback , this);
     
 }
 

   