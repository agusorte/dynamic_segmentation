#include "process_cloud.h"
#include "pcl/io/io.h"

#include "pcl/io/pcd_io.h"

process_cloud::process_cloud()
{
  
  //viewer=new visualization::PCLVisualizer(pcl::visualization::PCLVisualizer("3D Viewer"));
  
  //intialize the variables visuation
  boost::shared_ptr<visualization::PCLVisualizer> viewer_aux(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer= viewer_aux;
  viewer->setBackgroundColor (0, 0, 0);
  
  viewer->addCoordinateSystem (1.0);
  
  
  viewer->initCameraParameters ();
  viewer->addCoordinateSystem(1.0);
  viewer->setCameraPose (-10,0,0,1,0,0,0,0,1);
  /** \brief sets the camera pose given by position, viewpoint and up vector
   * \param posX the x co-ordinate of the camera location
   * \param posY the y co-ordinate of the camera location
   * \param posZ the z co-ordinate of the camera location
   * \param viewX the x component of the view upoint of the camera
   * \param viewY the y component of the view point of the camera
   * \param viewZ the z component of the view point of the camera
   * \param upX the x component of the view up direction of the camera
   * \param upY the y component of the view up direction of the camera
   * \param upZ the y component of the view up direction of the camera
   * \param viewport the viewport to modify camera of, if 0, modifies all cameras*/
  
  /////////////////////////////////////////////////////////////////////////
  //put the sensor in a fixed position
  
  //range image
  
  boost::shared_ptr<pcl::visualization::RangeImageVisualizer> range_image_widget_aux(new pcl::visualization::RangeImageVisualizer("Range image"));
  range_image_widget=range_image_widget_aux;
  
  boost::shared_ptr<pcl::RangeImage> range_image_aux(new pcl::RangeImage);
  range_image=range_image_aux;
  
  //for range images
  sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  coordinate_frame = pcl::RangeImage::LASER_FRAME;
  noiseLevel=0.00;
  minRange = 0.0f;
  borderSize = 1;
  
  
  ///image initialize
  firstFrame=1;
  im_firstFrame=1;
  
  // test anothe way to do it 
  
  c
  image_topic_=nh_.resolveName("images");

  
  // sub_=nh_.subscribe (cloud_topic_, 1,  &process_cloud::cloud_Callback , this);
  // sub_=nh_.subscribe (image_topic_, 1,  &process_cloud::image_Callback, this);
  
  ////////////////////////////////////////////////////////////////////////////////////////////
  /// ros subscribe messages
  //////////////////////////////////////////////////////////////////////////////////////////

  
    // cloud_topic_=nh_.resolveName("clouds");

  // cout<<"data " <<cloud_topic_<<" "<<image_topic_<<" "<<image_topic_2<<endl;
  
    //        subs_.push_back(nh_.subscribe (cloud_topic_, 1,  &process_cloud::cloud_Callback, this));
   //     
   //        
  //        subs_.push_back(nh_.subscribe (image_topic_, 1,  &process_cloud::image_Callback, this));
       
  
  
  /////////////////////////////////////////////////////////////////////////////////////////
  //   ///syncronize testing part
  //   //////////////////////////////////////////////////////////////////////////////////////
  
  //   
     
      image_sub.subscribe(nh_, "images" ,1);
      cloud_sub.subscribe(nh_, "clouds", 1);
//        subs_.push_back(cloud_sub.subscribe(nh_, cloud_topic_, 1));
//         subs_.push_back(image_sub.subscribe(nh_, image_topic_ ,1));
//    image_sub2.subscribe(nh_, image_topic_2,0.1);
//    /// exact time
//    typedef sync_policies::ExactTime<sensor_msgs::PointCloud2,sensor_msgs::Image> MySyncPolicy_exact;
//    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
//    
//    ///aproximated time
       typedef sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::PointCloud2> MySyncPolicy_aprox;
//    typedef sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy_aprox;
//    
     //TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(cloud_sub, image_sub, 10);
//   // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub, image_sub2, 25);
//   
      Synchronizer<MySyncPolicy_aprox> sync(MySyncPolicy_aprox(100000),  image_sub,cloud_sub);
//    Synchronizer<MySyncPolicy_aprox> sync(MySyncPolicy_aprox(1), image_sub, image_sub2);
//   //Synchronizer<MySyncPolicy_exact> sync(MySyncPolicy_exact(10), cloud_sub, image_sub);
//    
     sync.registerCallback(boost::bind(&process_cloud::Callback, this,_1,_2));
    // sub_.registerCallback((boost::bind(&process_cloud::Callback_imgs, this,_1,_2))
//   
  ROS_INFO("suscribed function to catch messages ... ");
   
}

void process_cloud::filter_cloud(PointCloud<PointXYZ>::ConstPtr cloud_,
				 const PointCloud<PointXYZ>::Ptr &  cloud_filtered)
{
  //first are removed sporius points
  PointCloud<PointXYZ>::Ptr cloud_stastic (new PointCloud<PointXYZ>);
  
  //segment in Z
  //now the cloud is filtered in heigh and
  PassThrough<PointXYZ> pass;
  pass.setInputCloud (cloud_);
  pass.setFilterFieldName ("z");// here we have to see where x and z
  pass.setFilterLimits (-6.0, 3.0);
  pass.filter (*cloud_filtered);
  PCL_INFO ("Removed x points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
  
  
  //now the cloud is filtered x
  
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (2, 5000);//maximum 7 meters;
  pass.filter (*cloud_filtered);
  PCL_INFO ("Removed x points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
  
  
  //filter in y
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-20, 20);
  pass.filter (*cloud_filtered);
  PCL_INFO ("Removed y points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
  
  StatisticalOutlierRemoval<PointXYZ> sor;
  sor.setInputCloud (cloud_filtered);
  sor.setMeanK (10);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);
  
  PCL_INFO ("Removed sporious points now the cloud contains %d points \n\n",cloud_filtered->points.size() );
  
}


void process_cloud::cloud_Callback (const sensor_msgs::PointCloud2ConstPtr & cloudI)
{
  

  
  PointCloud<PointXYZ>::Ptr cloud_ptr (new PointCloud<PointXYZ>);
  ROS_INFO("CLOUD timestamps %i.%i ", cloudI->header.stamp.sec,cloudI->header.stamp.nsec);
  
  if ((cloudI->width * cloudI->height) == 0)
    return;
  
  ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
	    
	    (int)cloudI->width * cloudI->height,
	    
	    cloudI->header.frame_id.c_str (),
	    
	    pcl::getFieldsList (*cloudI).c_str ());
  
  
  
  
  pcl::fromROSMsg(*cloudI, *cloud_ptr);
  
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  /// visualize cloud
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  visualization::PointCloudColorHandlerCustom <PointXYZ> point_cloud_color_handler (cloud_ptr, 150, 150, 150);   
  viewer->addPointCloud<PointXYZ> (cloud_ptr, point_cloud_color_handler, "original point cloud"); 
  
  range_image->createFromPointCloud (*cloud_ptr, pcl::deg2rad(0.6f),
				     pcl::deg2rad (360.f), pcl::deg2rad (180.0f),
				     sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  range_image_widget->showRangeImage(*range_image);
  viewer->spinOnce();
  pcl_sleep(1.0);
  
  viewer->removePointCloud("original point cloud");
  
  //////////////////////////////////////////////////////////
  /// now process the cloud while show in the pcl viewer
  
  image_utilities_obj.GenerarImagenOpenCV(range_image,im_range_current, im_range_current_norm);
  applyColorMap( im_range_current_norm, falseColorsMap, COLORMAP_HOT);
  ////////////////////////////////////////////////////////
  // apply optical flow
  ////////////////////////////////////////////////////////
  
  if(!firstFrame)
  {
    Mat im2 = Mat(im_range_previous_norm.rows, im_range_previous_norm.cols, im_range_previous_norm.type());
    cv::resize(im_range_current_norm, im2,im2.size(), INTER_LINEAR );
    cv::calcOpticalFlowFarneback(im2,im_range_previous_norm,flow,0.5, 3, 15, 3, 5, 1.2, 0);
    
    
    ///show data
    cv::cvtColor( im2 ,cflow, CV_GRAY2BGR);
    //drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));
    image_utilities_obj.drawOptFlowMap(flow, cflow, 10, 1.5, Scalar(0, 255, 0));
    image_utilities_obj.drawOpticalFlow_color(flow,color_flow, -1);
    
    imshow( "Optical flow",  cflow);  
    imshow( "Optical flow 2",  color_flow);  
    
    imshow("Range Image jet", falseColorsMap);
    
  }
  
  
  //create the images
  
  
  firstFrame=0;// we have read the first frame
  im_range_current_norm.copyTo(im_range_previous_norm);
  flow=Mat(im_range_current_norm.rows, im_range_current_norm.cols,CV_32FC2);
  cflow=Mat(im_range_current_norm.rows, im_range_current_norm.cols, CV_8UC3);
  
  
  //////////////////////////////////////////////////////////////////////////////////////
  ///gaussian mixtures
  //////////////////////////////////////////////////////////////////////////////////////
  
  falseColorsMap.copyTo(fore);
  //back=Mat(frame.rows, frame.cols, CV_8UC1);
  back=Mat(falseColorsMap.rows, falseColorsMap.cols, falseColorsMap.type());
  falseColorsMap.copyTo(back);
  bg.operator ()(falseColorsMap,fore);
  bg.getBackgroundImage(back);
  cv::erode(fore,fore,cv::Mat());
  /// cv::dilate(fore,fore,cv::Mat());
  // cv::findContours(fore,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
  //    cv::drawContours(frame,contours,-1,cv::Scalar(0,0,255),2);
  imshow("Frame",im_range_current_norm);
  imshow("Background",back);
  imshow("Fore",fore);
  waitKey(30);
}

///////////////////////////////////////////////////////////////////////////////
/// image calibration
///////////////////////////////////////////////////////////////////////////////

void process_cloud::image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  
  cv_bridge::CvImageConstPtr cv_ptr;
   ROS_INFO("IMAGE timestamps %i.%i ", msg->header.stamp.sec,msg->header.stamp.nsec);
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
  
  
  im_current=cv_ptr->image.clone();
  
  cvtColor(im_current, im_current_norm, CV_BGR2GRAY);
  imshow( "Image",  im_current_norm);  
  if(!im_firstFrame)
  {
    Mat im2 = Mat(im_previous_norm.rows, im_previous_norm.cols, im_previous_norm.type());
    cv::resize(im_current_norm, im2,im2.size(), INTER_LINEAR );
    cv::calcOpticalFlowFarneback(im2,im_previous_norm,im_flow,0.5, 3, 15, 3, 5, 1.2, 0);
    
    
    ///show data
    cv::cvtColor( im2 ,im_cflow, CV_GRAY2BGR);
    //drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));
    image_utilities_obj.drawOptFlowMap(im_flow, im_cflow, 10, 1.5, Scalar(0, 255, 0));
    image_utilities_obj.drawOpticalFlow_color(im_flow,im_color_flow, 1);
    
    imshow( "Optical flow image",  im_cflow);  
    imshow( "Optical flow image 2",  im_color_flow);  
    
    
    
  }
  
  //create the images
  
  im_firstFrame=0;// we have read the first frame
  im_current_norm.copyTo(im_previous_norm);
  im_flow=Mat(im_current_norm.rows, im_current_norm.cols,CV_32FC2);
  im_cflow=Mat(im_current_norm.rows, im_current_norm.cols, CV_8UC3);
  
  //////////////////////////////////////////////////////////////////////////////////////
  ///gaussian mixtures
  //////////////////////////////////////////////////////////////////////////////////////
  
  im_current.copyTo(im_fore);
  //back=Mat(frame.rows, frame.cols, CV_8UC1);
  im_back=Mat(im_current.rows, im_current.cols, im_current.type());
  im_current.copyTo(back);
  bg_im.operator ()(im_current,im_fore);
  bg_im.getBackgroundImage(im_back);
  cv::erode(im_fore,im_fore,cv::Mat());
  /// cv::dilate(fore,fore,cv::Mat());
  // cv::findContours(fore,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
  //    cv::drawContours(frame,contours,-1,cv::Scalar(0,0,255),2);
  imshow("Frame image ",im_current);
  imshow("Background image",im_back);
  imshow("Fore image",im_fore);
  waitKey(30);
  
  
}

void process_cloud::Callback(const sensor_msgs::ImageConstPtr& msg,
			     const sensor_msgs::PointCloud2ConstPtr& cloudI)
{
  
  
  /////////////////////////////////////////////////////////////////////
  /// point cloud 
  ///////////////////////////////////////////////////////////////////
  PointCloud<PointXYZ>::Ptr cloud_ptr (new PointCloud<PointXYZ>);
   ROS_INFO("Image timestamps%i.%i ", msg->header.stamp.sec,msg->header.stamp.nsec);
   ROS_INFO("Cloud timestamps%i.%i", cloudI->header.stamp.sec,cloudI->header.stamp.nsec);
  
  if ((cloudI->width * cloudI->height) == 0)
    return ;
  
  ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
	    
	    (int)cloudI->width * cloudI->height,
	    
	    cloudI->header.frame_id.c_str (),
	    
	    pcl::getFieldsList (*cloudI).c_str ());
  
  
  
  
  pcl::fromROSMsg(*cloudI, *cloud_ptr);
  
  ////////////////////////////////////////////////////////////////
  ///image
  ////////////////////////////////////////////////////////////////
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
    
    
    return ;
    
  }
  

  
}

 void process_cloud::Callback_imgs(const sensor_msgs::ImageConstPtr& image1,
		 const sensor_msgs::ImageConstPtr& image2)
 {
   
   cv_bridge::CvImageConstPtr cv_ptr1,cv_ptr2;
   ROS_INFO("enter here timestamps%i. ", image1->header.stamp.sec,image1->header.stamp.nsec);
  
  try
  
  {
    
    if (enc::isColor(image1->encoding) && enc::isColor(image2->encoding)){
      
      cv_ptr1 = cv_bridge::toCvShare(image1, enc::BGR8);
      cv_ptr2 = cv_bridge::toCvShare(image2, enc::BGR8);
    } 
    else
    {  
      cv_ptr1 = cv_bridge::toCvShare(image1, enc::MONO8);
       cv_ptr2 = cv_bridge::toCvShare(image2, enc::MONO8);
    
    ROS_INFO("Image1 data size (%d, %d)",cv_ptr1->image.rows, cv_ptr1->image.cols);
    ROS_INFO("Image2 data size (%d, %d)",cv_ptr1->image.rows, cv_ptr2->image.cols);
    } 
  }
  
  catch (cv_bridge::Exception& e)
  
  {
    
    ROS_ERROR("cv_bridge exception: %s", e.what());
    
    
    return ;
    
  }
 }
