/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>
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

 

#include <pcl/console/parse.h>
 #include <rosbag/bag.h>

 #include <rosbag/view.h>

 #include "ros/ros.h"

 #include <ros/names.h>
 using namespace pcl;

 using namespace std;

 using namespace Eigen;



 using namespace cv;
 
void Callback(const sensor_msgs::PointCloud2ConstPtr& cloudI )

 {

   

   

   /////////////////////////////////////////////////////////////////////

   /// point cloud 

   ///////////////////////////////////////////////////////////////////

  PointCloud<PointXYZ>::Ptr cloud_ptr (new PointCloud<PointXYZ>);
  ROS_INFO("CLOUD timestamps %i.%i ", cloudI->header.stamp.sec,cloudI->header.stamp.nsec);
  

  if ((cloudI->width * cloudI->height) == 0)
    return;

  

  ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
	    (int)cloudI->width * cloudI->height,
	    cloudI->header.frame_id.c_str (),
	    pcl::getFieldsList (*cloudI).c_str ());
 
  pcl::fromROSMsg(*cloudI, *cloud_ptr);
   
  
   //////////////////////////////////////////////
   ///here change the path
   //////////////////////////////////////////////
   pcl::io::savePCDFileASCII ("/home/aortega/Experiments/calibration_iri_velodyne/exp6/test_pcd.pcd", *cloud_ptr);
   
 }
int main(int argc, char **argv)
{
   ros::init(argc, argv, "vision_node");
   
   ros::NodeHandle nh_; 
   std::string cloud_topic_;
   ros::Subscriber sub;
   
    cloud_topic_=nh_.resolveName("clouds");
   
    cout<<cloud_topic_<<endl;
   sub=nh_.subscribe (cloud_topic_, 1,  &Callback,ros::TransportHints().unreliable() );
   
     ros::spin();

   return 0;
}
