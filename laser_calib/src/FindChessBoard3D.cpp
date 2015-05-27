#include "FindChessBoard3D.h"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>


FindChessBoard3D::FindChessBoard3D(){
}

FindChessBoard3D::~FindChessBoard3D(){
}

void FindChessBoard3D::SegmentData(PointCloud<PointXYZ>::ConstPtr cloud_,
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
   pass.setFilterLimits (2, 7.0);//iri
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


int FindChessBoard3D::SegmentData_by_planes(PointCloud<PointXYZ>::ConstPtr cloud_filtered,
			const PointCloud<PointXYZ>::Ptr &  cloud_Plane,
			const PointCloud<PointXYZ>::Ptr & cloud_nonPlanes,
			const PointCloud<PointXYZ>::Ptr & cloud_projected){
    //////////////////////////////////////////////////////////////////////////////////////////////
   //segmentacion of planes 
   //////////////////////////////////////////////////////////////////////////////////////////////
   
   PointIndices::Ptr inliers (new PointIndices);
   ModelCoefficients::Ptr coefficients (new ModelCoefficients);
   
   
   //Create the segmentation object
   
   SACSegmentation<pcl::PointXYZ> seg;
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
     PCL_ERROR ("Could not estimate a planar model for the given dataset.\n\n");
     return (-1);
   }
   
   
   PCL_INFO ("Model inliers with %d points \n\n",inliers->indices.size());
   
   /////////////////////////////////////////////////////////////
   //find planes and non planes
   
   
   
   ExtractIndices<pcl::PointXYZ> extract;
   extract.setInputCloud (cloud_filtered);
   extract.setIndices (inliers);
   extract.setNegative (false);
   
   extract.filter (*cloud_Plane); //*plane found
   
   
   PCL_INFO ("PointCloud representing the planar component: %d data points\n\n",cloud_Plane->points.size ());
   
   // Write the planar inliers to disk
   //extract no planes
   extract.setNegative (true);
   extract.filter (*cloud_nonPlanes); //*
   
   PCL_INFO ("PointCloud representing the non planar component: %d data points\n\n",cloud_nonPlanes->points.size ());
   
   
   //project points to the plane
   ProjectInliers<PointXYZ> proj;
   proj.setModelType (SACMODEL_PLANE);
   proj.setInputCloud (cloud_Plane);
   proj.setModelCoefficients (coefficients);
   proj.filter (*cloud_projected);    
   
   return(1); //planes found
  
}

 void FindChessBoard3D::Clustering(PointCloud<PointXYZ>::ConstPtr cloud_nonPlanes,
                const PointCloud<PointXYZRGB>::Ptr& cloudRGB_cluster,
		const PointCloud<PointXYZ>::Ptr& cloud_chessboard){
   
   
   
   
   search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
   tree->setInputCloud (cloud_nonPlanes);
   
   //***************************************************************************************
   //***************************************************************************************
   // this part is important to segment the number of planes
   //***************************************************************************************
   //***************************************************************************************
   vector<PointIndices> cluster_indices;
   EuclideanClusterExtraction<PointXYZ> ec;
   //ec.setClusterTolerance (0.02); // 2cm
   // 
   //////////////////////////////////////////////////////77
   //works indoors
   //////////////////////////////////////////////////////77
   ec.setClusterTolerance (0.02);
   ec.setMinClusterSize (1);
   ec.setMaxClusterSize (600);
    //////////////////////////////////////////////////////77
   //works outdoors
   //////////////////////////////////////////////////////77
   //ec.setClusterTolerance (0.2); 
   // ec.setMinClusterSize (1);
   //ec.setMaxClusterSize (1000);
   
   ec.setSearchMethod (tree);
   ec.setInputCloud( cloud_nonPlanes);
   ec.extract (cluster_indices);
   
   //pcl::PointCloud<PointXYZRGB>::Ptr cloudRGB_cluster (new PointCloud<PointXYZRGB>);
   //pcl::PointCloud<PointXYZ>::Ptr cloud_chessboard (new PointCloud<PointXYZ>);
   
   //random numbers
   srand ( time(NULL) );
   
   int j=1;
   for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
     
     PointCloud<PointXYZ>::Ptr cloud_cluster (new PointCloud<PointXYZ>);
     PointCloud<PointXYZRGB> PointAux;
     PointCloud<PointXYZ> PointAux_XYZ;
     
     PointAux.points.resize(1);
     PointAux_XYZ.points.resize(1);
     
     int r_rand = rand() % 255 + 1;
     int g_rand = rand() % 255 + 1;
     int b_rand = rand() % 255 + 1; 
     //rand color
     uint8_t r = r_rand;
     uint8_t g = g_rand;
     uint8_t b = b_rand;
     
     for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
       
       cloud_cluster->points.push_back (cloud_nonPlanes->points[*pit]); //*
       PointAux.points[0].x=cloud_nonPlanes->points[*pit].x;
       PointAux.points[0].y=cloud_nonPlanes->points[*pit].y;
       PointAux.points[0].z=cloud_nonPlanes->points[*pit].z;
       
       PointAux_XYZ.points[0].x=cloud_nonPlanes->points[*pit].x;
       PointAux_XYZ.points[0].y=cloud_nonPlanes->points[*pit].y;
       PointAux_XYZ.points[0].z=cloud_nonPlanes->points[*pit].z;
       //color 
       int32_t rgb = (r << 16) | (g << 8) | b;
       
       
       PointAux.points[0].rgb=rgb;     
       
       cloudRGB_cluster->points.push_back (PointAux.points[0]);
       cloud_chessboard->points.push_back (PointAux_XYZ.points[0]);
       
     }
     
     
     cout<< "Data points of cluster " << cloud_cluster->points.size () << " data points of cluster No." <<j<< endl;
     cout << "Data points of planar cluster " << cloud_cluster->points.size () << " data points of cluster No." <<j<< endl;
     
     j++;
   }
 }
 void FindChessBoard3D::chessboard_segment(PointCloud<PointXYZ>::ConstPtr cloud_chessboard,
		const PointCloud<PointXYZ>::Ptr& cloud_plane_cluster ,
		const PointCloud<PointXYZ>::Ptr& cloud_nonplane_cluster,
		const PointCloud<PointXYZ>::Ptr& cloud_plane_projected,
		const PointCloud<PointXYZ>::Ptr&   cloud_chessboard_square,
		const PointCloud<PointXYZ>::Ptr&   cloud_hull)
{
  //now find planes over the set of clusters
   //aqui me quede ahora generar un vector de point clouds proyectar en un plano y crear un convex hull
    PCL_INFO ("Points after %d...", cloud_chessboard->points.size());
    PCL_INFO ("*****************************************************************************\n");  
    PCL_INFO ("Here we segment the chessboard\n");  
    PCL_INFO ("*****************************************************************************\n");  
 
    
    
    if (cloud_chessboard->points.size()>0){
      //Create the segmentation object
      
      pcl::PointIndices::Ptr inliers (new PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new ModelCoefficients);
      
      
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      
      //segmentation of plane
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (1000);
      seg.setDistanceThreshold (0.05);
      
      
      seg.setInputCloud (cloud_chessboard);
      seg.segment (*inliers, *coefficients);
      
      PCL_INFO ("Points in the cloud %d points ",cloud_chessboard->points.size());  
      PCL_INFO ("Model of the cluster inliers with %d points ",inliers->indices.size());
      
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      //filters to segment plane and non plane
      extract.setInputCloud (cloud_chessboard);
      extract.setIndices (inliers);
      extract.setNegative (false);
      
      
      //extract planes
      extract.filter (*cloud_plane_cluster); //*plane found
      
      //extract non planes
      extract.setNegative (true);
      extract.filter (*cloud_nonplane_cluster); 
      
      
      //filter to extract the complete pattern
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud_plane_cluster);
      sor.setMeanK (30);
      sor.setStddevMulThresh (1);
      sor.filter (*cloud_plane_cluster);
      
      //project to 3d
      pcl::ProjectInliers<PointXYZ> proj;
      proj.setModelType (SACMODEL_PLANE);
      proj.setInputCloud (cloud_plane_cluster);
      proj.setModelCoefficients (coefficients);
      proj.filter (*cloud_plane_projected);
      
      //convex hull
      
      ConvexHull<PointXYZ> chull;
      chull.setInputCloud (cloud_plane_projected);
      chull.reconstruct (*cloud_hull);
      
      //now create a minimum bounding box
      
      Eigen::Vector3f plane_normal;
      plane_normal.x() = coefficients->values[0];
      plane_normal.y() = coefficients->values[1];
      plane_normal.z() = coefficients->values[2];
      // compute an orthogonal normal to the plane normal
      Eigen::Vector3f v = plane_normal.unitOrthogonal();
      // take the cross product of the two normals to get
      // a thirds normal, on the plane
      Eigen::Vector3f u = plane_normal.cross(v); 
      
      CvMat* points_mat = cvCreateMat(cloud_hull->points.size(), 2, CV_32F );
      
      
      vector<cv::Point2f> points; 
      Eigen::Vector3f p0(cloud_hull->points[0].x,
			 cloud_hull->points[0].y,
			 cloud_hull->points[0].z);; 
			 for(int ii=0; ii<cloud_hull->points.size(); ii++){
			   
			   Eigen::Vector3f p3d(cloud_hull->points[ii].x,
					       cloud_hull->points[ii].y,
			  cloud_hull->points[ii].z);
			   
			   // subtract all 3D points with a point in the plane
			   // this will move the origin of the 3D coordinate system
			   // onto the plane
			   p3d = p3d - p0;
			   
			   cv::Point2f p2d;
			   p2d.x = p3d.dot(u);
			   p2d.y = p3d.dot(v);
			   points.push_back(p2d); 
			 } 
			 
			 
			 //CvPoint2D32f _box_points[4];
			 
			 //Eigen::Vector4f  centroid;
			 //compute3DCentroid (*cloud_plane_projected, centroid);
			 
			 //cout << "y que pedo con el centroid   " <<centroid<<endl;
			 
			 //cerr<<box<<endl;
			 
			 cv::Mat points_mat2(points); 
			 
			 // PointCloud<PointXYZ>::Ptr cloud_chessboard_square(new PointCloud<PointXYZ>);
			 
			 
			 RotatedRect rrect = minAreaRect(points_mat2); 
			 
			 cv::Point2f rrPts[4];
			 rrect.points(rrPts);
			 
			 //store the table top bounding points in a vector
			 for(unsigned int ii=0; ii<4; ii++)
			 {
			   
			   Eigen::Vector3f pbbx(rrPts[ii].x*u + rrPts[ii].y*v + p0); 
			   
			   PointCloud<PointXYZ> PointAux;
			   
			   PointAux.points.resize(1);
			   PointAux.points[0].x=pbbx[0];
			   PointAux.points[0].y=pbbx[1];
			   PointAux.points[0].z=pbbx[2];
			   
			   cloud_chessboard_square->points.push_back (PointAux.points[0]);
			   
			   //    Eigen3::Vector3f pbbx(rrPts[ii].x*u + rrPts[ii].y*v + p0);
			   //   table_top_bbx.push_back(pbbx);
			 } 
    }
}
  void FindChessBoard3D::FindPointsChessboard3D(PointCloud<PointXYZ>::ConstPtr  cloud,
	      const PointCloud<PointXYZ>::Ptr&  cloud_filtered,
	      const PointCloud<PointXYZ>::Ptr&  cloud_Planes,
	      const PointCloud<PointXYZ>::Ptr&  cloud_nonPlanes,
	      const PointCloud<PointXYZ>::Ptr& cloud_nonplane_cluster,
	      const PointCloud<PointXYZ>::Ptr& cloud_projected,
	      const PointCloud<PointXYZRGB>::Ptr& cloudRGB_cluster,
	      const PointCloud<PointXYZ>::Ptr&  cloud_chessboard,
	      const PointCloud<PointXYZ>::Ptr&  cloud_plane_cluster,
	      const PointCloud<PointXYZ>::Ptr&  cloud_plane_projected,
	      const PointCloud<PointXYZ>::Ptr&  cloud_hull,
	      const PointCloud<PointXYZ>::Ptr&  cloud_chessboard_square){
    //filter data to only find the chessboard
  SegmentData( cloud,cloud_filtered);
  PCL_INFO ("Removed sporious points now the cloud contains %d points\n\n ",cloud_filtered->points.size() );
  
  //now we segment by planes and no planes
   SegmentData_by_planes( cloud_filtered,cloud_Planes,cloud_nonPlanes,cloud_projected);
   
  
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  
  

  
  //cluters data for selecting the board that does not belong to the principal planar surface
  
 Clustering(cloud_nonPlanes,cloudRGB_cluster,cloud_chessboard);
  
 
 //  now find the data in planes
  chessboard_segment( cloud_chessboard,
             cloud_plane_cluster ,
             cloud_nonplane_cluster,
             cloud_plane_projected,
	     cloud_chessboard_square,
	     cloud_hull);
 }
 
 void FindChessBoard3D::DrawClouds( visualization::PCLVisualizer viewer,
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
	      PointCloud<PointXYZ>::ConstPtr  cloud_chessboard_square){
   
   viewer.setBackgroundColor (0, 0, 0); 
   viewer.addCoordinateSystem (1.0);
   
   //normal cloud
   viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud",0);
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0,"cloud");
   
   
   //nfilter3d cloud
   viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud_filtered",1);
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0,"cloud_filtered");
   
   //planes
   viewer.addPointCloud<pcl::PointXYZ>(cloud_Plane, "cloud_Plane",0);
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0,"cloud_Plane");
   
   //non planes
   
   viewer.addPointCloud<pcl::PointXYZ>(cloud_nonPlanes, "cloud_nonPlane",0);
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.9, 1.0,"cloud_nonPlane");
   
   //plane projected
   
   viewer.addPointCloud<pcl::PointXYZ>(cloud_projected, "cloud_projected",0);
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,"cloud_projected");
   
   
   viewer.addPointCloud<pcl::PointXYZRGB>(cloud_RGB, "cloud_RGB",0);
   
   
   viewer.addPointCloud<pcl::PointXYZ>(cloud_chessboard, "cloud_chessboard",0);
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.0,"cloud_chessboard");
   
   
   viewer.addPointCloud<pcl::PointXYZ>(cloud_plane_cluster, "cloud_plane_cluster"); 
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2, "cloud_plane_cluster");
   
   
   viewer.addPointCloud<pcl::PointXYZ>(cloud_plane_projected, "cloud_plane_projected"); 
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "cloud_plane_projected");
   
   viewer.addPointCloud<pcl::PointXYZ>(cloud_hull, "convex_hull"); 
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "convex_hull");
   
   viewer.addPolygon<pcl::PointXYZ>(cloud_hull,1,1,1,"polygon",0);
   
   //visualizer.addPlane(*coefficients, "cloud_plane");  
   
   viewer.addPointCloud<pcl::PointXYZ>(cloud_chessboard_square, "chessboard"); 
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10, "chessboard");
   
   viewer.addPolygon<pcl::PointXYZ>(cloud_chessboard_square,0,1,0,"square",0);
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH ,105, "square");
   
   std::stringstream out;

  
/*  
  while (!viewer.wasStopped ()){
    viewer.spinOnce (100000);
    
    
  }*/
 }
 
 void FindChessBoard3D::AssociatePoints(Mat &image,PointCloud<PointXYZ>::ConstPtr  cloud, 
		      MatrixXf &x3d){
   
   RangeImage rangeImage;
   float angular_resolution_x = 0.5f,
   angular_resolution_y = angular_resolution_x;
   
   float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
   float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
   float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
   Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
   pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME ;
   float noiseLevel=0.00;
   float minRange = 0.0f;
   int borderSize = 1;
   Eigen::Matrix4f trans;
   //PointCloud<PointXYZ>::Ptr  cloud_out(new pcl::PointCloud<pcl::PointXYZ> ());//principal cloud
   
   trans<< 1,0,0,0,
   0,1,0,0,
   0,0,1,0,
   0,0,0,1;
   
   rangeImage.createFromPointCloud(*cloud, pcl::deg2rad(0.1f),
				   pcl::deg2rad (360.f), pcl::deg2rad (180.0f),
				   sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
   
   float image_x;
   float image_y;
   float range;
 
      
   MatrixXf x2d=MatrixXf::Ones(cloud->points.size(),3);

    for (int ii=0;ii<cloud->points.size();ii++){
      rangeImage.getImagePoint( cloud->points[ii].x,
				cloud->points[ii].y,
				cloud->points[ii].z,  
				image_x,
				image_y,
				range);
      

        
        
       
      x2d(ii,0)=image_x;
      x2d(ii,1)=image_y;
      
      
    }
     
     
   //  cout<<"sort points"<<endl;
     SortPoints3d(x2d,cloud, x3d);
     //apply convexhull and sort
   //  convexHull( Mat(pointBuf), hull, true);
     
     
     //visualize data y create image sintetic
     Mat image2(600,600, CV_64FC3,Scalar(0,0,0));
     
     //now show data
    // int hullcount = (int)hull.size();
     //Point2f pt0 = hull[hullcount-1];
     int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
     double fontScale = 2;
     int thickness = 3; 
   
     for(int  i = 0; i < x3d.rows(); i++ )
     {
       
       
       circle(image2, Point(x2d(i,0),x2d(i,1)), 3, Scalar(0, 0, 255), CV_FILLED, CV_AA);
       stringstream ss2;//create a stringstream
       ss2 << i;//add number to the stream
       
       putText(image2, ss2.str(), Point(x2d(i,0),x2d(i,1)), fontFace, 1,
	       Scalar(0, 255, 0), thickness, 8);
       
     
     }

     
     image=image2.clone();
    //  imshow( "Image2", image );  
   //waitKey(0);
//          pcl::visualization::PCLVisualizer viewer5("3D Viewer");
//      
//   // only if you want to visualize   
//   visualization::RangeImageVisualizer range_image_widget ("Range image");
//   range_image_widget.showRangeImage (rangeImage);
//   
//    
//    while (!viewer5.wasStopped ()){
//     range_image_widget.spinOnce ();
//      viewer5.spinOnce ();
//      pcl_sleep (0.01);
//    }
   
   
 }
 
 int Sort4PointsClockwise(PointCloud<PointXYZ>::ConstPtr  cloud){
   
    // check if you have 4 points
    
 
}

  
  
 void FindChessBoard3D::SortPoints3d(MatrixXf &x2d, PointCloud<PointXYZ>::ConstPtr  cloud, MatrixXf & x3d){
   
   
   Vector3f point=x2d.colwise().sum();
     
     point=point/x2d.rows();
     
     cout<<"point "<<point<<endl;
     
     vector<float> angles;
     int i,j;
     
          
     x3d=MatrixXf::Ones(x2d.rows(),4);

	   
     // compute angles
     for (i=0; i<x2d.rows();i++){
       
       double x=(point(0)-x2d(i,0));
       double y=(point(1)-x2d(i,1));
       
       angles.push_back(atan2(y,x)*180 / (3.1416));//degree in radians
     //  cout<<angles[i]<<endl;
     }
     
      for (i =0; i<x2d.rows();i++){
	x3d(i,0) = cloud->points[i].x;
	x3d(i,1) = cloud->points[i].y;
        x3d(i,2) = cloud->points[i].z;
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
       
     
     //////////////////////////////////////////////////////
     // now order by angle chossing the minor to mayor
      
       for (i =0; i<x2d.rows();i++){
          for(j = i+1; j < x2d.rows(); j ++) {
                if(angles[j] < angles[i]) {
		  
                     float temp_x = cloud->points[i].x;
	             float temp_y = cloud->points[i].y;
		     float temp_z = cloud->points[i].z;
		     
                      x3d(i,0) = cloud->points[j].x;
	              x3d(i,1) = cloud->points[j].y;
		      x3d(i,2) = cloud->points[j].z;
		      
	       
	             x3d(i,0) = temp_x;
	              x3d(i,1) = temp_y;
		      x3d(i,2) = temp_z;
		      
               
               
	           }
    
           }
       }
   // saving 
 }
