#include "FindChessBoardImage.h"


FindChessBoardImage::FindChessBoardImage(){
  
  thresh =50; 
   N = 11;
}

FindChessBoardImage::FindChessBoardImage(int t, int n){
  
  thresh =t; 
   N = n;
}
FindChessBoardImage::~FindChessBoardImage(){
}


bool FindChessBoardImage::FindBoardImage(Mat &image,  Size boardSize, MatrixXf &x2d)
{
  bool found;
  vector<Point2f> pointBuf;
  
  vector<Point2f> hull;
  
  cout<<"finding chessboard.......... "<<endl;
  cout<<"board size "<<boardSize.height<<endl;
  cout<<"board size "<<boardSize.width<<endl;
  found = findChessboardCorners( image, boardSize, pointBuf,
				 CV_CALIB_CB_ADAPTIVE_THRESH |  CV_CALIB_CB_NORMALIZE_IMAGE |CV_CALIB_CB_FILTER_QUADS);
  
  if (found){//draw corners
    
        
	drawChessboardCorners( image, boardSize, Mat(pointBuf), found );
	
	convexHull( Mat(pointBuf), hull, true);
	//we select only 4 points or the square
	RotatedRect rrect = minAreaRect(hull); 
	
	Point2f rrPts[4];
	rrect.points(rrPts);
	int count=(int)pointBuf.size();
	
	vector<Point2f> pointBuf2;
	
	vector<Point2f> hull2;
		
	for(int  i = 0; i < 4; i++ )
	  pointBuf2.push_back (rrPts[i]);
		
	// apply convexhull in the points to order again

	convexHull( Mat(pointBuf2), hull2, true);
	
	//now show data
	int hullcount = (int)hull2.size();
	
	x2d=MatrixXf::Ones(4,3);
	
	for(int  i = 0; i < hullcount; i++ )
	{
	  x2d(i,0)=rrPts[i].x;
	  x2d(i,1)=rrPts[i].y;
	}
	
	
	// now order points
	
	SortPointsAngle(x2d);
	int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontScale = 2;
	int thickness = 3;
	 for(int  i = 0; i < x2d.rows(); i++ ){
	  //Point2f pt = hull2[i];
	  //line(image, pt0, pt, Scalar(0, 255, 0), 1, CV_AA);
	  circle(image, Point(x2d(i,0),x2d(i,1)), 3, Scalar(0, 0, 255), CV_FILLED, CV_AA);
	   stringstream ss2;//create a stringstream
           ss2 << i;//add number to the stream
   
	   putText(image, ss2.str(), Point(x2d(i,0),x2d(i,1)), fontFace, 1,
           Scalar(0, 0, 255), thickness, 8);
	
	}
	
	
	/* int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontScale = 2;
	int thickness = 3; 
	
	 hullcount = (int)hull2.size();
	 Point2f pt0 = hull2[hullcount-1];
	 
	 for(int  i = 0; i < hullcount; i++ )
	{
	  Point2f pt = hull2[i];
	  line(image, pt0, pt, Scalar(0, 255, 0), 1, CV_AA);
	  circle(image, pt0, 3, Scalar(0, 0, 255), CV_FILLED, CV_AA);
	   stringstream ss2;//create a stringstream
           ss2 << i;//add number to the stream
   
	   putText(image, ss2.str(), pt0, fontFace, 1,
           Scalar(0, 0, 255), thickness, 8);
	   pt0 = pt;
	}*/
	
	
	return(true);// pattern found;
  }else
    
    return (false);// pattern not found
}


void  FindChessBoardImage::GenerateSyntethicImage(Mat &image){
  
  
       const Size imgSize(1280, 960);
       const Size brdSize(8, 7);
       const size_t brds_num = 1;
       
       cout << "Initializing chess board generator...";   
       Mat background(imgSize, CV_8UC3); 
    //  randu(background, Scalar::all(0), Scalar::all(255));    
     //  GaussianBlur(background, background, Size(5, 5), 2);
    
       ChessBoardGenerator cbg(brdSize);
       cbg.rendererResolutionMultiplier = 1;
       cout << "Done" << endl;
       
       /* camera params */
       Mat_<double> camMat(3, 3);
     
       //camMat << 300., 0., background.cols/2., 0, 300., background.rows/2., 0., 0., 1.;
      camMat<<  1616.77 ,0 ,615.92,
         0 ,1613.90 ,428.87,
        0 , 0  ,1.00;
    /*  K<<  1616.77 ,0 ,615.92,
         0 ,1613.90 ,428.87,
        0 , 0  ,1.00;*/
       
       
       Mat_<double> distCoeffs(1, 5);
       distCoeffs << 0, 0.0, 0., 0., 0.;
     
       
       cout << "Generating chessboards...";    
//        vector<Mat> boards(brds_num);
        vector<Point2f> tmp;
//         cout << "Done" << endl;
//        
//        for(size_t i = 0; i < brds_num; ++i)
// 	 cout << (boards = cbg(background, camMat, distCoeffs, tmp), i) << " ";
//        cout << "Done" << endl;    
       
       image=cbg(background, camMat, distCoeffs, tmp);
}


void FindChessBoardImage::findSquares( const Mat& image, vector<vector<Point> >& squares , vector<vector<Point> >& bsquares /* biggest  rectangle*/)
{
    squares.clear();
    
    Mat pyr, timg, gray0(image.size(), CV_8U), gray;
    
    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;
    vector<Point> approx_biggest;
    // find squares in every color plane of the image
    
     int n_rect;
	    
	    n_rect=0;
	    
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);
        
        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            vector<Point> approx;
	     
            
            // test each contour
	   
	    
            for( size_t i = 0; i < contours.size(); i++ )
	      
            {
	    
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
                
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    isContourConvex(Mat(approx)) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
		    
		        if ( n_rect<=contourArea(Mat(approx))){
			  
			   
			  n_rect=(int)contourArea(Mat(approx));
			  approx_biggest=approx;
			 // std::cout<<"n   \n"<<n_rect<<std::endl;
			}
                }
            }
            
        }
    }
    
    bsquares.push_back(approx_biggest);
}


double FindChessBoardImage::angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void FindChessBoardImage::drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 1, CV_AA);
	
    }

    //imshow(wndname, image);
}

void FindChessBoardImage::SortPointsAngle(MatrixXf &x2d){
  
    // the array is formed by [x1 y1 1,
//                             x2 y2  ]
  
     // compute center point
     Vector3f point=x2d.colwise().sum();
     
     point=point/x2d.rows();
     
     cout<<"point "<<point<<endl;
     
     vector<float> angles;
     int i,j;
     // compute angles
     for (i=0; i<x2d.rows();i++){
       
       double x=(point(0)-x2d(i,0));
       double y=(point(1)-x2d(i,1));
       
       angles.push_back(atan2(y,x)*180 / (3.1416));//degree in radians
       cout<<angles[i]<<endl;
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

bool FindChessBoardImage::FindBoardImageRectangle(Mat &image,  Size boardSize, MatrixXf &x2d){
  
  
  vector<vector<Point> > squares,biggest_square;
  findSquares(image, squares,biggest_square);
  //objfindimageboard.drawSquares(image2, squares);
  x2d=MatrixXf::Ones(4,3);
  
  for(int kk=0;kk<biggest_square.size();kk++){
    const Point* p = &biggest_square[kk][0];
    int n = (int)biggest_square[kk].size();
    for (int ii=0;ii<n;ii++){
      // circle(image2, p[ii], 3, Scalar(0, 0, 255), CV_FILLED, CV_AA);
      //x2d=MatrixXf::Ones(4,3);
      x2d(ii,0)=p[ii].x;
      x2d(ii,1)=p[ii].y;
    }
    
  }
  
  //sort data
  
  SortPointsAngle(x2d);
  int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
  double fontScale = 2;
  int thickness = 3;
  for(int  i = 0; i < x2d.rows(); i++ ){
    //Point2f pt = hull2[i];
    //line(image, pt0, pt, Scalar(0, 255, 0), 1, CV_AA);
    circle(image, Point(x2d(i,0),x2d(i,1)), 3, Scalar(0, 0, 255), CV_FILLED, CV_AA);
    stringstream ss2;//create a stringstream
    ss2 << i;//add number to the stream
    
    putText(image, ss2.str(), Point(x2d(i,0),x2d(i,1)), fontFace, 1,
	    Scalar(0, 0, 255), thickness, 8);
    
  }
  
  return (1);
	
	 
}

bool FindChessBoardImage::Find_image_chessboard(Mat &image,  Size boardSize, MatrixXf &x2d){

     MatrixXf x2d1,x2d2;
     
     //find chessboard opencv
     bool found_chessboard= FindBoardImage(image,  boardSize, x2d1);

     //find rectangle
    bool found_chessboard_2=FindBoardImageRectangle(image,  boardSize, x2d2);

     x2d=x2d2;
    
    if(found_chessboard){//first check fi the board is found
    if (belongs_points(x2d1, x2d2(2,0), x2d2(2,1),x2d2(0,0),x2d2(0,1))) //the points of the boards, belongs.
      return true;    
    else
      return false;
    }
}

bool FindChessBoardImage::is_inside (float px, float py, float prec_upper_left_x, float prec_upper_left_y,
float prec_lower_right_x, float prec_lower_right_y){

if ((px >= prec_upper_left_x ) && (px<= prec_lower_right_x) 
  && (py >= prec_upper_left_y ) && (py <= prec_lower_right_x)) 
return true;
else
return false;
 
}

// check if all the points belong to square.

bool FindChessBoardImage::belongs_points(vector<Point2f> points,
					 Point2f p_upper_left, 
					 Point2f p_lower_right){
 
 int count=(int)points.size();
	
   for( int i = 0; i < count; i++ )
	if(!is_inside (points[i].x, points[i].y,  p_upper_left.x, p_upper_left.y,
	p_lower_right.x, p_lower_right.y))
	return false;
	
  return true;
}	


bool FindChessBoardImage::belongs_points(MatrixXf points, float prec_upper_left_x, float prec_upper_left_y,
float prec_lower_right_x, float prec_lower_right_y){
 
  int count=points.rows();
  
  for( int i = 0; i < count; i++ ){
    if(!is_inside (points(i,0), points(i,1),  prec_upper_left_x, prec_upper_left_y,
	prec_lower_right_x, prec_lower_right_y))
      
	return false;
   }
	
   return true;
}	
