#include <Eigen/Eigen>
#include <Eigen/SVD> 



#include "ChessBoardGenerator.h"

using namespace Eigen;
using namespace cv;
using namespace std;

class FindChessBoardImage{


public:
  int thresh; 
  int N ;
  
  FindChessBoardImage();
  ~FindChessBoardImage();
  FindChessBoardImage(int t, int n);
  

  bool FindBoardImage(Mat &image,  Size boardSize, MatrixXf &x2d);// find patter with calibration method
  bool FindBoardImageRectangle(Mat &image,  Size boardSize, MatrixXf &x2d);// find patter with calibration method
  
  void GenerateSyntethicImage(Mat &image);// image syntethic
  
  void findSquares( const Mat& image, vector<vector<Point> >& squares , vector<vector<Point> >& bsquares /* biggest  rectangle*/);
  double angle( Point pt1, Point pt2, Point pt0 );
  void drawSquares( Mat& image, const vector<vector<Point> >& squares );
  void SortPointsAngle(MatrixXf &x2d);
  
   bool is_inside (float px, float py, float prec_upper_left_x, float prec_upper_left_y,
   float prec_lower_right_x, float prec_lower_right_y);

    bool belongs_points(vector<Point2f> points, Point2f p_upper_left, Point2f p_lower_right);

    bool belongs_points(MatrixXf points, float prec_upper_left_x, float prec_upper_left_y,
     float prec_lower_right_x, float prec_lower_right_y);

    
   bool Find_image_chessboard(Mat &image,  Size boardSize, MatrixXf &x2d);
 
};