
#include "cv.h"
namespace cv
{

class ChessBoardGenerator
{
public:
    double sensorWidth; 
    double sensorHeight;     
    size_t squareEdgePointsNum;
    double min_cos;
    mutable double cov;
    Size patternSize;
    int rendererResolutionMultiplier;

    ChessBoardGenerator(const Size& patternSize = Size(8, 6));
    Mat operator()(const Mat& bg, const Mat& camMat, const Mat& distCoeffs, vector<Point2f>& corners) const;    
    Size cornersSize() const;
private:
    void generateEdge(const Point3f& p1, const Point3f& p2, vector<Point3f>& out) const;
    Mat generageChessBoard(const Mat& bg, const Mat& camMat, const Mat& distCoeffs, 
        const Point3f& zero, const Point3f& pb1, const Point3f& pb2, 
        float sqWidth, float sqHeight, const vector<Point3f>& whole, vector<Point2f>& corners) const;
    void generateBasis(Point3f& pb1, Point3f& pb2) const;  
    Point3f generateChessBoardCenter(const Mat& camMat, const Size& imgSize) const;
    Mat rvec, tvec;
};
};