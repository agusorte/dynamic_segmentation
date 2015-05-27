#include "/usr/include/eigen3/Eigen/Eigen"
//#include "/usr/include/eigen3/Eigen/LU"
 #include <string>
#include <iostream>
#include <vector>
#include "/usr/include/eigen3/Eigen/SVD"
 
using namespace Eigen;

class Lu {

public:

Lu ();
~Lu ();

void Initialize(MatrixXf x3d_h,MatrixXf x2d_h,  Matrix3f A);

void compute();

int objpose (MatrixXf P, MatrixXf Qp);

void data_show ();//only debbuging

//Transform and project
MatrixXf xformproj(MatrixXf P, Matrix3f R, Vector3f t);
MatrixXf xform(MatrixXf P, Matrix3f  R, Vector3f t);
MatrixXf rpyMat (Vector3f angs);
MatrixXf randrotmat(int n);
MatrixXf quat2mat(Vector4f q);
MatrixXf qmatW(Vector4f  q);
MatrixXf qmatQ(Vector4f  q);

void abskernel(MatrixXf P, MatrixXf Q, std::vector<Matrix3f > F, MatrixXf G);
Vector3f estimate_t( MatrixXf R,MatrixXf G,std::vector<Matrix3f > F,MatrixXf P,int n );
int sgn(double d);
Matrix3f GetRi(){ return Ri;};
Matrix3f GetR(){ return R;};
Vector3f Gett(){ return t;};
Vector3f Getti(){ return ti;};
MatrixXf GetQout(){ return Qout;};
double Geterr(){ return err;};
MatrixXf Getxcam(){ return Xcam;};


private:

MatrixXf x3d;
MatrixXf x2d;
MatrixXf x2dn;
MatrixXf Xw;
MatrixXf Xcam;

Matrix3f Ri;
Matrix3f K;
Vector3f ti;
MatrixXf Qout;

Matrix3f R;
Vector3f t;

double obj_err;

double err;

//Lu parameters
 double  TOL;
 double EPSILON;

 int MAX_ITER;


};
