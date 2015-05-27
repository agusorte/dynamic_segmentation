#include "Lu.h"
#include <string>
#include <iostream>
#include <vector>
#include "/usr/include/eigen3/Eigen/SVD"
#include <cmath>
 Lu::Lu(){
   TOL = 1e-5;//1E-5;
 EPSILON = 1e-8;//1E-8;

  MAX_ITER=10000;
 
 }
 
 Lu::~Lu(){
 }
 
void Lu::Initialize(MatrixXf x3d_h,MatrixXf x2d_h,  Matrix3f A) {
 

 

 x2d=x2d_h.transpose();
  // std::cout<<"x2dtrasn"<<x2d<<std::endl;
 x3d=x3d_h.transpose();
 //std::cout<<"x3dtrasn"<<x3d<<std::endl;

 x2dn=A.inverse ()*x2d;
 //std::cout<<"x2dn"<<x2dn<<std::endl;
 K=A;
  //std::cout<<"Intialize points "<<std::endl; 
}

void Lu::compute(){

 int n=x3d.cols();
 
 //std::cout<<"No. points "<<n<<std::endl;

  int val= objpose(x3d.block(0,0,3,n), x2dn.block(0,0,2,n));
  
  //Estimate 3d position of the points in camera coordinates
  MatrixXf  xcam=MatrixXf::Zero(n,3); //recovered 3d positions optional
  MatrixXf x3daux=x3d.block(0,0,3,n);
  Vector3f xw;
  Xcam=MatrixXf::Zero(3,n); 
  

  
 for (int i=0; i<n;i++){
 
   xw=x3daux.col(i);

   Xcam.col(i)=K*(R*xw+t);
   
   Xcam.col(i)=Xcam.col(i)/Xcam(2,i);
   
   //std::cout<<"xcam \n"<<Xcam<<std::endl;
   
    
   }
}



/*OBJPOSE(P, Qp) compute the pose (exterior orientation)
  between the 3D point set P represented in object space 
  and its projection Qp represented in normalized image 
  plane. It implements the algorithm described in "Fast 
  and Globally Convergent Pose Estimation from Video 
  Images" by Chien-Ping Lu et. al. to appear in IEEE 
  Transaction on Pattern Analysis and Machine intelligence*/
  
int Lu::objpose(MatrixXf P, MatrixXf Qp)
{
  int n = P.cols();
  int i;
 // move the origin to the center of P
// std::cout<<"P\n"<<P<<std::endl; 
  
  Vector3f pbar=Vector3f::Zero();
 // std::cout<<"pbar\n"<<pbar<<std::endl;
    
  for (i = 0; i<n;i++){
 // std::cout<<"Pcol\n"<<P.col(i)<<std::endl; 
      pbar=pbar+P.col(i);
   }
 //std::cout<<"pbar\n"<<pbar; 
  pbar= pbar/n;
  
 // std::cout<<"move the origin to the center of P"<<std::endl;
 //std::cout<<"P\n"<<P;
 //std::cout<<"pbar\n"<<pbar<<std::endl; 
  
  for (i = 0; i<n;i++){
     P.col(i) = P.col(i)-pbar;
  }
 
 //std::cout<<"P\n"<<P<<std::endl; 
  MatrixXf Q;

  Q=MatrixXf::Zero(3,n);
  
for (i = 0 ;i<n;i++){

  Q(0,i)=Qp(0,i);
    Q(1,i)=Qp(1,i);
      Q(2,i)=1;
}

//std::cout<<"Q\n"<<Q<<std::endl; 
//create matrix F with 0's
//Matrix3f maux;

//for (i=0;i<n;i++ ){
//  F.push_back(Matrix3f::Zero(3,3));
//}

 
RowVector3f V;
V= RowVector3f::Zero();
std::vector<Matrix3f > F;
Matrix3f F_aux;
 
 for (i = 0 ;i<n;i++){
    V[0] = Q(0,i)/Q(2,i);
    V[1] = Q(1,i)/Q(2,i);
    V[2] = Q(2,i)/Q(2,i);
   //std::cout<<"V\n"<<V<<std::endl; 
   
   Matrix3f F_aux;
   F_aux=(V.transpose()*V)/V.dot(V);
   
   //std::cout<<"F_aux\n"<<F_aux<<std::endl; 
 //  F_aux = (V*V.transpose())/(V.transpose()*V);
   F.push_back(F_aux);
}

 //compute the matrix factor required to compute t  
 Matrix3f F_sum=Matrix3f::Zero(3,3);
 
  for (i = 0 ;i< n;i++){
    F_sum=F_sum + F[i];
  }
    
  //std::cout<<"F_sum\n"<<F_sum<<std::endl;
  
  Matrix3f tFactor = (Matrix3f::Identity()-F_sum/n);
  tFactor=tFactor.inverse()/n;
  
 //std::cout<<"tfactor\n"<<tFactor<<std::endl;
  
  MatrixXf Ri_;
  MatrixXf ti_;
  MatrixXf Qi_;
  double old_err;
  
   // compute initial pose estimate
  abskernel(P, Q, F, tFactor);
  //Lu::abskernel(MatrixXf P, MatrixXf Q, std::vector<Matrix3f > F, MatrixXf G)
   
   
  Ri_=GetRi();
  ti_=Getti();
  Qi_=GetQout();
   int it = 1;
     
  old_err=Geterr();

  
  /*std::cout<<"Ri_\n"<<Ri_<<std::endl;
  std::cout<<"ti_\n"<<ti_<<std::endl;
  std::cout<<"Qi_\n"<<Qi_<<std::endl;
 std::cout<<"old_err\n"<<old_err<<std::endl;*/
  
   //std::cout<<"old_err\n"<<old_err<<std::endl;

// compute next pose estimate
   abskernel(P, Qi_, F, tFactor);
   Ri_=GetRi();
   ti_=Getti();
   Qi_=GetQout();
   
   double new_err=Geterr();

   it = it + 1; 
   
   /*std::cout<<"Ri_2\n"<<Ri_<<std::endl;
  std::cout<<"ti_2\n"<<ti_<<std::endl;
  std::cout<<"Qi_2\n"<<Qi_<<std::endl;
 std::cout<<"new_err\n"<<new_err<<std::endl;*/
  //std::cout<<"new_err\n"<<new_err<<std::endl;
 
  double val=(old_err-new_err)/old_err;
    val=sqrt(val*val);
  /* std::cout<<"val\n"<<val<<std::endl;
   std::cout<<"TOL\n"<<TOL<<std::endl;
   std::cout<<"EPSILON\n"<<EPSILON<<std::endl;
   std::cout<<"MAX_ITER\n"<<MAX_ITER<<std::endl;*/
   while ((val > TOL) && (new_err > EPSILON)  && (it<MAX_ITER)){

    old_err = new_err;
   //std::cout<<"old_err\n"<<old_err<<std::endl;
    // compute the optimal estimate of R
    abskernel(P, Qi_, F, tFactor);
    Ri_=GetRi();
    ti_=Getti();
    Qi_=GetQout();
    new_err=Geterr();
    it = it + 1;
  }
   
   
   R = Ri;
   t = ti_;
   obj_err = sqrt(new_err/n);
   
  //double img_err = sqrt(img_err/n);
   t = t - Ri*pbar;

  
   return 0;

}

void Lu::data_show()
{
   
   std::cout << "input data"<<std::endl;
   std::cout << "K\n" << K << std::endl;
      
   std::cout<<"----after of contructor"<<std::endl;   
   std::cout << "x3d\n" << x3d<< "\nx2d:\n"<< x2d  << std::endl;
   std::cout << "x2dn\n" << x2dn<< std::endl;    
   std::cout << "N points: \n" << x2dn.cols()<< std::endl;    
}

// XFORMPROJ - Transform and project
//  XFORMPROJ(P, R, t) transform the 3D point set P by 
//  rotation R and translation t, and then project them   to the normalized image plane
MatrixXf Lu::xformproj(MatrixXf P, Matrix3f R, Vector3f t)
{
   int n = P.cols();
   MatrixXf Q=MatrixXf::Zero(3,n);
   MatrixXf Qp=MatrixXf::Zero(2,n);;
   
   
   for (int i=0;i<=n;i++){
    Q.col(i) = R*P.col(i)+t;
    Qp.col(i)=Q.col(i)/Q(3,i);
   }


}

// XFORM - Transform
//   XFORM(P, R, t) transform the 3D point set P by rotation
//   R and translation t
   
MatrixXf Lu::xform(MatrixXf P, Matrix3f  R, Vector3f t){

   int n = P.cols();
   MatrixXf Q=MatrixXf::Zero(3,n);
   
   for (int i=0;i<n;i++){
      Q.col(i) = R*P.col(i)+t;
    }
       //std::cout<<"Q\n"<<Q<<std::endl;
       return Q;
       
  
}

// Return the 3x3 rotation matrix described by a set of Roll, Pitch and Yaw
// angles.
MatrixXf Lu::rpyMat (Vector3f angs){

double cosA = cos (angs[2]);
double sinA = sin (angs[2]);
double cosB = cos (angs[1]);
double sinB = sin (angs[1]);
double cosC = cos (angs[0]);
double sinC = sin (angs[0]);

double cosAsinB = cosA * sinB;
double sinAsinB = sinA * sinB;

//R = [ cosA.*cosB  cosAsinB.*sinC-sinA.*cosC  cosAsinB.*cosC+sinA.*sinC ;
 //     sinA.*cosB  sinAsinB.*sinC+cosA.*cosC  sinAsinB.*cosC-cosA.*sinC ;
 //       -sinB            cosB.*sinC                 cosB.*cosC         ];


 MatrixXf R_(3,3);
 R_<< cosA*cosB,cosAsinB*sinC-sinA*cosC , cosAsinB*cosC+sinA*sinC , sinA*cosB, sinAsinB*sinC+cosA*cosC, sinAsinB*cosC-cosA*sinC, -sinB, cosB*sinC, cosB*cosC;
 
 return R_;

}

// RANDROTMAT - Random rotation matrix
//   RANDROTMAT(n) generates n uniformly distributed
//   rotation matrix

MatrixXf Lu::randrotmat(int n){

  MatrixXf Rs;
  //Rs= MatrixXf::Zero(3,3,n);
  
  return Rs;
}

//QUAT2MAT - Convert a quaternion to a 3x3 rotation matrix
MatrixXf Lu::quat2mat(Vector4f q){

float a = q[1]; 
float b = q[2]; 
float c = q[3]; 
float d = q[4];
  MatrixXf R_(3,3);

R_<< a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c), 
2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b),
2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c;

return R_;
}

// QMATW - Compute the W matrix (4x4) of quaternion q
MatrixXf Lu::qmatW(Vector4f  q){
float w = q[1]; 
float x = q[2]; 
float y = q[3]; 
float z = q[4];

Matrix4f W;

W << w, -x, -y, -z,x, w, z, -y,y, -z, w, x,z, y, -x, w;

return W;

}

//QMATQ - Compute the Q matrix (4x4) of quaternion q

MatrixXf Lu::qmatQ(Vector4f  q){

float w = q[1]; 
float x = q[2]; 
float y = q[3]; 
float z = q[4];

Matrix4f Q;

Q<< w, -x, -y, -z, x, w, -z, y,y, z, w, -x,z, -y, x, w;

return Q;


}

void Lu::abskernel(MatrixXf P, MatrixXf Q, std::vector<Matrix3f > F, MatrixXf G){

   int n = P.cols();
   int i;
   
   //std::cout<<"P\n"<<P<<std::endl;
   //std::cout<<"Q\n"<<Q<<std::endl;
   //std::cout<<"-------------\n"<<std::endl;
   
   for( i=0; i<n;i++){
      Q.col(i)=F[i]*Q.col(i);
      
    //  std::cout<<"F\n"<<F[i]*Q.col(i)<<std::endl;
      
   }
   //std::cout<<"Q\n"<<Q<<std::endl;
   
   // compute P' and Q'
    Vector3f pbar;
    Vector3f qbar;
    Vector3f P_sum;
  
    Vector3f Q_sum;
    
    P_sum= Vector3f::Zero();
    Q_sum= Vector3f::Zero();
    
    for( i=0; i<n;i++){
      P_sum = P_sum+P.col(i);
      Q_sum = Q_sum+Q.col(i);
    }
    
    pbar=P_sum/n;
    qbar=Q_sum/n;
  // std::cout<<"pbar\n"<<pbar<<std::endl;
   // std::cout<<"qbar\n"<<qbar<<std::endl;

    for( i=0; i<n;i++){
      P.col(i)=P.col(i)-pbar;
      Q.col(i)=Q.col(i)-qbar;
    }
    
    //std::cout<<"P\n"<<P<<std::endl;
    //std::cout<<"Q\n"<<Q<<std::endl;
    /////////////use SVM method
    Matrix3f M=Matrix3f::Zero();
    Vector3f vq;
    Vector3f vp;
    Matrix3f Vs;
    
    for( i=0; i<n;i++){
       vq=Q.col(i);
       vp=P.col(i);
       
      Vs << vp[0]*vq[0],vp[0]*vq[1],vp[0]*vq[2],
            vp[1]*vq[0],vp[1]*vq[1],vp[1]*vq[2],
            vp[2]*vq[0],vp[2]*vq[1],vp[2]*vq[2];
      
       M = M+Vs;
    }
    
    //std::cout<<"M\n"<<M<<std::endl;
    //Matrix
    //M.svd().compute();
    JacobiSVD<Matrix3f> svdA(M,ComputeFullU | ComputeFullV);
    //svdA.compute();
    
    Matrix3f U=svdA.matrixU();
    Matrix3f V=svdA.matrixV();   
    
    Ri=V*U.transpose();
    
    //std::cout<<"U\n"<<U<<std::endl;
    //std::cout<<"V\n"<<V<<std::endl;
    //std::cout<<"Ri\n"<<Ri<<std::endl;
    
    if (sgn(Ri.determinant())==1.0){
     ti= estimate_t( Ri,G,F,P,n );
     //std::cout<<"ti\n"<<ti<<std::endl;
     
     if (ti(2)<0){
      //R=-[V(:,1:2) -V(:,3)]*U.';
      Matrix3f Va;
      Va<<V(0,0),V(0,1),-V(0,2),
          V(1,0),V(1,1),-V(1,2),
          V(2,0),V(2,1),-V(2,2);
           
       Ri=-Va*U.transpose();
       ti = estimate_t( Ri,G,F,P,n );
     }
     
    }else{
      
      Matrix3f Va;
      Va<<V(0,0),V(0,1),-V(0,2),
          V(1,0),V(1,1),-V(1,2),
          V(2,0),V(2,1),-V(2,2);
         // std::cout<<"Va\n"<<ti<<std::endl; 
     //R=[V(:,1:2) -V(:,3)]*U.';
      Ri=Va*U.transpose();
      
       ti = estimate_t( Ri,G,F,P,n );

    if (ti(2) < 0){ 
      //disp(['t_3 = ' num2str(t(3)) ]);
      // we need to invert the t 
      Ri =- V*U.transpose();
      ti = estimate_t( Ri,G,F,P,n );
  
    }
    
    }
   //  std::cout<<"ti\n"<<ti<<std::endl;
    
  Qout = xform(P, Ri, ti);
    // std::cout<<"Qout\n"<<Qout<<std::endl;

// calculate error
   err = 0;
   Vector3f vec;
   
for (i = 0 ;i< n; i++){
  vec = (Matrix3f::Identity()-F[i])*Qout.col(i);
  //std::cout<<"vec\n"<<vec<<std::endl;
  err = err + vec.dot(vec);
}

    //std::cout<<"err\n"<<err<<std::endl;
}


Vector3f Lu::estimate_t( MatrixXf R,MatrixXf G,std::vector<Matrix3f > F,MatrixXf P,int n ){


Vector3f sum_ =Vector3f::Zero();

for (int i=0;i< n; i++){
   sum_=sum_+F[i]*R*P.col(i);
} 
   return G*sum_;
}

int Lu::sgn(double d){

   if (d>0)
      return 1;
   else 
      return 0;
     

}



