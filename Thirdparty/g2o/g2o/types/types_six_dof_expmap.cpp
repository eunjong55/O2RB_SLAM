// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "types_six_dof_expmap.h"

#include "../core/factory.h"
#include "../stuff/macros.h"

namespace g2o {

using namespace std;

Vector2d project2d(const Vector3d& v)  {
  Vector2d res;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

Vector3d unproject2d(const Vector2d& v)  {
  Vector3d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>() {
}

bool VertexSE3Expmap::read(std::istream& is) {
  Vector7d est;
  for (int i=0; i<7; i++)
    is  >> est[i];
  SE3Quat cam2world;
  cam2world.fromVector(est);
  setEstimate(cam2world.inverse());
  return true;
}

bool VertexSE3Expmap::write(std::ostream& os) const {
  SE3Quat cam2world(estimate().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  return os.good();
}

// sphere model:: x_sphere(3d), X(3d), Twc(RT)
// EdgeSphereSE3ProjectXYZ :: 3d_world_pt 2 3d_sphere_pt
Vector3d EdgeSphereSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz) const{
  Vector3d res;
  double norm_sphere = sqrt(trans_xyz[0]*trans_xyz[0] + trans_xyz[1]*trans_xyz[1] + trans_xyz[2]*trans_xyz[2]);

  res[0] =  trans_xyz[0] / norm_sphere * rad ;
  res[1] =  trans_xyz[1] / norm_sphere * rad ;
  res[2] =  trans_xyz[2] / norm_sphere * rad ;

  return res;
}

Vector3d EdgeSphereSE3ProjectXYZOnlyPose::cam_project(const Vector3d & trans_xyz) const{
  Vector3d res;
  double norm_sphere = sqrt(trans_xyz[0]*trans_xyz[0] + trans_xyz[1]*trans_xyz[1] + trans_xyz[2]*trans_xyz[2]);

  res[0] =  trans_xyz[0] / norm_sphere * rad;
  res[1] =  trans_xyz[1] / norm_sphere * rad;
  res[2] =  trans_xyz[2] / norm_sphere * rad;

  return res;
}

// sphere model:: x_sphere(3d), X(3d), Twc(RT)
// EdgeSphereSE3ProjectXYZ ::
EdgeSphereSE3ProjectXYZ::EdgeSphereSE3ProjectXYZ() : BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>() {
}
// sphere model:: x_sphere(3d), X(3d), Twc(RT)
// EdgeSphereSE3ProjectXYZ ::
bool EdgeSphereSE3ProjectXYZ::read(std::istream& is){
  for (int i=0; i<=2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}
// sphere model:: x_sphere(3d), X(3d), Twc(RT)
// EdgeSphereSE3ProjectXYZ ::
bool EdgeSphereSE3ProjectXYZ::write(std::ostream& os) const {
  for (int i=0; i<=2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}
// sphere model:: x_sphere(3d), X(3d), Twc(RT)
// sphere model: EdgeSphereSE3ProjectXYZ ::
void EdgeSphereSE3ProjectXYZ::linearizeOplus() {

  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]); //RT vertex estimation
  SE3Quat T(vj->estimate());
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]); //3D_Pt vertex estimation
  Vector3d xyz = vi->estimate();//from 3D_pt vertex X

  Vector3d xyz_trans = T.map(xyz);//from RT vertex, World2Cam_origin  X'
  
  const Matrix3d R =  T.rotation().toRotationMatrix();//R
  // cout  << "R\n" << R << endl;

  double x_T = xyz_trans[0];
  double y_T = xyz_trans[1];
  double z_T = xyz_trans[2];

  double D = sqrt(x_T*x_T + y_T*y_T + z_T*z_T);//D

  Matrix3d eye_mat =  Matrix3d::Identity(3, 3);//I

  double size = sqrt( pow(x_T,2) + pow(y_T,2) + pow(z_T, 2) );

  Matrix3d lo = Matrix3d::Identity(3, 3);
  lo(0,0) = rad * (size - pow(x_T, 2)/size) / pow(size, 2);
  lo(0,1) = -1* rad * x_T * y_T / pow(size, 3);
  lo(0,2) = -1* rad * x_T * z_T / pow(size, 3);

  lo(1,0) = -1* rad * y_T * x_T / pow(size, 3);
  lo(1,1) = rad * (size - pow(y_T, 2)/size) / pow(size, 2);
  lo(1,2) = -1* rad * y_T * z_T / pow(size, 3);

  lo(2,0) = -1* rad * z_T * x_T / pow(size, 3);
  lo(2,1) = -1* rad * z_T * y_T / pow(size, 3);
  lo(2,2) = rad * (size - pow(z_T, 2)/size) / pow(size, 2);

  _jacobianOplusXi = -1 * lo * R;

  MatrixXd Xj_R = skew(-1*xyz_trans);
  MatrixXd Xj_T = eye_mat;
  MatrixXd RT(Xj_R.rows(), Xj_R.cols()+Xj_T.cols());
  RT << Xj_R, Xj_T;
  

  _jacobianOplusXj = -1 * lo * RT;

}

//Only Pose

// sphere model:: x_sphere(3d), X(3d), Twc(RT)
//todo
bool EdgeSphereSE3ProjectXYZOnlyPose::read(std::istream& is){
  for (int i=0; i<=2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

// sphere model:: x_sphere(3d), X(3d), Twc(RT)
//todo
bool EdgeSphereSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

  for (int i=0; i<=2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

// sphere model:: x_sphere(3d), X(3d), Twc(RT)
// sphere model: EdgeSphereSE3ProjectXYZOnlyPose::
void EdgeSphereSE3ProjectXYZOnlyPose::linearizeOplus() {
  // VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);//RT vertex estimation
  
  // Vector3d xyz_trans = vi->estimate().map(Xw);//X', X
  // // const Matrix3d R = vi->estimate().rotation().toRotationMatrix();//R

  // double x_T = xyz_trans[0];//X'
  // double y_T = xyz_trans[1];
  // double z_T = xyz_trans[2];
  // double D = sqrt(x_T*x_T + y_T*y_T + z_T*z_T);//D
  // Matrix3d eye_mat =  Matrix3d::Identity(3, 3);//I

  // double tmp0 = -1.0 * (rad  / D );
  // Eigen::Matrix3d tmp1 = 1/(D*D) * xyz_trans * xyz_trans.transpose();

  // MatrixXd Xj_R = tmp0 * ((skew(-1*Xw)) - tmp1 * (skew(-1*Xw)));
  // MatrixXd Xj_T = tmp0 * (eye_mat - tmp1 * eye_mat);
  // _jacobianOplusXi(0,0) = Xj_R(0,0);//x_obs_RT
  // _jacobianOplusXi(0,1) = Xj_R(0,1);
  // _jacobianOplusXi(0,2) = Xj_R(0,2);
  // _jacobianOplusXi(0,3) = Xj_T(0,0);
  // _jacobianOplusXi(0,4) = Xj_T(0,1);
  // _jacobianOplusXi(0,5) = Xj_T(0,2);

  // _jacobianOplusXi(1,0) = Xj_R(1,0);//y_obs_RT
  // _jacobianOplusXi(1,1) = Xj_R(1,1);
  // _jacobianOplusXi(1,2) = Xj_R(1,2);
  // _jacobianOplusXi(1,3) = Xj_T(1,0);
  // _jacobianOplusXi(1,4) = Xj_T(1,1);
  // _jacobianOplusXi(1,5) = Xj_T(1,2);

  // _jacobianOplusXi(2,0) = Xj_R(2,0);//z_obs_RT
  // _jacobianOplusXi(2,1) = Xj_R(2,1);
  // _jacobianOplusXi(2,2) = Xj_R(2,2);
  // _jacobianOplusXi(2,3) = Xj_T(2,0);
  // _jacobianOplusXi(2,4) = Xj_T(2,1);
  // _jacobianOplusXi(2,5) = Xj_T(2,2);

    VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);//RT vertex estimation
    
    Vector3d xyz_trans = vi->estimate().map(Xw);

    double x_T = xyz_trans[0];
    double y_T = xyz_trans[1];
    double z_T = xyz_trans[2];

    double size = sqrt( pow(x_T,2) + pow(y_T,2) + pow(z_T, 2) );

    Matrix3d eye_mat =  Matrix3d::Identity(3, 3);//I
    Matrix3d lo = Matrix3d::Identity(3, 3);
    lo(0,0) = rad * (size - pow(x_T, 2)/size) / pow(size, 2);
    lo(0,1) = -1* rad * x_T * y_T / pow(size, 3);
    lo(0,2) = -1* rad * x_T * z_T / pow(size, 3);

    lo(1,0) = -1* rad * y_T * x_T / pow(size, 3);
    lo(1,1) = rad * (size - pow(y_T, 2)/size) / pow(size, 2);
    lo(1,2) = -1* rad * y_T * z_T / pow(size, 3);

    lo(2,0) = -1* rad * z_T * x_T / pow(size, 3);
    lo(2,1) = -1* rad * z_T * y_T / pow(size, 3);
    lo(2,2) = rad * (size - pow(z_T, 2)/size) / pow(size, 2);

    MatrixXd Xj_R = skew(-1*xyz_trans);
    MatrixXd Xj_T = eye_mat;

    MatrixXd RT(Xj_R.rows(), Xj_R.cols()+Xj_T.cols());

    RT << Xj_R, Xj_T;
    _jacobianOplusXi = -1 * lo * RT;

}

} // end namespace
