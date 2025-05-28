#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.h"
#include "sophus/se3.h"

using namespace std;

int main(int argc, char** argv)
{
    // SO3 operations
    // a 90 degree rotation matrix along z-axis
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Sophus::SO3 SO3_R(R);            // contruct SO3 from R
    Sophus::SO3 SO3_V(0, 0, M_PI/2); // Or rotation vector
    Eigen::Quaterniond q(R);         // Or quaternion
    Sophus::SO3 SO3_q(q);
    
    cout << "SO3 from matrix: " << SO3_R << endl;
    cout << "SO3 from vector: " << SO3_V << endl;
    cout << "SO3 from quaternion: " << SO3_q << endl;
    
    // use log affine to get Lie algebra
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    
    // hat -> from vector to skew matrix
    cout << "so3 hat = " << Sophus::SO3::hat(so3) << endl;
    
    // vee -> from skew matrix to vector
    cout << "so3 hat vee = " << Sophus::SO3::vee( Sophus::SO3::hat(so3) ).transpose() << endl;
    
    // perturbation model update
    Eigen::Vector3d update_so3(1e-4, 0,0); // update value
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3)*SO3_R; // left-side multiply 
    cout << "SO3 updated = " << SO3_updated << endl;
    
    cout << "**********************************************" << endl;
    
    // SE(3) operations
    Eigen::Vector3d t(1, 0, 0); //  translation 1 along x-axis
    Sophus::SE3 SE3_Rt(R, t);   // contruct SE(3) from R, t
    Sophus::SE3 SE3_qt(q, t);    // Or from q, t
    cout << "SE3 from R,t = " << endl << SE3_Rt << endl;
    cout << "SE3 from q,t = " << endl << SE3_qt << endl;
    
    // Lie algebra se3 is a 6-dimensional vector
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;
    
    // hat and vee
    cout << "se3 hat = " << endl << Sophus::SE3::hat(se3) << endl;
    cout << "se3 hat vee = " << Sophus::SE3::vee( Sophus::SE3::hat(se3) ).transpose() << endl;
    
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;
    
    return 0;
}
