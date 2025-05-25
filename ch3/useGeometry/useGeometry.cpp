#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char** argv)
{
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d (0, 0, 1) );  // rotate 45 degree along z-axis
    cout .precision(3);
    cout << "rotation matrix = \n " << rotation_vector.matrix() << endl;
    
    rotation_matrix = rotation_vector.toRotationMatrix();
    
    
    // cordinate transform
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout<<"(1,0,0) after rotation = "<< v_rotated.transpose() << endl;
    
    // use rotation matrix
    v_rotated = rotation_matrix * v;
    cout<<"(1,0,0) after rotation = "<< v_rotated.transpose() << endl;
    
    // Euler angle : transfer rotation matrix to Euler angle
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles (2, 1, 0); // ZYX -> yaw roll pitch
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;
    
    /* Euclidean transform */
    // use Eigen::Isometry --> Euclidean transform matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); // actually a 4*4 matrix
    T.rotate(rotation_vector); // rotate by rotation vector
    T.pretranslate( Eigen::Vector3d (1, 3, 4) ); // translation vector is (1,3,4)
    cout << "transform matrix = \n" << T.matrix() << endl ;
    
    // use transform matrix to conduct cordinate transform
    Eigen::Vector3d v_transformed = T * v;
    cout << "v tranformed = "<< v_transformed.transpose() << endl; // = R*v+t
    
    /* Affine transform */
    Eigen::Affine3d affine = Eigen::Affine3d::Identity();
    affine.rotate(rotation_vector);                         // rotation
    affine.pretranslate(Eigen::Vector3d(2, 0, 0));          // translation (2, 0, 0)
    Eigen::Vector3d v_affined = affine * v;
    cout << "v after affine transform = " << v_affined.transpose() << endl;
    cout << "affine matrix = \n" << affine.matrix() << endl;

    /* Projective transform */
    Eigen::Projective3d projective = Eigen::Projective3d::Identity();
    projective.rotate(rotation_vector);
    projective.pretranslate(Eigen::Vector3d(0, 2, 0));  // translation (0, 2, 0)

    Eigen::Vector4d v_h;
    v_h << v(0), v(1), v(2), 1;

    Eigen::Vector4d v_projected_h = projective.matrix() * v_h;
    Eigen::Vector3d v_projected = v_projected_h.head<3>() / v_projected_h(3);  // optional perspective division
    cout << "v after projective transform = " << v_projected.transpose() << endl;
    cout << "projective matrix = \n" << projective.matrix() << endl;
    
    /* quarternion */
    Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
    cout << "quaternion = \n" << q.coeffs() << endl; // coeffs = (x, y, z, w), w is real part and (x,y,z) are imaginary part
    
    q = Eigen::Quaterniond ( rotation_matrix );
    cout << "quaternion = \n " << q.coeffs() << endl;
    v_rotated = q*v;  // use quarternion to rotate a vector
    cout << " (1,0,0) after rotation = " << v_rotated.transpose() << endl; 
    
    return 0;
}
