#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main (int argc, char** argv)
{
    Eigen::Quaterniond q1 (0.35, 0.2, 0.3, 0.1);
    Eigen::Vector3d t1 (0.3, 0.1, 0.1);
    
    Eigen::Quaterniond q2 (-0.5, 0.4, -0.1, 0.2);
    Eigen::Vector3d t2 (-0.1, 0.5, 0.3);
    
    q1.normalize();
    q2.normalize();
    
    // construct Tcw1 (world to camera 1)
    Eigen::Isometry3d Tcw1 = Eigen::Isometry3d::Identity();
    Tcw1.rotate(q1);
    Tcw1.pretranslate(t1);
    
    // construct Tcw2 (world to camera 2)
    Eigen::Isometry3d Tcw2 = Eigen::Isometry3d::Identity();
    Tcw2.rotate(q2);
    Tcw2.pretranslate(t2);
    
    /* from formula Tcw1 * pw = p1 (point p from world frame to camera1)
                    Tcw2 * pw = p2 (point p from world frame to camera2)
    */
    Eigen::Vector3d p1 (0.5, 0, 0.2);  // p1
    Eigen::Vector3d pw = Tcw1.inverse() * p1; // get point p in world frame
    Eigen::Vector3d p2 = Tcw2 * pw; // get p2 from formula
    
    cout << "Point p in world frame = \n" << pw << endl;
    cout << "point p in camera1 = \n" << Tcw1 * pw << endl;
    cout << "point p in camera2 = \n" << p2 << endl;
}
