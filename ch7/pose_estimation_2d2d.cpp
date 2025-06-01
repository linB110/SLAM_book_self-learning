#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
//#include "extra.h" 

using namespace std;
using namespace cv;

void find_feature_matches (
    const Mat& img1, const Mat& img2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector<DMatch>& matches);
    
void pose_estimation_2d2d (
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch>& matches,
    Mat& R, Mat& t );
    
// pixel to normalized cordinate
Point2d pixel2cam (const Point2d& p, const Mat& k );

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        cout << "usage: pose_estimation_2d2d img1 img2 " << endl;
        return 1;
    }
    
    // read image
    Mat img1 = imread(argv[1], IMREAD_COLOR);
    Mat img2 = imread(argv[2], IMREAD_COLOR);
    
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img1, img2, keypoints_1, keypoints_2, matches );
    cout << "found " << matches.size() << " matched point" << endl;
    
    // estimate motion from 2 images
    Mat R, t;
    pose_estimation_2d2d (keypoints_1, keypoints_2, matches, R, t);
    
    // verify E = t^R*scale
    Mat t_x = ( Mat_<double> ( 3,3 ) <<
                0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
                t.at<double> ( 2,0 ),      0,                      -t.at<double> ( 0,0 ),
                -t.at<double> ( 1,0 ),     t.at<double> ( 0,0 ),      0 );

    cout << "t^R=" << endl << t_x*R << endl;
    
    // verify epipolar constraint
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 
                                      0, 521.0, 249.7, 
                                      0, 0, 1 );
    for ( DMatch m: matches )
    {
        Point2d pt1 = pixel2cam ( keypoints_1[ m.queryIdx ].pt, K );
        Mat y1 = ( Mat_<double> ( 3,1 ) << pt1.x, pt1.y, 1 );
        Point2d pt2 = pixel2cam ( keypoints_2[ m.trainIdx ].pt, K );
        Mat y2 = ( Mat_<double> ( 3,1 ) << pt2.x, pt2.y, 1 );
        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }
    
    return 0;
}

void find_feature_matches(const Mat& img1, const Mat& img2,
                          std::vector<KeyPoint>& keypoints_1,
                          std::vector<KeyPoint>& keypoints_2,
                          std::vector<DMatch>& matches) 
{
    // initialization
    Mat descriptors_1, descriptors_2;
    //Ptr<FeatureDetector> detector = FeatureDetector::create("ORB");
    //Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create("ORB");
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    Ptr<Feature2D> orb = ORB::create();
    
    // detect Oriented FAST corner point
    orb->detect (img1, keypoints_1);
    orb->detect (img2, keypoints_2);
    
    // calculate ORB descriptor
    orb->compute (img1, keypoints_1, descriptors_1);
    orb->compute (img2, keypoints_2, descriptors_2);
    
    // use Hamming Distance to match two descriptors
    vector<DMatch> match;
    //BFMatcher matcher(NORM_HAMMING);
    matcher->match (descriptors_1, descriptors_2, match);
    
    // sift matched point
    double min_dist = 10000.0;
    double max_dist = 0.0;
    
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );
    
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= max (2*min_dist, 30.0))
        {
            matches.push_back(match[i]);
        }
    }
}

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

void pose_estimation_2d2d(std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches,
                            Mat& R, Mat& t)
{
    // camera intrinsics
    Mat K = ( Mat_<double> (3, 3) << 520.9, 0, 325.1, 
                                     0, 521.0, 249.7,
                                     0, 0, 1);
    vector<Point2f> points_1;
    vector<Point2f> points_2;
    
    for (int i = 0; i < (int)matches.size(); i++)
    {
        points_1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points_2.push_back(keypoints_2[matches[i].queryIdx].pt);
    }
    
    // calculate fundamental Matrix
    Mat fundamentalMat;
    fundamentalMat = findFundamentalMat(points_1, points_2, FM_8POINT);
    cout << "fundamental matrix is : " << endl << fundamentalMat << endl;
    
    // calculate essential Matrix
    Point2d principal_point (325.1, 249.7);
    double focal_length = 521;
    Mat essentialMat;
    essentialMat = findEssentialMat(points_1, points_2, focal_length, principal_point);
    cout << "essential matrix is : " << endl << essentialMat << endl;
    
    // calcualte homography matrix
    Mat homoMat;
    homoMat = findHomography(points_1, points_2, RANSAC, 3);
    cout << "homography matrix is : " << endl << homoMat << endl;
    
    // get R, t from homography matrix
    recoverPose (essentialMat, points_1, points_2, R, t, focal_length, principal_point);
    cout << "R is : " << endl << R << endl;
    cout << "t is : " << endl << t << endl;
}
                         
