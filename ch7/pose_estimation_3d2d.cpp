#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>
#include <memory>

using namespace std;
using namespace cv;

void find_feature_matches (
    const Mat& img1, const Mat& img2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector<DMatch>& matches);

// pixel to normalized cordinate
Point2d pixel2cam (const Point2d& p, const Mat& k );

void bundleAdjustment(
    const vector<Point3f> point_3d,
    const vector<Point2f> point_2d,
    const Mat& K,
    Mat& R, Mat& t);
    
int main (int argc, char** argv)
{
    if (argc != 5)
    {
        cout << "usage pose_estimation_3d2d img1 img2 depth1 depth2 " << endl;
    }
    
    // read image
    Mat img1 = imread(argv[1], IMREAD_COLOR);
    Mat img2 = imread(argv[2], IMREAD_COLOR);
    
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img1, img2, keypoints_1, keypoints_2, matches );
    cout << "found " << matches.size() << " matched point" << endl;
    
    // construct 3d points
    Mat d1 = imread (argv[3], IMREAD_UNCHANGED);  
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 
                                      0, 521.0, 249.7, 
                                      0, 0, 1 );
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for (DMatch m : matches)
    {
        ushort d = d1.ptr<unsigned short> (int (keypoints_1[m.queryIdx].pt.y)) [int (keypoints_1[m.queryIdx].pt.x)];
        if (d == 0)
            continue;
        
        float dd = d / 5000.0;
        Point2d p1 = pixel2cam (keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back(Point3f (p1.x*dd, p1.y*dd, dd));
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);
    }
    
    cout << "3d-2d pairs: " << pts_3d.size() << endl;
    
    Mat r, t;
    solvePnP (pts_3d, pts_2d, K, Mat(), r, t, false );  // can change Opencv Pnp (false here) to EPNP, DLS...
    r.convertTo(r, CV_64F);
    t.convertTo(t, CV_64F);
    
    Mat R;
    cv::Rodrigues(r, R); // r os rotation vector and R is rotation matrix
    
    cout << "R = " << endl << R << endl;
    cout << "t = " << endl << t << endl;
    
    cout << "calling bundle adjustment " << endl;
    
    bundleAdjustment(pts_3d, pts_2d, K, R, t);
    
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

void bundleAdjustment(
    const vector<Point3f> point_3d,
    const vector<Point2f> point_2d,
    const Mat& K,
    Mat& R, Mat& t)
{
    // initialize g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    typedef g2o::LinearSolverCSparse<Block::PoseMatrixType> LinearSolver;

    std::unique_ptr<LinearSolver> linearSolver = std::make_unique<LinearSolver>();
    std::unique_ptr<Block> solver_ptr = std::make_unique<Block>(std::move(linearSolver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));


    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    
    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
               
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                        ) );
                        
    optimizer.addVertex ( pose );
    
    int index = 1;
    for (const Point3f p : point_3d) // traverse landmarks
    {
        g2o::VertexPointXYZ* point = new g2o::VertexPointXYZ();
        point->setId(index++);
        point->setEstimate(Eigen::Vector3d (p.x, p.y, p.z));
        point->setMarginalized (true);
        optimizer.addVertex(point);
    }
    
    // camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters(
        K.at<double> (0, 0), Eigen::Vector2d (K.at<double> (0, 2), K.at<double> (1, 2)), 0
    );
    camera->setId(0);
    optimizer.addParameter(camera);
    
    // edges
    index = 1;
    for (const Point2f p : point_2d)
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }
    
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

    cout << endl << "after optimization:" << endl;
    cout << "T=" << endl << Eigen::Isometry3d ( pose->estimate() ).matrix() << endl;
}
