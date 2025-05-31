#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        cout << "usage: feature_extraction img1 img2 " << endl;
        return 1;
    }
    
    // read image
    Mat img1 = imread(argv[1], IMREAD_COLOR);
    Mat img2 = imread(argv[2], IMREAD_COLOR);
    
    // initialization
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<ORB> orb = ORB::create ( 500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
    
    // detect Oriented FAST corner point
    orb->detect (img1, keypoints_1);
    orb->detect (img2, keypoints_2);
    
    // calculate BRIEF descriptor
    orb->compute (img1, keypoints_1, descriptors_1);
    orb->compute (img2, keypoints_2, descriptors_2);
    
    Mat outImg1;
    drawKeypoints(img1, keypoints_1, outImg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB feature points", outImg1);
    imwrite("out_keypoints.png", outImg1);
    waitKey(0);
    
    // match descriptors from img1 and img2, use Hamming distance
    vector<DMatch> matches;
    BFMatcher matcher (NORM_HAMMING);
    matcher.match (descriptors_1, descriptors_2, matches);
    
    // sifting matched points
    double min_dist = 10000, max_dist = 0;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );
    
    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if ( matches[i].distance <= max (2.0*min_dist, 30.0) )
        {
            good_matches.push_back(matches[i]);
        }
    }
    
    // plotting result
    Mat img_match;
    Mat img_goodMatch;
    drawMatches( img1, keypoints_1, img2, keypoints_2, matches, img_match );
    drawMatches ( img1, keypoints_1, img2, keypoints_2, good_matches, img_goodMatch );
    
    imshow( "all matches", img_match);
    imwrite("all_matches.png", img_match);
    imshow("Optimized", img_goodMatch);
    imwrite("out_good_matches.png", img_goodMatch);
    waitKey(0);
    
    return 0;
}
