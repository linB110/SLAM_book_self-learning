#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main (int argc, char** argv)
{
    cv::Mat image(200, 200, CV_8UC1);  // use random to create a picture
    cv::theRNG().state = 42;   // set seed to obtain same result via each imshow
    cv::randu(image, 0, 256);
    cv::imshow("Random Grayscale image", image);
    cv::waitKey(0);
    
    cout << "image width is " << image.cols << " image height is " << image.rows << endl;
    cv::waitKey(0);
    
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3 ){
        cout << "iamge is not either BGR or Gray" << endl;
        return -1;
     }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    
    // traverse each pixel of the image
    for (size_t y = 0; y < image.rows; y++){
        for (size_t x = 0; x < image.cols; x++){
            unsigned char* row_ptr = image.ptr<unsigned char> (y); // row pointer is point to row y
            unsigned char* data_ptr = &row_ptr [x*image.channels()]; // data pointer point to the pixel data to br accessed
            
            for (int c = 0; c < image.channels(); c++){
                unsigned char data = data_ptr[c];  // data = I(x, y) and channel c
            }
        }
    }
    
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration <double> time_used = chrono::duration_cast<chrono::duration<double>> (t2 - t1);
    cout << "time consumed for traversing the image is : " << time_used.count() << " sec" << endl;
    
    // if image is changed, image_new is also affected, vice versa
    cv::Mat image_new = image;  
    image_new ( cv::Rect (0, 0, 100, 100 )).setTo(0);  // top left (100, 100) = 0
    cv::imshow("image_new", image);
    cv::waitKey(0);
    
    // use clone to avoid interference
    cv::Mat image_clone = image.clone();
    image_clone (cv::Rect (0, 0, 100, 100)).setTo(255);  // top left (100, 100) = 255
    cv::imshow ( "image_clone", image_clone );
    cv::waitKey ( 0 );
    
    cv::destroyAllWindows();
    return 0;
}
