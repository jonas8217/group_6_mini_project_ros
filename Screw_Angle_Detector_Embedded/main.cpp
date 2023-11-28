#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Mat Grayscale(Mat Image, float R_Weight = 0.299, float G_Weight = 0.587, float B_Weight = 0.114){
    // Get image size
    int Rows = Image.rows;
    int Cols = Image.cols;

    // Create ouput matrix (CV_64FC1 define the channels as being only grayscale not RGB)
    Mat Grayscale_Image = Mat::zeros(Size(Cols,Rows), CV_8UC1);

    // go through image and grayscale each pixel
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            Vec3b BGR = Image.at<Vec3b>(Point(i,j));
            uchar Gray_Pixel = R_Weight * BGR[2] + G_Weight * BGR[1] + B_Weight * BGR[0];
            Grayscale_Image.at<uchar>(Point(i,j)) = Gray_Pixel;
        }
    }
    return Grayscale_Image;
}

Mat Sobel(Mat Image){

}

int main()
{
    // Read image
    Mat Image = imread("/home/benjamin/Screw_Angle_Detector_Embedded/test_image2.jpg");

    // Check if image was read correctly
    if(Image.empty()){
        cout << "ERROR 1: Image not found" << endl;
    }
    else{
        Mat Grayscale_Image = Grayscale(Image);
        imshow("Resulting image", Grayscale_Image);
        waitKey(0);
    }

    return 0;
}
