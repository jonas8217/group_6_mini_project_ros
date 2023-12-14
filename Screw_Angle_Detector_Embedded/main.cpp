#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>


using namespace std;
using namespace cv;

struct Sobel_Outputs{
    Mat Sobel_Image, Sobel_Direction;
};

Mat Grayscale(Mat Image, float R_Weight = 0.299, float G_Weight = 0.587, float B_Weight = 0.114){
    // Get image size
    int Rows = Image.rows;
    int Cols = Image.cols;

    // Create ouput matrix (CV_8UC1 define the channels as being only grayscale not RGB)
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

Mat Gaussian_Filter(Mat Image, int Radius = 1){
    // Get image size
    int Rows = Image.rows;
    int Cols = Image.cols;

    // Initialize standard deviation from radius
    int Standard_Deviation = 0;

    if(1 > (int)(Radius/2)){
        Standard_Deviation = 1;
    }
    else{
        Standard_Deviation = Radius/2;
    }

    // Initialise blur kernel size (needs to be odd)
    int Kernel_Width = 1+2*Radius;

    // Create ouput matrix (CV_8UC1 define the channels as being only grayscale not RGB)
    Mat Blur_Image = Mat::zeros(Size(Cols-Radius*2,Rows-Radius*2), CV_8UC1);


    // Initialise Kernel
    float Kernel[Kernel_Width][Kernel_Width];

    // Initialise Sum
    float Sum = 0;

    // fill every kernel position with gaussian distribution
    for(int x = 0; x < Kernel_Width; x++){
        for(int y = 0; y < Kernel_Width; y++){
            float Exponent_Numerator = ((x-Radius)*(x-Radius)+(y-Radius)*(y-Radius));
            float Exponent_Denominator = 2*Standard_Deviation*Standard_Deviation;
            float e_Expression = exp(-(Exponent_Numerator/Exponent_Denominator));
            float Kernel_Value = e_Expression/(2*M_PI*Standard_Deviation*Standard_Deviation);
            Kernel[x][y] = Kernel_Value;
            Sum += Kernel_Value;
        }
    }

    //Normalize the kernel
    for(int x = 0; x < Kernel_Width; x++){
        for(int y = 0; y < Kernel_Width; y++){
            Kernel[x][y] = Kernel[x][y]/Sum;
        }
    }

    // Apply to image
    for(int i = Radius; i < Cols-Radius; i++){
        for(int j = Radius; j < Rows-Radius; j++){
            float Value = 0;
            // Sum up all weights
            for(int x = 0; x < Kernel_Width; x++){
                for(int y = 0; y < Kernel_Width; y++){
                    float Weight  = Kernel[x][y];
                    Value +=  Image.at<uchar>(Point(i+(x-Radius),j+(y-Radius)))*Weight;
                }
             }
             Blur_Image.at<uchar>(Point(i-Radius,j-Radius)) = Value;
         }
    }
    return Blur_Image;
}

Sobel_Outputs Sobel(Mat Image){
    // Get image size
    int Rows = Image.rows;
    int Cols = Image.cols;

    // Create ouput matrix (CV_8UC1 define the channels as being only grayscale not RGB)
    Mat Sobel_Image = Mat::zeros(Size(Cols,Rows), CV_8UC1);
    Mat Sobel_Direction = Mat::zeros(Size(Cols,Rows), CV_8UC1);

    // Magnitude array
    float Magnitudes[Cols][Rows];
    float G1_Sums[Cols][Rows];
    float G2_Sums[Cols][Rows];

    Sobel_Outputs Outputs;

    float Magnitude_Max = 0;
    float G1_Max = 0;
    float G2_Max = 0;

    // Calculate sums
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            // If edge set to zero since we need to look at a square
            if(i == 0 || i == Cols-1 || j == 0 || j == Rows-1){
                G1_Sums[i][j] = 0;
                G2_Sums[i][j] = 0;
            }
            // Else look at square to determine if edge
            else{
                // Calculate row changes
                float G1[3];
                G1[0] = Image.at<uchar>(Point(i-1,j-1)) + 2*Image.at<uchar>(Point(i,j-1)) + Image.at<uchar>(Point(i+1,j-1));
                G1[1] = 0;
                G1[2] =  -Image.at<uchar>(Point(i-1,j+1)) - 2*Image.at<uchar>(Point(i,j+1)) - Image.at<uchar>(Point(i+1,j+1));
                float G1_Sum = G1[0]+G1[1]+G1[2];
                // Calculate col changes
                float G2[3];
                G2[0] = Image.at<uchar>(Point(i-1,j-1)) - Image.at<uchar>(Point(i+1,j-1));
                G2[1] = 2*Image.at<uchar>(Point(i-1,j)) - 2*Image.at<uchar>(Point(i+1,j));
                G2[2] = Image.at<uchar>(Point(i-1,j+1)) - Image.at<uchar>(Point(i+1,j+1));
                float G2_Sum = G2[0]+G2[1]+G2[2];

                G1_Sums[i][j] = G1_Sum;
                G2_Sums[i][j] = G2_Sum;

                if(G1_Sum > G1_Max){
                    G1_Max = G1_Sum;
                }
                if(G2_Sum > G2_Max){
                    G2_Max = G2_Sum;
                }
            }
        }
    }

    // Make sure they are under 255
    //for(int i = 0; i < Cols; i++){
    //    for(int j = 0; j < Rows; j++){
    //        G1_Sums[i][j] = (G1_Sums[i][j]/G1_Max)*255;
    //        G2_Sums[i][j] = (G2_Sums[i][j]/G2_Max)*255;
    //    }
    //}


    // Go through each pixel
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            float Pixel_Magnitude = sqrt((G1_Sums[i][j]*G1_Sums[i][j])+(G2_Sums[i][j]*G2_Sums[i][j]));

            float Theta = atan2(G1_Sums[i][j],G2_Sums[i][j]);
            float Angle = Theta*180/M_PI;
            if(Angle < 0){
                Angle += 180;
            }

            if(Pixel_Magnitude > Magnitude_Max){
                Magnitude_Max = Pixel_Magnitude;
            }
            Magnitudes[i][j] = Pixel_Magnitude;
            Sobel_Direction.at<uchar>(Point(i,j)) = Angle;
        }
    }

    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            Sobel_Image.at<uchar>(Point(i,j)) = (Magnitudes[i][j]/Magnitude_Max)*255;
        }
    }


    Outputs.Sobel_Image = Sobel_Image;
    Outputs.Sobel_Direction= Sobel_Direction;
    return Outputs;
}

Mat Non_Maximum_Suppression(Mat Image, Mat Directions){
    // Get image size
    int Rows = Image.rows;
    int Cols = Image.cols;

    // Create ouput matrix (CV_8UC1 define the channels as being only grayscale not RGB)
    Mat Line_Image = Mat::zeros(Size(Cols,Rows), CV_8UC1);

    // Go through each pixel
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){

            // Initialize two compare pixels
            int Pixel_1 = 255;
            int Pixel_2 = 255;

            // Define angle
            float Angle = Directions.at<uchar>(Point(i,j));

            // if angle between 0 and 22.5 or between 157.5 and 180
            if((0 <= Angle < 22.5) || (157.5 <= Angle <= 180)){
                Pixel_1 =  Image.at<uchar>(Point(i,j-1));
                Pixel_2 =  Image.at<uchar>(Point(i,j+1));
            }
            // if angle between 22.5 and 67.5
            else if(22.5 <= Angle < 67.5){
                Pixel_1 =  Image.at<uchar>(Point(i-1,j+1));
                Pixel_2 =  Image.at<uchar>(Point(i+1,j-1));
            }
            // If angle between 67.5 and 112.5
            else if(67.5 <= Angle < 112.5){
                Pixel_1 =  Image.at<uchar>(Point(i-1,j));
                Pixel_2 =  Image.at<uchar>(Point(i+1,j));
            }
            // If angle between 112.5 and 157.5
            else if(112.5 <= Angle < 157.5){
                Pixel_1 =  Image.at<uchar>(Point(i+1,j+1));
                Pixel_2 =  Image.at<uchar>(Point(i-1,j-1));
            }

            // Check if current pixel is bigger than the two others
            if((Image.at<uchar>(Point(i,j)) >= Pixel_1) && (Image.at<uchar>(Point(i,j)) >= Pixel_2)){
                Line_Image.at<uchar>(Point(i,j)) = Image.at<uchar>(Point(i,j));
            }
            else{
                Line_Image.at<uchar>(Point(i,j)) = 0;
            }
        }
    }
    return Line_Image;
}

Mat Double_Thresholding(Mat Image, int High_Threshold = 200, int Low_Threshold = 100){
    // Get image size
    int Rows = Image.rows;
    int Cols = Image.cols;

    // Create ouput matrix (CV_8UC1 define the channels as being only grayscale not RGB)
    Mat Improved_Line_Image = Mat::zeros(Size(Cols,Rows), CV_8UC1);

    // Go through each pixel
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){

            // If strong edge
            if(Image.at<uchar>(Point(i,j)) > High_Threshold){
                Improved_Line_Image.at<uchar>(Point(i,j)) = 255;
            }
            // Remove weak edges
            else if(Image.at<uchar>(Point(i,j)) < Low_Threshold){
                Improved_Line_Image.at<uchar>(Point(i,j)) = 0;
            }
            // In betweens will be handled later so set to 150
            else{
                Improved_Line_Image.at<uchar>(Point(i,j)) = 150;
            }
        }
    }

    // Now hysteresis is completed. Inbetween points are only strong if near other strong points
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            if(Improved_Line_Image.at<uchar>(Point(i,j)) == 150){
                if(
                        (Improved_Line_Image.at<uchar>(Point(i+1,j-1)) == 255) ||
                        (Improved_Line_Image.at<uchar>(Point(i+1,j)) == 255) ||
                        (Improved_Line_Image.at<uchar>(Point(i+1,j+1)) == 255) ||
                        (Improved_Line_Image.at<uchar>(Point(i,j-1)) == 255) ||
                        (Improved_Line_Image.at<uchar>(Point(i,j+1)) == 255) ||
                        (Improved_Line_Image.at<uchar>(Point(i-1,j-1)) == 255) ||
                        (Improved_Line_Image.at<uchar>(Point(i-1,j)) == 255) ||
                        (Improved_Line_Image.at<uchar>(Point(i-1,j+1)) == 255)
                        ){
                    Improved_Line_Image.at<uchar>(Point(i,j)) = 255;
                }
                else{
                    Improved_Line_Image.at<uchar>(Point(i,j)) = 0;
                }
            }
        }
    }
    return Improved_Line_Image;
}

vector<Vec4i> Hough_Transform(Mat Image){
    // Get image size
    int Rows = Image.rows;
    int Cols = Image.cols;

    // Image diagonal
    int d = sqrt(Rows*Rows+Cols*Cols);

    // Define Teta and Rho
    vector<int> Rho;
    int Rho_Min = -d;
    int Rho_Max = d;
    int Rho_Step = 1;

    for(int i = Rho_Min; i <= Rho_Max; i += Rho_Step){
        Rho.push_back(i);
    }

    vector<int> Teta;
    int Teta_Min = -90;
    int Teta_Max = 90;
    int Teta_Step = 1;

    for(int i = Teta_Min; i <= Teta_Max; i += Teta_Step){
        Teta.push_back(i);
    }

    // Define accumulator


    // Go through each pixel
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            // If pixel is a line
            if(Image.at<uchar>(Point(i,j)) == 255){
                // Vary teta from 0 to pi
                for(float Teta = 0; Teta <= M_PI; Teta += 0.1){
                    float p = i*cos(Teta)+j*sin(Teta);
                }
            }
        }
    }
}

Mat OpenCV_Probabilistic_Hough_Transform(Mat Original_Image, int Thresh, int Min_Length, int Max_Gap){
    // Get image size
    int Rows = Original_Image.rows;
    int Cols = Original_Image.cols;
    Mat Gray_Image;
    cvtColor(Original_Image, Gray_Image,COLOR_BGR2GRAY);

    Mat Edge_Image;

    Canny(Gray_Image,Edge_Image, 10, 223);

    vector<Vec4i> Lines;

    Mat Result_Image = Original_Image.clone();

    HoughLinesP(Edge_Image,Lines,1,M_PI/180,Thresh,Min_Length,Max_Gap);
    for(size_t i = 0; i < Lines.size();i++){
        Vec4i Line = Lines[i];
        line(Result_Image, Point(Line[0],Line[1]), Point(Line[2],Line[3]),Scalar(255,0,0),3,LINE_4);
    }
    return Result_Image;
}

//__________________________________________________________________________________________________________________________
// My new version using arrays

// Converts Mat image to a BGR vector
vector<vector<vector<int>>> Mat_To_Vec_BGR(Mat BGR_Image){
    cout << "Converting image to vector" << endl;

    // Get image size
    int Rows = BGR_Image.rows;
    int Cols = BGR_Image.cols;

    // Create static array
    vector<vector<vector<int>>> BGR_Matrix_Vector;

    // Insert all image values
    for(int i = 0; i < Cols; i++){
        vector<vector<int>> BGR_Row_Vector;
        for(int j = 0; j < Rows; j++){
            // Read BGR values
            Vec3b BGR = BGR_Image.at<Vec3b>(Point(i,j));
            vector<int> BGR_Vector{BGR[0], BGR[1], BGR[2]};
            BGR_Row_Vector.push_back(BGR_Vector);
        }
        BGR_Matrix_Vector.push_back(BGR_Row_Vector);
    }
    return BGR_Matrix_Vector;
}

// Convert vector to mat image
Mat Vec_2D_To_Mat(vector<vector<float>> Float_Vector){
    cout << "Converting 2D vector to OpenCV matrix" << endl;
    // Get sizes
    int Cols = Float_Vector.size();
    int Rows = Float_Vector[0].size();

    // Create ouput matrix (CV_8UC1 define the channels as being only grayscale not RGB)
    Mat Matrix = Mat::zeros(Size(Cols,Rows), CV_8UC1);

    // Insert vector elements into matrix
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            Matrix.at<uchar>(Point(i,j)) = static_cast<int>(Float_Vector[i][j]);
        }
    }
    return Matrix;
}

// Grayscales a vector matrix
vector<vector<float>> Grayscale_Vector_Image(vector<vector<vector<int>>> BGR_Vector_Image, float R_Weight = 0.299, float G_Weight = 0.587, float B_Weight = 0.114){
    cout << "Converting to grayscale" << endl;
    // Get sizes
    int Cols = BGR_Vector_Image.size();
    int Rows = BGR_Vector_Image[0].size();

    // Create output vector
    vector<vector<float>> Grayscale_Vector;

    // go through image and grayscale each pixel
    for(int i = 0; i < Cols; i++){
        vector<float> Grayscale_Row_Vector;
        for(int j = 0; j < Rows; j++){
            float Grayscale_Value = B_Weight * BGR_Vector_Image[i][j][0] + G_Weight * BGR_Vector_Image[i][j][1] + R_Weight * BGR_Vector_Image[i][j][2];
            Grayscale_Row_Vector.push_back(Grayscale_Value);
        }
        Grayscale_Vector.push_back(Grayscale_Row_Vector);
    }
    return Grayscale_Vector;
}

// Add gaussian blur to vector matrix
vector<vector<float>> Gaussian_Blur_Vector_Image(vector<vector<float>> Grayscale_Vector_Image, int Radius = 1){
    cout << "Adding gaussian blur" << endl;
    // Get sizes
    int Cols = Grayscale_Vector_Image.size();
    int Rows = Grayscale_Vector_Image[0].size();

    // Initialize standard deviation from radius
    int Standard_Deviation = 0;

    if(1 > Radius/2){
        Standard_Deviation = 1;
    }
    else{
        Standard_Deviation = Radius/2;
    }

    // Initialise blur kernel size (needs to be odd)
    int Kernel_Width = 1+2*Radius;

    // Create ouput vector
    vector<vector<float>> Blurred_Vector;


    // Initialise Kernel
    float Kernel[Kernel_Width][Kernel_Width];

    // Initialise Sum
    float Sum = 0;

    // fill every kernel position with gaussian distribution
    for(int x = 0; x < Kernel_Width; x++){
        for(int y = 0; y < Kernel_Width; y++){
            float Exponent_Numerator = ((x-Radius)*(x-Radius)+(y-Radius)*(y-Radius));
            float Exponent_Denominator = 2*Standard_Deviation*Standard_Deviation;
            float e_Expression = exp(-(Exponent_Numerator/Exponent_Denominator));
            float Kernel_Value = e_Expression/(2*M_PI*Standard_Deviation*Standard_Deviation);
            Kernel[x][y] = Kernel_Value;
            Sum += Kernel_Value;
        }
    }

    //Normalize the kernel
    for(int x = 0; x < Kernel_Width; x++){
        for(int y = 0; y < Kernel_Width; y++){
            Kernel[x][y] = Kernel[x][y]/Sum;
        }
    }

    // Apply to image
    for(int i = Radius; i < Cols-Radius; i++){
        vector<float> Blurred_Row;
        for(int j = Radius; j < Rows-Radius; j++){
            float Value = 0;
            // Sum up all weights
            for(int x = 0; x < Kernel_Width; x++){
                for(int y = 0; y < Kernel_Width; y++){
                    float Weight  = Kernel[x][y];
                    Value +=  Grayscale_Vector_Image[i+(x-Radius)][j+(y-Radius)]*Weight;
                }
             }
            Blurred_Row.push_back(Value);
         }
        Blurred_Vector.push_back(Blurred_Row);
    }
    return Blurred_Vector;
}

// Output struct for sobel
struct Sobel_Vector_Outputs{
    vector<vector<float>> Sobel_Magnitude, Sobel_Direction;
};

// Sobbel line detection on vector matrix
Sobel_Vector_Outputs Sobel_Vector_Image(vector<vector<float>> Blurred_Grayscale_Vector_Image, int Sobel_Threshold = 60){
    cout << "Converting to line image using Sobel" << endl;
    // Get sizes
    int Cols = Blurred_Grayscale_Vector_Image.size();
    int Rows = Blurred_Grayscale_Vector_Image[0].size();

    // Create output vectors
    vector<vector<float>> Sobel_Magnitude_Vector;
    vector<vector<float>> Sobel_Direction_Vector;

    // Initialize kernels
    vector<vector<float>> Col_Kernel{{-1,0,1},{-2,0,2},{-1,0,1}};
    vector<vector<float>> Row_Kernel{{1,2,1},{0,0,0},{-1,-2,-1}};

    // Magnitude array
    float Magnitudes[Cols][Rows];
    float Col_Sums[Cols][Rows];
    float Row_Sums[Cols][Rows];

    Sobel_Vector_Outputs Outputs;

    float Magnitude_Max = 0;
    float Col_Max = 0;
    float Row_Max = 0;

    // Calculate sums
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            // If edge set to zero since we need to look at a square
            if(i == 0 || i == Cols-1 || j == 0 || j == Rows-1){
                Col_Sums[i][j] = 0;
                Row_Sums[i][j] = 0;
            }
            // Else look at square to determine if edge
            else{
                vector<vector<float>> A;
                // Create square
                for(int col = -1; col < 2; col++){
                    vector<float> A_Row;
                    for(int row = -1; row < 2; row++){
                        A_Row.push_back(Blurred_Grayscale_Vector_Image[i+col][j+row]);
                    }
                    A.push_back(A_Row);
                }

                // Calculate row changes
                float Col_Sum = 0;
                float Row_Sum = 0;
                for(int col = 0; col < 3; col++){
                    for(int row = 0; row < 3; row++){
                        Col_Sum += A[col][row]*Col_Kernel[col][row];
                        Row_Sum += A[col][row]*Row_Kernel[col][row];
                    }
                }

                Col_Sums[i][j] = Col_Sum;
                Row_Sums[i][j] = Row_Sum;

                if(Col_Sum > Col_Max){
                    Col_Max = Col_Sum;
                }
                if(Row_Sum > Row_Max){
                    Row_Max = Row_Sum;
                }
            }
        }
    }

    // Go through each pixel
    for(int i = 0; i < Cols; i++){
        vector<float> Row_Direction_Vector;
        for(int j = 0; j < Rows; j++){
            // Calculate magnitude
            float Pixel_Magnitude = sqrt((Col_Sums[i][j]*Col_Sums[i][j])+(Row_Sums[i][j]*Row_Sums[i][j]));

            // Calculate angle
            float Theta = atan2(Row_Sums[i][j],Col_Sums[i][j]);
            float Angle = Theta*180/M_PI;
            if(Angle < 0){
                Angle += 180;
            }

            if(Pixel_Magnitude > Magnitude_Max){
                Magnitude_Max = Pixel_Magnitude;
            }
            Magnitudes[i][j] = Pixel_Magnitude;
            Row_Direction_Vector.push_back(Angle);
        }
        Sobel_Direction_Vector.push_back(Row_Direction_Vector);
    }

    // Normalise magnitudes
    for(int i = 0; i < Cols; i++){
        vector<float> Row_Magnitude_Vector;
        for(int j = 0; j < Rows; j++){
            float Normalized_Magnitude = Magnitudes[i][j]*255/Magnitude_Max;
            // Apply homemade threshold
            if(Normalized_Magnitude < Sobel_Threshold){
                Normalized_Magnitude = 0;
            }
            Row_Magnitude_Vector.push_back(Normalized_Magnitude);
        }
        Sobel_Magnitude_Vector.push_back(Row_Magnitude_Vector);
    }


    Outputs.Sobel_Magnitude = Sobel_Magnitude_Vector;
    Outputs.Sobel_Direction= Sobel_Direction_Vector;
    return Outputs;
}

// Perform non maximum suppression on vectors
vector<vector<float>> Non_Max_Supression_Vector_Image(vector<vector<float>> Sobel_Magnitudes, vector<vector<float>> Sobel_Directions){
    cout << "Performing non maximum supression" << endl;
    // Get sizes
    int Cols = Sobel_Magnitudes.size();
    int Rows = Sobel_Magnitudes[0].size();

    // Create output vector matrix
    vector<vector<float>> Suppressed_Vector_Image;

    // Go through each pixel
    for(int i = 0; i < Cols; i++){
        vector<float> Row_Vector;
        for(int j = 0; j < Rows; j++){

            // Initialize two compare pixels
            int Pixel_1 = 255;
            int Pixel_2 = 255;

            // Define angle
            float Angle = Sobel_Directions[i][j];

            // if angle between 0 and 22.5 or between 157.5 and 180
            if(((0 <= Angle) && (Angle < 22.5)) || ((157.5 <= Angle) && (Angle <= 180))){
                Pixel_1 = Sobel_Magnitudes[i][j-1];
                Pixel_2 = Sobel_Magnitudes[i][j+1];
            }
            // if angle between 22.5 and 67.5
            else if((22.5 <= Angle)  && (Angle < 67.5)){
                Pixel_1 = Sobel_Magnitudes[i-1][j+1];
                Pixel_2 = Sobel_Magnitudes[i+1][j-1];
            }
            // If angle between 67.5 and 112.5
            else if((67.5 <= Angle) && (Angle < 112.5)){
                Pixel_1 = Sobel_Magnitudes[i-1][j];
                Pixel_2 = Sobel_Magnitudes[i+1][j];
            }
            // If angle between 112.5 and 157.5
            else if((112.5 <= Angle) && (Angle < 157.5)){
                Pixel_1 = Sobel_Magnitudes[i+1][j+1];
                Pixel_2 = Sobel_Magnitudes[i-1][j-1];
            }

            // Check if current pixel is bigger than the two others
            if((Sobel_Magnitudes[i][j] >= Pixel_1) && (Sobel_Magnitudes[i][j]>= Pixel_2)){
                Row_Vector.push_back(Sobel_Magnitudes[i][j]);
            }
            else{
                Row_Vector.push_back(0);
            }
        }
        Suppressed_Vector_Image.push_back(Row_Vector);
    }
    return Suppressed_Vector_Image;
}

// Perform double thresholding on vector image
vector<vector<float>> Double_Threholding_Vector_Image(vector<vector<float>> Suppressed_Vector_Image, int High_Threshold = 200, int Low_Threshold = 100){
    cout << "Performing double thresholding" << endl;
    // Get sizes
    int Cols = Suppressed_Vector_Image.size();
    int Rows = Suppressed_Vector_Image[0].size();

    // Create output vector matrix
    vector<vector<float>> Thresholded_Vector_Image;

    // Go through each pixel
    for(int i = 0; i < Cols; i++){
        vector<float> Row_Vector;
        for(int j = 0; j < Rows; j++){

            // If strong edge
            if(Suppressed_Vector_Image[i][j] > High_Threshold){
                Row_Vector.push_back(255);
            }
            // Remove weak edges
            else if(Suppressed_Vector_Image[i][j] < Low_Threshold){
                Row_Vector.push_back(0);
            }
            // In betweens will be handled later so set to 150
            else{
                Row_Vector.push_back(150);
            }
        }
        Thresholded_Vector_Image.push_back(Row_Vector);
    }

    // Now hysteresis is completed. Inbetween points are only strong if near other strong points
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            if(Thresholded_Vector_Image[i][j] == 150){
                if(
                        (Thresholded_Vector_Image[i+1][j-1] == 255) ||
                        (Thresholded_Vector_Image[i+1][j] == 255) ||
                        (Thresholded_Vector_Image[i+1][j+1] == 255) ||
                        (Thresholded_Vector_Image[i][j-1] == 255) ||
                        (Thresholded_Vector_Image[i][j+1] == 255) ||
                        (Thresholded_Vector_Image[i-1][j-1] == 255) ||
                        (Thresholded_Vector_Image[i-1][j] == 255) ||
                        (Thresholded_Vector_Image[i-1][j+1] == 255)
                        ){
                    Thresholded_Vector_Image[i][j] = 255;
                }
                else{
                    Thresholded_Vector_Image[i][j] = 0;
                }
            }
        }
    }
    return Thresholded_Vector_Image;
}

// Find lines in the image using hough transform
struct Hough_Outputs{
    vector<vector<float>> Hough_Image_Vector;
    vector<float> Hough_Lines;
};

Hough_Outputs Hough_Transform_Vector_Image(vector<vector<float>> Thresholded_Vector_Image, int Accumulator_Size = 20){
    Hough_Outputs Outputs;

    return Outputs;
}

// Main control function for detection angles
void Angle_Detector(string Image_Path){

    // Read image using openCV
    Mat Image = imread(Image_Path);

    // Check if image was read correctly
    if(Image.empty()){
        cout << "ERROR 1: Image not found" << endl;
    }
    else{
        cout << "Image read correctly" << endl;

        // Get working vector
        vector<vector<vector<int>>> BGR_Vector = Mat_To_Vec_BGR(Image);

        // Grayscale image
        vector<vector<float>> Grayscale_Vector = Grayscale_Vector_Image(BGR_Vector,0.3,0.59,0.11);

        // Get grayscale image for viewing
        Mat Grayscale_Image = Vec_2D_To_Mat(Grayscale_Vector);

        // Add gaussian blur
        vector<vector<float>> Blurred_Vector = Gaussian_Blur_Vector_Image(Grayscale_Vector,1);

        // Get blured image for viewing
        Mat Blurred_Image = Vec_2D_To_Mat(Blurred_Vector);

        // Perform line detection using sobel
        Sobel_Vector_Outputs Sobel_Vectors;

        Sobel_Vectors = Sobel_Vector_Image(Grayscale_Vector);
        vector<vector<float>> Sobel_Magnitudes = Sobel_Vectors.Sobel_Magnitude;
        vector<vector<float>> Sobel_Directions = Sobel_Vectors.Sobel_Direction;

        // Get sobel images for viewing
        Mat Sobel_Magnitude_Image = Vec_2D_To_Mat(Sobel_Magnitudes);
        Mat Sobel_Direction_Image = Vec_2D_To_Mat(Sobel_Directions);

        // Perform non maximum suppresion
        vector<vector<float>> Suppressed_Vector = Non_Max_Supression_Vector_Image(Sobel_Magnitudes,Sobel_Directions);

        // Get suppressed image for viewing
        Mat Suppressed_Image = Vec_2D_To_Mat(Suppressed_Vector);

        // Perform hysteris thresholding
        vector<vector<float>> Thresholded_Vector = Double_Threholding_Vector_Image(Suppressed_Vector,50,30);

        // Get thresholded image for viewing
        Mat Threshold_Image = Vec_2D_To_Mat(Thresholded_Vector);


        imshow("Original image", Image);
        imshow("Grayscale image", Grayscale_Image);
        imshow("Blurred image", Blurred_Image);
        imshow("Sobel magnitude image", Sobel_Magnitude_Image);
        imshow("Sobel direction image", Sobel_Direction_Image);
        imshow("Suppressed image", Suppressed_Image);
        imshow("Thresholded image", Threshold_Image);
        waitKey(0);
    }
}


int main()
{
    Angle_Detector("/home/benjamin/Screw_Angle_Detector_Embedded/Cross_Image.png");

//    // Read image
//    Mat Image = imread("/home/benjamin/Screw_Angle_Detector_Embedded/Cross_Image.png");

//    // Check if image was read correctly
//    if(Image.empty()){
//        cout << "ERROR 1: Image not found" << endl;
//    }
//    else{
//        Mat Grayscale_Image = Grayscale(Image,0.33,0.33,0.33);
//        Mat Blur_Image = Gaussian_Filter(Grayscale_Image,8);
//        Sobel_Outputs Sobel_Results;
//        Sobel_Results = Sobel(Blur_Image);
//        Mat Sobel_Image = Sobel_Results.Sobel_Image;
//        Mat Sobel_Direction = Sobel_Results.Sobel_Direction;
//        Mat Line_Image = Non_Maximum_Suppression(Sobel_Image,Sobel_Direction);
//        Mat Updated_Line_Image = Double_Thresholding(Line_Image,100,50);

//        // OpenCV try
//        Mat Hough_Image = OpenCV_Probabilistic_Hough_Transform(Image,8,3,5);

//        imshow("Original image", Image);
//        imshow("Grayscale image",Grayscale_Image);
//        imshow("Blurred image",Blur_Image);
//        imshow("Sobel image",Sobel_Image);
//        imshow("Sobel Directions",Sobel_Direction);
//        imshow("Non maximum image",Line_Image);
//        imshow("Hysteresis Threshold", Updated_Line_Image);
//        imshow("Hough image",Hough_Image);
//        waitKey(0);
//    }

    return 0;
}
