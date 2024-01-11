#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>

using namespace std;
using namespace cv;

// Functionality functions

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
vector<vector<float>> Grayscale_Image(vector<vector<vector<int>>> BGR_Vector_Image, float R_Weight = 0.299, float G_Weight = 0.587, float B_Weight = 0.114){
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
vector<vector<float>> Gaussian_Blur_Image(vector<vector<float>> Grayscale_Vector_Image, int Radius = 1){
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

// Cutoff everything but center of image
vector<vector<float>> Cutoff_Image(vector<vector<float>> Image, int Cutoff_Thresh = 30){
    cout << "Cutting off most of image" << endl;
    // Get sizes
    int Cols = Image.size();
    int Rows = Image[0].size();

    // Find middle
    int Middle_Col = Cols/2;
    int Middle_Row = Rows/2;

    // Result vector
    vector<vector<float>> Result_Image;

    // Keep only pixels within threshold
    for(int i = 0; i < Cols; i++){
        // Working row vector
        vector<float> Row_Vector;
        for(int j = 0; j < Rows; j++){
            if(i < Middle_Col-Cutoff_Thresh || i > Middle_Col+Cutoff_Thresh || j < Middle_Row-Cutoff_Thresh || j > Middle_Row+Cutoff_Thresh){
                Row_Vector.push_back(0);
            }
            else{
                Row_Vector.push_back(Image[i][j]);
            }
        }
        Result_Image.push_back(Row_Vector);
    }

    return Result_Image;

}

// Output struct for sobel
struct Sobel_Vector_Outputs{
    vector<vector<float>> Sobel_Magnitude, Sobel_Direction;
};

// Sobbel line detection on vector matrix
Sobel_Vector_Outputs Sobel_Image(vector<vector<float>> Blurred_Grayscale_Vector_Image){
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
            Row_Magnitude_Vector.push_back(Normalized_Magnitude);
        }
        Sobel_Magnitude_Vector.push_back(Row_Magnitude_Vector);
    }


    Outputs.Sobel_Magnitude = Sobel_Magnitude_Vector;
    Outputs.Sobel_Direction= Sobel_Direction_Vector;
    return Outputs;
}

// Perform non maximum suppression on vectors
vector<vector<float>> Non_Max_Supression_Image(vector<vector<float>> Sobel_Magnitudes, vector<vector<float>> Sobel_Directions){
    cout << "Performing non maximum supression" << endl;
    // Get sizes
    int Cols = Sobel_Magnitudes.size();
    int Rows = Sobel_Magnitudes[0].size();

    // Find circle center vectors
    vector<int> Circle_Col_Indexes;
    vector<int> Circle_Row_Indexes;

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
vector<vector<float>> Double_Threholding_Image(vector<vector<float>> Suppressed_Vector_Image, int High_Threshold = 200, int Low_Threshold = 100){
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

// Find center of all lines on image
vector<int> Find_Center(vector<vector<float>> Image, int Strength_Threshold = 150){
    cout << "Finding center of lines" << endl;
    // Get sizes
    int Cols = Image.size();
    int Rows = Image[0].size();

    vector<int> Line_x_Values;
    vector<int> Line_y_Values;

    // Find all x and y values
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            // If pixel is white keep its coordinates
            if(Image[i][j] >= Strength_Threshold){
                Line_x_Values.push_back(i);
                Line_y_Values.push_back(j);
            }
        }
    }

    // Calculate center point
    float Sum_x = 0;
    float Sum_y = 0;
    for(int i = 0; i < Line_x_Values.size(); i++){
        Sum_x += Line_x_Values[i];
    }
    for(int i = 0; i < Line_y_Values.size(); i++){
        Sum_y += Line_y_Values[i];
    }
    int Center_x = Sum_x/Line_x_Values.size();
    int Center_y = Sum_y/Line_y_Values.size();

    vector<int> Center_Point = {Center_x,Center_y};

    return Center_Point;
}

// Draw a center point
Mat Draw_Point(Mat Image, vector<int> Draw_Point, int Colour = 255){
    Mat Drawn_Image = Image.clone();
    circle(Drawn_Image,Point(Draw_Point[0],Draw_Point[1]),0,Scalar(Colour,Colour,Colour), 4,8,0);
    return Drawn_Image;
}

// Find end points
struct Endpoints{
    vector<vector<int>> Points;
    vector<vector<float>> Point_Image;
};

Endpoints Find_Endpoints(vector<vector<float>> Image, int Desired_Point_Count = 4){
    cout << "Finding endpoints" << endl;
    // Get sizes
    int Cols = Image.size();
    int Rows = Image[0].size();

    Endpoints Outputs;

    vector<vector<int>> Points;
    vector<vector<float>> Point_Image = Image;

    vector<vector<int>> Endpoint_Kernel = {{1,1,1},{1,0,1},{1,1,1}};

    // Find all points connected to only one neighbor
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            if(Image[i][j] == 255 && i != 0 && j != 0){
                float Neighbor_Sum = 0;
                for(int col = -1; col <= 1; col++){
                    for(int row = -1; row <= 1; row++){
                        Neighbor_Sum += Image[i+col][j+row]*Endpoint_Kernel[col+1][row+1];
                    }
                }
                if(Neighbor_Sum > 255 || Neighbor_Sum == 0){
                    Point_Image[i][j] = 0;
                }
                else{
                    Point_Image[i][j] = 255;
                    Points.push_back({i,j});
                }
            }
        }
    }

    // If not enough points
    if(Points.size() < Desired_Point_Count){
        cout << "Trying different endpoint strategy" << endl;
        // Look at points with only one direct neighbor
        vector<vector<int>> Simple_Kernel = {{0,1,0},{1,0,1},{0,1,0}};
        for(int i = 0; i < Cols; i++){
            for(int j = 0; j < Rows; j++){
                if(Image[i][j] == 255 && i != 0 && j != 0){
                    float Neighbor_Sum = 0;
                    for(int col = -1; col <= 1; col++){
                        for(int row = -1; row <= 1; row++){
                            Neighbor_Sum += Image[i+col][j+row]*Simple_Kernel[col+1][row+1];
                        }
                    }
                    if(Neighbor_Sum > 255 || Neighbor_Sum == 0){
                        Point_Image[i][j] = 0;
                    }
                    else{
                        Point_Image[i][j] = 255;
                        Points.push_back({i,j});
                    }
                }
            }
        }
    }

    // If still not enough points

    if(Points.size() < Desired_Point_Count){
        cout << "Trying another endpoint strategy" << endl;
        // Look at points with only one direct neighbor
        vector<vector<int>> Kernel_Right = {{0,0,1},{0,1,1},{1,1,1}};
        vector<vector<int>> Kernel_Left = {{1,1,1},{1,1,0},{0,0,0}};
        for(int i = 0; i < Cols; i++){
            for(int j = 0; j < Rows; j++){
                if(Image[i][j] == 255 && i != 0 && j != 0){
                    float Neighbor_Right_Sum = 0;
                    float Neighbor_Left_Sum = 0;
                    for(int col = -1; col <= 1; col++){
                        for(int row = -1; row <= 1; row++){
                            Neighbor_Right_Sum += Image[i+col][j+row]*Kernel_Right[col+1][row+1];
                            Neighbor_Left_Sum += Image[i+col][j+row]*Kernel_Left[col+1][row+1];
                        }
                    }
                    if((Neighbor_Left_Sum > 255 || Neighbor_Left_Sum == 0) && (Neighbor_Right_Sum > 255 || Neighbor_Right_Sum == 0)){
                        Point_Image[i][j] = 0;
                    }
                    else{
                        if(Neighbor_Left_Sum == 255 && Neighbor_Right_Sum == 255){
                            Point_Image[i][j] = 0;
                        }
                        else{
                            Point_Image[i][j] = 255;
                            Points.push_back({i,j});
                        }
                    }
                }
            }
        }
    }
    vector<vector<int>> Final_Points;
    // Remove identical points
    for(int i = 0; i < Points.size();i++){
        bool Identical = false;
        for(int j = i+1;j < Points.size(); j++){
            if(Points[i][0] == Points[j][0] && Points[i][1] == Points[j][1]){
                Identical = true;
            }
        }
        if(Identical == false){
            Final_Points.push_back(Points[i]);
        }
    }
    Outputs.Points = Final_Points;
    Outputs.Point_Image = Point_Image;
    return Outputs;
}

// Find corners
struct Corners{
    vector<vector<int>> Points;
    vector<vector<float>> Point_Image;
};

Corners Find_Corners(vector<vector<float>> Image, int Desired_Corner_Count = 4){
    cout << "Finding Corners" << endl;
    // Get sizes
    int Cols = Image.size();
    int Rows = Image[0].size();

    Corners Outputs;

    vector<vector<int>> Points;
    vector<vector<float>> Point_Image = Image;

    vector<vector<int>> Kernel_Right = {{0,0,1},{0,1,1},{1,1,1}};

    vector<vector<int>> Kernel_Left = {{1,1,1},{1,1,0},{0,0,0}};

    // Find corners
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            if(Image[i][j] == 255 && i != 0 && j != 0){
                float Neighbor_Right_Sum = 0;
                float Neighbor_Left_Sum = 0;
                for(int col = -1; col <= 1; col++){
                    for(int row = -1; row <= 1; row++){
                        Neighbor_Right_Sum += Image[i+col][j+row]*Kernel_Right[col+1][row+1];
                        Neighbor_Left_Sum += Image[i+col][j+row]*Kernel_Left[col+1][row+1];
                    }
                }
                if((Neighbor_Left_Sum > 255 || Neighbor_Left_Sum == 0) && (Neighbor_Right_Sum > 255 || Neighbor_Right_Sum == 0)){
                    Point_Image[i][j] = 0;
                }
                else{
                    if(Neighbor_Left_Sum == 255 && Neighbor_Right_Sum == 255){
                        Point_Image[i][j] = 0;
                    }
                    else{
                        Point_Image[i][j] = 255;
                        Points.push_back({i,j});
                    }
                }
            }
        }
    }

    // Get center
    vector<int> Center = Find_Center(Point_Image);

    // Make lines between all far away points
    vector<vector<int>> Lines;

    float Distance_Threshold = 10;

    for(int i = 0; i < Points.size();i++){
        for(int j = i+1; j < Points.size(); j++){
            float Dist = sqrt((Points[j][0]-Points[i][0])*(Points[j][0]-Points[i][0]) + (Points[j][1]-Points[i][1])*(Points[j][1]-Points[i][1]));
            if(Dist > Distance_Threshold){
                Lines.push_back({Points[i][0],Points[i][1],Points[j][0],Points[j][1]});
            }
        }
    }

    // Calculate angles between all lines
    vector<vector<int>> Keep_Points;
    for(int i = 0; i < Lines.size();i++){
        for(int j = i+1; j < Lines.size(); j++){
            float x1_Change = Lines[i][2]-Lines[i][0];
            float y1_Change = Lines[i][3]-Lines[i][1];

            float x2_Change = Lines[j][2]-Lines[j][0];
            float y2_Change = Lines[j][3]-Lines[j][1];

            float Dot = x1_Change*x2_Change + y1_Change*y2_Change;

            float Norm1 =  sqrt(x1_Change*x1_Change+y1_Change*y1_Change);
            float Norm2 =  sqrt(x2_Change*x2_Change+y2_Change*y2_Change);
            float Line_Angle = acos(Dot/(Norm1*Norm2))*180/M_PI;

            if(abs(Line_Angle) == 90 ){
                cout << Line_Angle << endl;
                Keep_Points.push_back({Lines[i][0],Lines[i][1]});
                Keep_Points.push_back({Lines[i][2],Lines[i][3]});
                Keep_Points.push_back({Lines[j][0],Lines[j][1]});
                Keep_Points.push_back({Lines[j][2],Lines[j][3]});
            }
        }
    }


    // For all similar points keep the closest to center
//    float Distance_Threshold = 10;
//    vector<vector<int>> Keep_Points;
//    for(int i = 0 ; i < Points.size(); i++){
//        vector<int> Close_Points;
//        for(int j = 0; j < Points.size(); j++){
//            float Dist = sqrt((Points[j][0]-Points[i][0])*(Points[j][0]-Points[i][0]) + (Points[j][1]-Points[i][1])*(Points[j][1]-Points[i][1]));
//            if(Dist < Distance_Threshold){
//                Close_Points.push_back(j);
//            }
//        }
//        float Min_Distance = 100;
//        int Keep_Index = 0;
//        for(int j = 0; j < Close_Points.size();j++){
//            float Dist = sqrt((Points[Close_Points[j]][0]-Center[0])*(Points[Close_Points[j]][0]-Center[0]) + (Points[Close_Points[j]][1]-Center[1])*(Points[Close_Points[j]][1]-Center[1]));
//            if(Dist < Min_Distance){
//                Min_Distance = Dist;
//                Keep_Index = j;
//            }
//        }
//        vector<int> Keep_Point = {Points[Close_Points[Keep_Index]][0],Points[Close_Points[Keep_Index]][1]};
//        Keep_Points.push_back(Keep_Point);
//    }

    vector<vector<int>> Final_Points;
    // Remove identical points
    for(int i = 0; i < Keep_Points.size();i++){
        bool Identical = false;
        for(int j = i+1;j < Keep_Points.size(); j++){
            if(Keep_Points[i][0] == Keep_Points[j][0] && Keep_Points[i][1] == Keep_Points[j][1]){
                Identical = true;
            }
        }
        if(Identical == false){
            Final_Points.push_back(Keep_Points[i]);
        }
    }

    // If only three points add missing corner
    int Average_x = 0;
    int Average_y = 0;
    if(Final_Points.size() == 3){
        for(int i = 0; i < Final_Points.size(); i++){
            Average_x += Final_Points[i][0];
            Average_y += Final_Points[i][1];
        }
        Average_x /= Final_Points.size();
        Average_y /= Final_Points.size();

        int Max_x_Dist = 0;
        int Max_y_Dist = 0;
        int Lone_x = 0;
        int Lone_y = 0;

        for(int i = 0; i < Final_Points.size(); i++){
            int x_Dist = abs(Final_Points[i][0]-Average_x);
            int y_Dist = abs(Final_Points[i][1]-Average_y);
            if(x_Dist > Max_x_Dist){
                Max_x_Dist = x_Dist;
                Lone_x = Final_Points[i][0];
            }
            if(y_Dist > Max_y_Dist){
                Max_y_Dist = y_Dist;
                Lone_y = Final_Points[i][1];
            }
        }
        Final_Points.push_back({Lone_x,Lone_y});
        Point_Image[Lone_x][Lone_y] = 255;
    }

    // Remove points from image
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            bool Found = false;
            for(int k = 0; k < Final_Points.size();k ++){
                if(i == Final_Points[k][0] && j == Final_Points[k][1]){
                    Found = true;
                }
            }
            if(Found == false){
                Point_Image[i][j] = 0;
            }
        }
    }


    Outputs.Points = Final_Points;

    Outputs.Point_Image = Point_Image;

    return Outputs;
}

// Find angle
struct Angle_Data{
    float Angle;
    Mat Angle_Image;
};

Angle_Data Find_Angle(Mat Image, vector<int> Center, vector<vector<int>> Points, int Screw_Type){
    cout << "Finding angle" << endl;
    float Angle = 0;

    Angle_Data Outputs;
    Mat Angle_Image = Image.clone();

    // If line screw
    if(Screw_Type == 0){
        // if more than 4 points we have a problem
        if(Points.size() > 4){
            cout << "ERROR: More than 4 points" << endl;
        }

        if(Points.size() == 2){
            cout << "Only two points so solving with one line" << endl;

            // Make vertical center line
            int y_Change = 40;
            vector<vector<int>> Center_Line = {{Center},{Center[0],Center[1]-y_Change}};

            // Define inner angles;
            float Vertical_x_Change = Center_Line[1][0]-Center_Line[0][0];
            float Vertical_y_Change = Center_Line[1][1]-Center_Line[0][1];

            float Line_1_x_Change = Points[1][0]-Points[0][0];
            float Line_1_y_Change = Points[1][1]-Points[0][1];

            float Line_1_Dot = Line_1_x_Change*Vertical_x_Change + Line_1_y_Change*Vertical_y_Change;

            float Line_1_Norm = sqrt(Line_1_x_Change*Line_1_x_Change+Line_1_y_Change*Line_1_y_Change);
            float Vertical_Norm = sqrt(Vertical_x_Change*Vertical_x_Change+Vertical_y_Change*Vertical_y_Change);

            float Line_1_Angle = acos(Line_1_Dot/(Line_1_Norm*Vertical_Norm));

            float Angle_1 = Line_1_Angle*180/M_PI;

            if(abs(180-Angle_1) < Angle_1){
                Angle_1 = -(180-Angle_1);
            }
            line(Angle_Image,Point(Points[0][0],Points[0][1]),Point(Points[1][0],Points[1][1]),Scalar(255,255,255),1,LINE_4);
            line(Angle_Image,Point(Center_Line[0][0],Center_Line[0][1]),Point(Center_Line[1][0],Center_Line[1][1]),Scalar(200,200,200),1,LINE_4);
            Angle = Angle_1;
        }
        else{
            // Find points close to each other
            float Min_Dist = 100;
            int Min_Index = 0;
            vector<vector<int>> Closest_Points;
            for(int i = 1; i < Points.size(); i++){
                float Dist = sqrt((Points[i][0]-Points[0][0])*(Points[i][0]-Points[0][0])+(Points[i][1]-Points[0][1])*(Points[i][1]-Points[0][1]));
                if(Dist < Min_Dist){
                    Min_Dist = Dist;
                    Min_Index = i;
                }
            }
            Closest_Points.push_back({0,Min_Index});

            vector<int> Rest_Point;
            for(int i = 0; i < Points.size(); i++){
                if(i != Closest_Points[0][0] && i != Closest_Points[0][1]){
                    Rest_Point.push_back(i);
                }
            }
            Closest_Points.push_back(Rest_Point);

            // If more than two combinations something went wrong
            if(Closest_Points.size() > 2){
                cout << "ERROR: More than two closest lines" << endl;
            }

            line(Angle_Image,Point(Points[Closest_Points[0][0]][0],Points[Closest_Points[0][0]][1]),Point(Points[Closest_Points[0][1]][0],Points[Closest_Points[0][1]][1]),Scalar(100,100,100),1,LINE_4);
            line(Angle_Image,Point(Points[Closest_Points[1][0]][0],Points[Closest_Points[1][0]][1]),Point(Points[Closest_Points[1][1]][0],Points[Closest_Points[1][1]][1]),Scalar(100,100,100),1,LINE_4);

            // Connect points into lines
            vector<vector<int>> Lines;
            int Index_1 = Closest_Points[0][0];
            int Index_Rest = Closest_Points[0][1];
            int Index_2 = Closest_Points[1][0];
            int Index_3 = Closest_Points[1][1];

            float Dist_1 = sqrt((Points[Index_2][0]-Points[Index_1][0])*(Points[Index_2][0]-Points[Index_1][0])+(Points[Index_2][1]-Points[Index_1][1])*(Points[Index_2][1]-Points[Index_1][1]));
            float Dist_2 = sqrt((Points[Index_3][0]-Points[Index_1][0])*(Points[Index_3][0]-Points[Index_1][0])+(Points[Index_3][1]-Points[Index_1][1])*(Points[Index_3][1]-Points[Index_1][1]));
            if(Dist_2 < Dist_1){
                Lines.push_back({Points[Index_1][0],Points[Index_1][1],Points[Index_3][0],Points[Index_3][1]});
                Lines.push_back({Points[Index_Rest][0],Points[Index_Rest][0],Points[Index_2][0],Points[Index_2][1]});
            }
            else{
                Lines.push_back({Points[Index_1][0],Points[Index_1][1],Points[Index_2][0],Points[Index_2][1]});
                Lines.push_back({Points[Index_Rest][0],Points[Index_Rest][1],Points[Index_3][0],Points[Index_3][1]});
            }

            line(Angle_Image,Point(Lines[0][0],Lines[0][1]),Point(Lines[0][2],Lines[0][3]),Scalar(200,200,200),1,LINE_4);
            line(Angle_Image,Point(Lines[1][0],Lines[1][1]),Point(Lines[1][2],Lines[1][3]),Scalar(200,200,200),1,LINE_4);

            // Possibly add check for parrallel

            // Make vertical center line
            int y_Change = 40;
            vector<vector<int>> Center_Line = {{Center},{Center[0],Center[1]-y_Change}};


            // Define inner angles;
            float Vertical_x_Change = Center_Line[1][0]-Center_Line[0][0];
            float Vertical_y_Change = Center_Line[1][1]-Center_Line[0][1];

            float Line_1_x_Change = Lines[0][2]-Lines[0][0];
            float Line_1_y_Change = Lines[0][3]-Lines[0][1];
            float Line_2_x_Change = Lines[1][2]-Lines[1][0];
            float Line_2_y_Change = Lines[1][3]-Lines[1][1];

            float Line_1_Dot = Line_1_x_Change*Vertical_x_Change + Line_1_y_Change*Vertical_y_Change;
            float Line_2_Dot = Line_2_x_Change*Vertical_x_Change + Line_2_y_Change*Vertical_y_Change;

            float Line_1_Norm = sqrt(Line_1_x_Change*Line_1_x_Change+Line_1_y_Change*Line_1_y_Change);
            float Line_2_Norm = sqrt(Line_2_x_Change*Line_2_x_Change+Line_2_y_Change*Line_2_y_Change);
            float Vertical_Norm = sqrt(Vertical_x_Change*Vertical_x_Change+Vertical_y_Change*Vertical_y_Change);

            float Line_1_Angle = acos(Line_1_Dot/(Line_1_Norm*Vertical_Norm));
            float Line_2_Angle = acos(Line_2_Dot/(Line_2_Norm*Vertical_Norm));

            float Angle_1 = Line_1_Angle*180/M_PI;
            float Angle_2 = Line_2_Angle*180/M_PI;

            if(abs(180-Angle_1) < Angle_1){
                Angle_1 = -(180-Angle_1);
            }
            if(abs(180-Angle_2) < Angle_2){
                Angle_2 = -(180-Angle_2);
            }
            if(abs(Angle_2) < abs(Angle_1)){
                Angle = Angle_2;
                // Draw Line
                //line(Angle_Image,Point(Lines[1][0],Lines[1][1]),Point(Lines[1][2],Lines[1][3]),Scalar(255,255,255),1,LINE_4);
            }
            else{
                Angle = Angle_1;
                //line(Angle_Image,Point(Lines[0][0],Lines[0][1]),Point(Lines[0][2],Lines[0][3]),Scalar(255,255,255),1,LINE_4);
            }
            line(Angle_Image,Point(Center_Line[0][0],Center_Line[0][1]),Point(Center_Line[1][0],Center_Line[1][1]),Scalar(200,200,200),1,LINE_4);
        }
    }

    // If square screw
    else if(Screw_Type == 1){
        // Make vertical center line
        int y_Change = 40;
        vector<vector<int>> Center_Line = {{Center},{Center[0],Center[1]-y_Change}};
        line(Angle_Image,Point(Center[0],Center[1]),Point(Center_Line[1][0],Center_Line[1][1]),Scalar(255,255,255),1,LINE_4);
        // Define inner angles;
        float Vertical_x_Change = Center_Line[1][0]-Center_Line[0][0];
        float Vertical_y_Change = Center_Line[1][1]-Center_Line[0][1];
        float Vertical_Norm = sqrt(Vertical_x_Change*Vertical_x_Change+Vertical_y_Change*Vertical_y_Change);


        // For all lines
        vector<float> Line_Angles;
        for(int i = 0; i < Points.size();i++){
            for(int j = i+1; j < Points.size(); j++){
                line(Angle_Image,Point(Points[j][0],Points[j][1]),Point(Points[i][0],Points[i][1]),Scalar(255,255,255),1,LINE_4);
                float x_Change = Points[j][0]-Points[i][0];
                float y_Change = Points[j][1]-Points[i][1];
                float Dot = x_Change*Vertical_x_Change + y_Change*Vertical_y_Change;
                float Norm =  sqrt(x_Change*x_Change+y_Change*y_Change);
                float Line_Angle = acos(Dot/(Norm*Vertical_Norm));
                Line_Angles.push_back(Line_Angle*180/M_PI);
            }
        }
        float Min_Angle = 180;
        for(int i = 0; i < Line_Angles.size(); i++){

            // Correct since square
            if(Line_Angles[i] > 90 && Line_Angles[i] < 180){
                Line_Angles[i] = Line_Angles[i]-90;
            }
            else if(Line_Angles[i] > 180 && Line_Angles[i] < 270){
                Line_Angles[i] = -(Line_Angles[i]-180);
            }
            else if(Line_Angles[i] > 270 && Line_Angles[i]){
                Line_Angles[i] = -(Line_Angles[i]-270);
            }

            if(abs(Line_Angles[i]) == 90){
                Line_Angles[i] = 0;
            }
            if(abs(90-Line_Angles[i]) < Line_Angles[i]){
                Line_Angles[i] = -(90-Line_Angles[i]);
            }
            if(abs(Line_Angles[i]) < abs(Min_Angle)){
                Min_Angle = Line_Angles[i];
                Angle = Line_Angles[i];
            }

        }

    }
    Outputs.Angle = Angle;
    Outputs.Angle_Image = Angle_Image;
    return Outputs;
}

// Implementation functions

float FlatHead_Angle(string Image_Path){
    // Read image using openCV
    Mat Image = imread(Image_Path);

    // Initialize angle
    float Angle = 0;

    // Check if image was read correctly
    if(Image.empty()){
        cout << "ERROR 1: Image not found" << endl;
    }
    else{
        cout << "Image read correctly" << endl;

        // Get working vector
        vector<vector<vector<int>>> BGR_Vector = Mat_To_Vec_BGR(Image);

        // Grayscale image
        vector<vector<float>> Grayscale_Vector = Grayscale_Image(BGR_Vector,0.587,0.299,0.114);

        // Get grayscale image for viewing
        Mat Grayscale_Image = Vec_2D_To_Mat(Grayscale_Vector);

        // Add gaussian blur
        vector<vector<float>> Blurred_Vector = Gaussian_Blur_Image(Grayscale_Vector,5);

        // Get blured image for viewing
        Mat Blurred_Image = Vec_2D_To_Mat(Blurred_Vector);

        // Perform line detection using sobel
        Sobel_Vector_Outputs Sobel_Vectors;
        Sobel_Vectors = Sobel_Image(Blurred_Vector);

        // Get sobel images for viewing
        Mat Sobel_Magnitude_Image = Vec_2D_To_Mat(Sobel_Vectors.Sobel_Magnitude);
        Mat Sobel_Direction_Image = Vec_2D_To_Mat(Sobel_Vectors.Sobel_Direction);

        // Perform non maximum suppresion
        vector<vector<float>> Suppressed_Vector = Non_Max_Supression_Image(Sobel_Vectors.Sobel_Magnitude,Sobel_Vectors.Sobel_Direction);

        // Get suppressed image for viewing
        Mat Suppressed_Image = Vec_2D_To_Mat(Suppressed_Vector);

        // Cutoff outer pixels of image
        vector<vector<float>> Cutoff_Vector = Cutoff_Image(Suppressed_Vector,13);

        // Make cutoff image
        Mat Cut_Image = Vec_2D_To_Mat(Cutoff_Vector);

        // Perform hysteris thresholding
        vector<vector<float>> Thresholded_Vector = Double_Threholding_Image(Cutoff_Vector,35,20);

        // Get thresholded image for viewing
        Mat Threshold_Image = Vec_2D_To_Mat(Thresholded_Vector);

        // Find endpoints
        Endpoints Point_Data;
        Point_Data = Find_Endpoints(Thresholded_Vector);

        Mat Endpoint_Image = Vec_2D_To_Mat(Point_Data.Point_Image);

        // Find center point
        vector<int> Center_Point = Find_Center(Point_Data.Point_Image,150);

        // Draw point
        Mat Point_Image = Draw_Point(Endpoint_Image,Center_Point,255);

        // Find angle of line screw
        Angle_Data Angle_Result;
        Angle_Result = Find_Angle(Point_Image, Center_Point, Point_Data.Points, 0);

        cout << "Found angle: " << Angle_Result.Angle << endl;


        //Mat Resized_Result;
        //resize(Suppressed_Image,Resized_Result, Size(200,200), INTER_LINEAR);
        imshow("Line screw outline", Image);
        waitKey(0);
    }

    return Angle;
}

// Square does not work
float Square_Angle(string Image_Path){
    // Read image using openCV
    Mat Image = imread(Image_Path);

    // Initialize angle
    float Angle = 0;

    // Check if image was read correctly
    if(Image.empty()){
        cout << "ERROR 1: Image not found" << endl;
    }
    else{
        cout << "Image read correctly" << endl;

        // Get working vector
        vector<vector<vector<int>>> BGR_Vector = Mat_To_Vec_BGR(Image);

        // Grayscale image
        vector<vector<float>> Grayscale_Vector = Grayscale_Image(BGR_Vector,0.587,0.299,0.114);

        // Get grayscale image for viewing
        Mat Grayscale_Image = Vec_2D_To_Mat(Grayscale_Vector);

        // Add gaussian blur
        vector<vector<float>> Blurred_Vector = Gaussian_Blur_Image(Grayscale_Vector,4);

        // Get blured image for viewing
        Mat Blurred_Image = Vec_2D_To_Mat(Blurred_Vector);

        // Perform line detection using sobel
        Sobel_Vector_Outputs Sobel_Vectors;
        Sobel_Vectors = Sobel_Image(Blurred_Vector);

        // Get sobel images for viewing
        Mat Sobel_Magnitude_Image = Vec_2D_To_Mat(Sobel_Vectors.Sobel_Magnitude);
        Mat Sobel_Direction_Image = Vec_2D_To_Mat(Sobel_Vectors.Sobel_Direction);

        // Perform non maximum suppresion
        vector<vector<float>> Suppressed_Vector = Non_Max_Supression_Image(Sobel_Vectors.Sobel_Magnitude,Sobel_Vectors.Sobel_Direction);

        // Get suppressed image for viewing
        Mat Suppressed_Image = Vec_2D_To_Mat(Suppressed_Vector);

        // Cutoff outer pixels of image
        vector<vector<float>> Cutoff_Vector = Cutoff_Image(Suppressed_Vector,13);

        // Make cutoff image
        Mat Cut_Image = Vec_2D_To_Mat(Cutoff_Vector);

        // Perform hysteris thresholding
        vector<vector<float>> Thresholded_Vector = Double_Threholding_Image(Cutoff_Vector,20,10);

        // Get thresholded image for viewing
        Mat Threshold_Image = Vec_2D_To_Mat(Thresholded_Vector);

        // Find corners
        Corners Corner_Data;
        Corner_Data = Find_Corners(Thresholded_Vector);
        Mat Corner_Image = Vec_2D_To_Mat(Corner_Data.Point_Image);

        // Find center point
        vector<int> Center_Point = Find_Center(Corner_Data.Point_Image,150);

        // Draw point
        Mat Point_Image = Draw_Point(Corner_Image,Center_Point,255);

        // Find angle of line screw
        Angle_Data Angle_Result;
        Angle_Result = Find_Angle(Point_Image, Center_Point, Corner_Data.Points, 1);

        cout << "Found angle: " << Angle_Result.Angle << endl;


        // Canny test
        //Canny(Blurred_Image,Blurred_Image,10,25,3);

        //Mat Resized_Result;
        //resize(Suppressed_Image,Resized_Result, Size(200,200), INTER_LINEAR);
        imshow("Square screw", Grayscale_Image);
        imshow("Square2 screw", Angle_Result.Angle_Image);
        imshow("Square3 screw", Threshold_Image);
        imshow("Square4 screw", Image);
        waitKey(0);
    }

    return Angle;
}

int main(){
    //FlatHead_Angle("/home/benjamin/Flat_Screw_0.png");
    //Square_Angle("/home/benjamin/Square_40.png");
    return 0;
}
