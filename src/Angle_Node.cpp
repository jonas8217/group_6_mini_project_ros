#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgcodecs.hpp>
#include "std_msgs/msg/float32.hpp"

using namespace std;
using namespace cv;

#define UIO_INVERT_Nsjjk 0L

#define UIO_DMA_N 1

#define XST_FAILURE		1L	//This is nice to have :)

#define DEVICE_FILENAME "/dev/reservedmemLKM"
#define IMAGE_WIDTH		320
#define IMAGE_HEIGHT	240
#define LENGTH IMAGE_WIDTH*IMAGE_HEIGHT*4 //(320*240*4) // Number of bytes (rgb + grayscale)
#define LENGTH_INPUT 	LENGTH*3/4 // Number of bytes for input (3/4 because rgb)
#define LENGTH_OUTPUT	LENGTH/4 // Number of bytes for output (1/4 because grayscale)

#define P_START 0x70000000
#define TX_OFFSET 0
#define RX_OFFSET_BYTES LENGTH_INPUT
#define RX_OFFSET_32 RX_OFFSET_BYTES/4 // This needs to be a whole number, otherwise input in ram is overwritten!

//Reserved_Mem pmem;
//AXIDMAController dma(UIO_DMA_N, 0x10000);
//XInvert invertIP;

uint8_t *inp_buff;
uint8_t *out_buff;

// Ros node stuff
class AnglePublisher : public rclcpp::Node
{
    public:
        AnglePublisher() : Node("Angle_Publisher") {
            RCLCPP_INFO(this->get_logger(), "Initializing Angle publisher node");

            RCLCPP_INFO(this->get_logger(), "Starting camera subscription");

            camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                    "/image_raw",
                    10,
                    std::bind(&AnglePublisher::onImageMsg, this, std::placeholders::_1)
            );

            Angle_Publisher_ = this->create_publisher<std_msgs::msg::Float32>(
				"/Angle_processed",
				10
			);
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr Angle_Publisher_;

        cv::Mat inp_img;
        std_msgs::msg::Float32 Angle;

        // Angle Detection methods

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
        Sobel_Vector_Outputs Sobel_Vector_Image(vector<vector<float>> Blurred_Grayscale_Vector_Image, int Sobel_Threshold = 45){
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
        //            if(Normalized_Magnitude < Sobel_Threshold){
        //                Normalized_Magnitude = 0;
        //            }
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
                        // Homemade remove circle filter
                        if(Sobel_Magnitudes[i][j] > 120){
                            Circle_Col_Indexes.push_back(i);
                            Circle_Row_Indexes.push_back(j);
                            Row_Vector.push_back(0);
                        }
                        else{
                            Row_Vector.push_back(Sobel_Magnitudes[i][j]);
                        }
                    }
                    else{
                        Row_Vector.push_back(0);
                    }
                }
                Suppressed_Vector_Image.push_back(Row_Vector);
            }

            // Calculate center point
            int Col_Center = 0;
            for(int i = 0; i < Circle_Col_Indexes.size();i++){
                Col_Center += Circle_Col_Indexes[i];
            }
            Col_Center /= Circle_Col_Indexes.size();

            int Row_Center = 0;
            for(int i = 0; i < Circle_Row_Indexes.size();i++){
                Row_Center += Circle_Row_Indexes[i];
            }
            Row_Center /= Circle_Row_Indexes.size();

            int Circle_Threshold = 50;
            int Col_Min = Col_Center-Circle_Threshold;
            int Col_Max = Col_Center+Circle_Threshold;
            int Row_Min = Row_Center-Circle_Threshold;
            int Row_Max = Row_Center+Circle_Threshold;

            // Remove everything outside of circle
            for(int i = 0; i < Cols; i++){
                for(int j = 0; j < Rows; j++){
                    if(i < Col_Min || i > Col_Max || j < Row_Min || j > Row_Max){
                        Suppressed_Vector_Image[i][j] = 0;
                    }

                }
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

        // Find Points
        vector<vector<float>> Find_Points(vector<vector<float>> Vector_Image){
            cout << "Finding points" << endl;

            // Get sizes
            int Cols = Vector_Image.size();
            int Rows = Vector_Image[0].size();

            vector<int> Line_x_Values;
            vector<int> Line_y_Values;

            // Find all x and y values
            for(int i = 0; i < Cols; i++){
                for(int j = 0; j < Rows; j++){
                    // If pixel is white keep its coordinates
                    if(Vector_Image[i][j] == 255){
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

            // Find max distance points
            vector<vector<int>> Max_Points;
            int Point_Remaining = 4;
            while(Point_Remaining > 0){
                int Max_Dist = 0;
                int Max_x = 0;
                int Max_y = 0;

                for(int i = 0; i < Line_x_Values.size(); i++){
                    for(int j = 0; j < Line_y_Values.size(); j++){
                        float Current_Distance = sqrt((Center_x-Line_x_Values[i])*(Center_x-Line_x_Values[i]) + (Center_y-Line_y_Values[i])*(Center_y-Line_y_Values[i]));
                        bool Winner = true;
                        for(int k = 0; k < Max_Points.size(); k++){
                            float Distance = sqrt((Max_Points[k][0]-Line_x_Values[i])*(Max_Points[k][0]-Line_x_Values[i]) + (Max_Points[k][1]-Line_y_Values[i])*(Max_Points[k][1]-Line_y_Values[i]));

                            if(Distance < Max_Points[k][2]-1){
                                Winner = false;
                            }

                        }
                        if(Current_Distance > Max_Dist && Winner == true){
                            Max_Dist = Current_Distance;
                            Max_x = Line_x_Values[i];
                            Max_y = Line_y_Values[i];
                        }
                    }
                }
                Point_Remaining--;
                Max_Points.push_back({Max_x,Max_y,Max_Dist});
            }

            // Make new image
            vector<vector<float>> Return_Image = Vector_Image;
            for(int i = 0; i < Cols; i++){
                for(int j = 0; j < Rows; j++){
                    for(int k = 0; k < Max_Points.size();k++){
                        if(i == Max_Points[k][0] && j == Max_Points[k][1]){
                            Return_Image[i][j] = 255;
                            break;
                        }
                        else{
                            Return_Image[i][j] = 0;
                        }
                    }
                }
            }

            return Return_Image;
        }

        // Make cross
        struct Cross{
            vector<Vec4i> Lines;
            Mat Image;
        };

        Cross Make_Cross(Mat Point_Image){
            cout << "Making cross" << endl;

            int Cols = Point_Image.cols;
            int Rows = Point_Image.rows;

            vector<Vec4i> Lines;

            Cross Outputs;

            Mat Output_Image = Point_Image.clone();

            vector<vector<int>> Points;

            for(int i = 0; i < Cols; i++){
                for(int j = 0; j < Rows; j++){
                    if(Point_Image.at<uchar>(Point(i,j)) == 255){
                        Points.push_back({i,j});
                    }
                }
            }

            // Find max distance to point 0
            float Max_Dist = 0;
            int Match = 0;
            for(int j = 1; j < Points.size();j++){
                float Current_Distance = sqrt((Points[0][0]-Points[j][0])*(Points[0][0]-Points[j][0]) + (Points[0][1]-Points[j][1])*(Points[0][1]-Points[j][1]));
                if(Current_Distance > Max_Dist){
                    Max_Dist = Current_Distance;
                    Match = j;
                }
            }
            // Draw line
            Lines.push_back({Points[0][0],Points[0][1],Points[Match][0],Points[Match][1]});
            line(Output_Image,Point(Points[0][0],Points[0][1]),Point(Points[Match][0],Points[Match][1]),Scalar(255,255,255),2,LINE_4);

            vector<vector<int>> New_Points;
            for(int i = 0; i < Points.size(); i++){
                if(i != Match && i != 0){
                    New_Points.push_back(Points[i]);
                }
            }

            // Draw line
            Lines.push_back({New_Points[0][0],New_Points[0][1],New_Points[1][0],New_Points[1][1]});
            line(Output_Image,Point(New_Points[0][0],New_Points[0][1]),Point(New_Points[1][0],New_Points[1][1]),Scalar(255,255,255),2,LINE_4);

            Outputs.Image = Output_Image;
            Outputs.Lines = Lines;

            return Outputs;
        }

        // Calculate angle
        struct Angle_Outputs{
            float Angle;
            Mat Image;
        };
        Angle_Outputs Calculate_Angle_Cross(vector<Vec4i> Lines, Mat Image){
            cout << "Calculating angle" << endl;
            Angle_Outputs Outputs;
            float Output_Angle = 0;
            Mat Output_Image = Image.clone();

            // Find intersection point between lines
            if(Lines.size() < 2){
                cout << "Not enough lines to determine angle" << endl;
            }
            else{
                vector<vector<float>> Line_variables;

                for(int i = 0; i < Lines.size(); i++){
                    float a = Lines[i][3]-Lines[i][1]; // y2-y1
                    float b = Lines[i][0]-Lines[i][2]; // x1-x2
                    float c = a*Lines[i][0]+b*Lines[i][1];
                    vector<float> Variables = {a,b,c};
                    Line_variables.push_back(Variables);
                }
                if(Line_variables.size() > 2){
                    cout << "More than two lines." << endl;
                }
                float Determinant = Line_variables[0][0]*Line_variables[1][1] - Line_variables[1][0]*Line_variables[0][1];

                if(Determinant == 0){
                    cout << "ERROR: No intersection" << endl;
                }
                else{
                    float x = (Line_variables[1][1]*Line_variables[0][2]-Line_variables[0][1]*Line_variables[1][2])/Determinant;
                    float y = (Line_variables[0][0]*Line_variables[1][2]-Line_variables[1][0]*Line_variables[0][2])/Determinant;


                    // Make vertical line from this point
                    float y_Change = 20;
                    float x2 = x;
                    float y2 = y - y_Change;

                    // Define inner angles;
                    float Vertical_x_Change = x2-x;
                    float Vertical_y_Change = y2-y;
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

                    if(abs(Angle_1) < abs(Angle_2)){
                        Output_Angle = Angle_1;
                        cout << "Dismissed angle: " << Angle_2 << endl;
                        line(Output_Image,Point(Lines[0][0],Lines[0][1]),Point(Lines[0][2],Lines[0][3]),Scalar(255,255,255),3,LINE_4);
                    }
                    else{
                        line(Output_Image,Point(Lines[1][0],Lines[1][1]),Point(Lines[1][2],Lines[1][3]),Scalar(255,255,255),3,LINE_4);
                        Output_Angle = Angle_2;
                        cout << "Dismissed angle: " << Angle_1 << endl;
                    }

                    // Draw image
                    line(Output_Image,Point(x,y),Point(x2,y2),Scalar(0,0,0),3,LINE_4);
                    circle(Output_Image,Point(x,y),0,Scalar(255,255,255), 4,8,0);

                }
            }

            Outputs.Angle = Output_Angle;
            Outputs.Image = Output_Image;

            return Outputs;
        }

        // Find angle
        float Find_Angle(Mat Image){
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
                vector<vector<float>> Blurred_Vector = Gaussian_Blur_Vector_Image(Grayscale_Vector,4);

                // Get blured image for viewing
                Mat Blurred_Image = Vec_2D_To_Mat(Blurred_Vector);

                // Perform line detection using sobel
                Sobel_Vector_Outputs Sobel_Vectors;

                Sobel_Vectors = Sobel_Vector_Image(Blurred_Vector);
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
                vector<vector<float>> Thresholded_Vector = Double_Threholding_Vector_Image(Suppressed_Vector,60,42);

                // Get thresholded image for viewing
                Mat Threshold_Image = Vec_2D_To_Mat(Thresholded_Vector);

                // Find Points
                vector<vector<float>> Cross_Vector = Find_Points(Thresholded_Vector);
                Mat Point_Image = Vec_2D_To_Mat(Cross_Vector);

                // Make cross
                Cross Cross_Data;
                Cross_Data = Make_Cross(Point_Image);

                // Get angle
                Angle_Outputs Angle_Stuff;
                Angle_Stuff = Calculate_Angle_Cross(Cross_Data.Lines,Blurred_Image);

                return Angle_Stuff.Angle;
            }
            return 0;
        }

        void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received image!");

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            inp_img = cv_ptr->image;

            std::cout << inp_img.type() << std::endl;
            Angle.data = Find_Angle(inp_img);
            RCLCPP_INFO(this->get_logger(), "Angle found now publishing");
            Angle_Publisher_->publish(Angle);
            RCLCPP_INFO(this->get_logger(), "Angle published");
        }
};

int main(int argc, char *argv[])
{
    inp_buff = (uint8_t *)malloc(LENGTH_INPUT);
    if (inp_buff == NULL)
    {
        printf("could not allocate user buffer\n");
        return -1;
    }
    out_buff = (uint8_t *)malloc(LENGTH_OUTPUT);
    if (out_buff == NULL)
    {
        printf("could not allocate user buffer\n");
        return -1;
    }

    /*
    int Status;
    Status = XInvert_Initialize(&invertIP, "Invert");

    if (Status != XST_SUCCESS) {
        printf("Invert initialization failed %d\r\n", Status);
        return XST_FAILURE;
    }
    */
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<AnglePublisher>());

    rclcpp::shutdown();
    return 0;
}
