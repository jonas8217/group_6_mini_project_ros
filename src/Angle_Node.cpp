#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgcodecs.hpp>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"

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

            screw_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
                    "/screw_type",
                    10,
                    std::bind(&AnglePublisher::onScrewMsg, this, std::placeholders::_1)
            );

            Angle_Publisher_ = this->create_publisher<std_msgs::msg::Float32>(
				"/screw_angle",
				10
			);
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr screw_subscription_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr Angle_Publisher_;

        cv::Mat inp_img;
        std_msgs::msg::Float32 Angle;
        int Screw_Type = 0;

        // Cut out center square
        Mat Reduce_Image(Mat Image, int Radius, int BGR_Gray = 0){
            // Get image size
            int Rows = Image.rows;
            int Cols = Image.cols;

            // Output image
            Mat Reduced_Image = Image.clone();

            // Find middle
            int Middle_Col = Cols/2;
            int Middle_Row = Rows/2;

            // Insert all image values
            for(int i = 0; i < Cols; i++){
                for(int j = 0; j < Rows; j++){
                    // If not center colour it black
                    if(i < Middle_Col-Radius || i > Middle_Col+Radius || j < Middle_Row-Radius || j > Middle_Row+Radius){
                        if(BGR_Gray == 0){
                        Reduced_Image.at<Vec3b>(Point(i,j)) = {0,0,0};
                        }
                        else if(BGR_Gray == 1){
                            Reduced_Image.at<uchar>(Point(i,j)) = 0;
                        }
                    }
                }
            }

            return Reduced_Image;
        }

        float Flat_Head_Angle(Mat Image, int Radius){
            // Get image size
            int Rows = Image.rows;
            int Cols = Image.cols;

            // Outputs
            float Angle = 360;
            Mat Angle_Image = Image.clone();

            // Find center of image
            vector<int> Center = {Cols/2,Rows/2};

            // Keep white points on border
            vector<vector<int>> Points;
            for(int i = 0; i < Cols; i++){
                for(int j = 0; j < Rows; j++){
                    if(Image.at<uchar>(Point(i,j)) == 255 && (i == Center[0]-Radius || i == Center[0]+Radius || j == Center[1]-Radius || j == Center[1]+Radius)){
                        Points.push_back({i,j});
                    }
                    else{
                        Angle_Image.at<uchar>(Point(i,j)) = 0;
                    }
                }
            }

            // Find all lines with a distance of above 2*Radius pixels
            int Distance_Threshold = 2*Radius;
            vector<vector<int>> Lines; // x1, y1, x2, y2
            for(int i = 0; i < Points.size(); i++){
                for(int j = i+1; j < Points.size();j++){
                    float Dist = sqrt((Points[j][0]-Points[i][0])*(Points[j][0]-Points[i][0]) + (Points[j][1]-Points[i][1])*(Points[j][1]-Points[i][1]));
                    if(Dist > Distance_Threshold){
                        Lines.push_back({Points[i][0],Points[i][1],Points[j][0],Points[j][1]});
                    }
                }
            }

            if(Lines.size() < 1){
                cout << "No lines detected" << endl;
                return Angle;
            }

            // Find line with most parallels
            vector<vector<int>> New_Points;
            int Max_Parallels = 0;
            int Max_Line_Index = 0;
            for(int i = 0; i < Lines.size();i++){
                int Parallels = 0;
                for(int j = 0; j < Lines.size(); j++){
                    if(i != j){
                        float x1_Change = abs(Lines[i][2]-Lines[i][0]);
                        float y1_Change = abs(Lines[i][3]-Lines[i][1]);

                        float x2_Change = abs(Lines[j][2]-Lines[j][0]);
                        float y2_Change = abs(Lines[j][3]-Lines[j][1]);

                        float Dot = x1_Change*x2_Change + y1_Change*y2_Change;

                        float Norm1 =  sqrt(x1_Change*x1_Change+y1_Change*y1_Change);
                        float Norm2 =  sqrt(x2_Change*x2_Change+y2_Change*y2_Change);

                        float Line_Angle;
                        if(Dot/(Norm1*Norm2) > 1){
                            Line_Angle = acos(1)*180/M_PI;
                        }
                        else if(Dot/(Norm1*Norm2) < 0){
                            Line_Angle = acos(0)*180/M_PI;
                        }
                        else{
                        Line_Angle = acos(Dot/(Norm1*Norm2))*180/M_PI;
                        }

                        if(abs(Line_Angle) == 0){
                            Parallels++;
                        }
                    }
                }
                if(Parallels > Max_Parallels){
                    Max_Parallels = Parallels;
                    Max_Line_Index = i;
                }
            }

            // Get final points
            vector<vector<int>> Final_Points;
            Final_Points.push_back({Lines[Max_Line_Index][0],Lines[Max_Line_Index][1]});
            Final_Points.push_back({Lines[Max_Line_Index][2],Lines[Max_Line_Index][3]});

            // calculate angle between center line
            float Vertical_x_Change = (Center[0])-(Center[0]);
            float Vertical_y_Change = (Center[1]+20)-(Center[1]-20);

            float x_Change = abs(Final_Points[0][0]-Final_Points[1][0]);
            float y_Change = abs(Final_Points[0][1]-Final_Points[1][1]);

            float Dot = x_Change*Vertical_x_Change + y_Change*Vertical_y_Change;

            float Norm =  sqrt(x_Change*x_Change+y_Change*y_Change);
            float Vertical_Norm = sqrt(Vertical_x_Change*Vertical_x_Change+Vertical_y_Change*Vertical_y_Change);

            if(Dot/(Norm*Vertical_Norm) > 1){
                Angle = acos(1)*180/M_PI;
            }
            else if(Dot/(Norm*Vertical_Norm) < 0){
                Angle = acos(0)*180/M_PI;
            }
            else{
            Angle = acos(Dot/(Norm*Vertical_Norm))*180/M_PI;
            }

            // Detect if minus or plus
            if((Final_Points[0][0] < Final_Points[1][0] && Final_Points[0][1] < Final_Points[1][1]) || (Final_Points[1][0] < Final_Points[0][0] && Final_Points[1][1] < Final_Points[0][1])){
                Angle = -Angle;
            }

            return Angle;
        }

        float Cross_Head_Angle(Mat Image, int Radius){
            // Get image size
            int Rows = Image.rows;
            int Cols = Image.cols;

            // Find center of image
            vector<int> Center = {Cols/2,Rows/2};

            // Outputs
            float Angle = 360;
            Mat Angle_Image = Image.clone();;

            // Test probabilistic Hough transform
            vector<Vec4i> Lines;
            int Pixel_Resolution = 1;
            float Angle_Resolution = (CV_PI/180)*0.01;
            int Votes_Threshold = 10;
            int Min_Length = 4;
            int Max_Gap = 0;
            HoughLinesP(Image,Lines,Pixel_Resolution,Angle_Resolution,Votes_Threshold,Min_Length,Max_Gap);


            if(Lines.size() < 1){
                cout << "Not enough lines" << endl;
                return Angle;
            }

            // Keep lines with best 90 degree angle match
            int Closest_Dist = 360;
            int Closest_Index_1 = 0;
            int Closest_Index_2 = 0;

            for(int i = 0; i < Lines.size();i++){
                for(int j = 0; j < Lines.size(); j++){
                    if(i != j){
                        float x1_Change =   abs(Lines[i][0]-Lines[i][2]);
                        float y1_Change =   abs(Lines[i][1]-Lines[i][3]);
                        float x2_Change =   abs(Lines[j][0]-Lines[j][2]);
                        float y2_Change =   abs(Lines[j][1]-Lines[j][3]);

                        float Dot = x1_Change*x2_Change + y1_Change*y2_Change;

                        float Norm1 =  sqrt(x1_Change*x1_Change+y1_Change*y1_Change);
                        float Norm2 =  sqrt(x2_Change*x2_Change+y2_Change*y2_Change);

                        float Cosine = Dot/Norm1/Norm2;

                        float Line_Angle = acos(Cosine)*180/M_PI;

                        if(abs(Line_Angle-90) < Closest_Dist){
                            Closest_Dist = abs(Line_Angle-90);
                            Closest_Index_1 = i;
                            Closest_Index_2 = j;
                        }
                    }
                }
            }

            vector<Vec4i> Best_Lines;
            Best_Lines.push_back(Lines[Closest_Index_1]);
            Best_Lines.push_back(Lines[Closest_Index_2]);

            // Calculate angles to center line

            float Vertical_x_Change = (Center[0])-(Center[0]);
            float Vertical_y_Change = (Center[1]+20)-(Center[1]-20);

            float Vertical_Norm = sqrt(Vertical_x_Change*Vertical_x_Change+Vertical_y_Change*Vertical_y_Change);
            vector<float> Angles;
            for(int i = 0; i < Best_Lines.size(); i++){
                float x_Change = abs(Best_Lines[i][0]-Best_Lines[i][2]);
                float y_Change = abs(Best_Lines[i][1]-Best_Lines[i][3]);
                float Dot = x_Change*Vertical_x_Change + y_Change*Vertical_y_Change;
                float Norm =  sqrt(x_Change*x_Change+y_Change*y_Change);

                float Current_Angle = 360;
                if(Dot/(Norm*Vertical_Norm) > 1){
                    Current_Angle = acos(1)*180/M_PI;
                }
                else if(Dot/(Norm*Vertical_Norm) < 0){
                    Current_Angle = acos(0)*180/M_PI;
                }
                else{
                Current_Angle = acos(Dot/(Norm*Vertical_Norm))*180/M_PI;
                }
                // Detect if minus or plus
                if((Best_Lines[i][0] < Best_Lines[i][2] && Best_Lines[i][1] < Best_Lines[i][3]) || (Best_Lines[i][2] < Best_Lines[i][0] && Best_Lines[i][3] < Best_Lines[i][1])){
                    Current_Angle = -Current_Angle;
                }
                Angles.push_back(Current_Angle);
            }

            // find average absolute angle
            float Average_Angle = 0;
            for(int i = 0; i < Angles.size(); i++){
                Average_Angle += abs(Angles[i]);
            }
            Average_Angle /= Angles.size();

            float Angle_Diff;
            if(Angles[0] >= 0){
                Angle_Diff  = Angles[0]-Average_Angle;
            }
            else{
                Angle_Diff  = Angles[0]+Average_Angle;
            }

            if(abs(Angle_Diff-45) < abs(Angle_Diff+45)){
                Angle = Angle_Diff-45;
            }
            else{
                Angle = Angle_Diff+45;
            }

            return Angle;
        }

        float Square_Head_Angle(Mat Image, int Radius){
            // Get image size
            int Rows = Image.rows;
            int Cols = Image.cols;

            // Find center of image
            vector<int> Center = {Cols/2,Rows/2};

            // Outputs
            float Angle = 360;
            Mat Angle_Image = Image.clone();

            // Test probabilistic Hough transform
            vector<Vec4i> Lines;
            int Pixel_Resolution = 1;
            float Angle_Resolution = (CV_PI/180)*0.01;
            int Votes_Threshold = 15;
            int Min_Length = 5;
            int Max_Gap = 5;
            HoughLinesP(Image,Lines,Pixel_Resolution,Angle_Resolution,Votes_Threshold,Min_Length,Max_Gap);


            if(Lines.size() < 1){
                cout << "Not enough lines" << endl;
                return Angle;
            }

            // Combine all lines that are close to parrallel
            vector<vector<Vec4i>> Bucketed_Vectors;
            float Parrallel_Threshold = 5.0;

            for(int i = 0; i < Lines.size();i++){
                vector<Vec4i> Bucket = {Lines[i]};
                bool Already_Taken = false;
                // Check if i already in bucket dont do anything
                for(int k = 0; k < Bucketed_Vectors.size(); k++){
                    for(int l = 0; l < Bucketed_Vectors[k].size(); l++){
                        if(Lines[i] == Bucketed_Vectors[k][l]){
                            Already_Taken = true;
                        }
                    }
                }
                if(Already_Taken == false){
                    for(int j = 0; j < Lines.size(); j++){
                        if(i != j){
                            float x1_Change =   abs(Lines[i][0]-Lines[i][2]);
                            float y1_Change =   abs(Lines[i][1]-Lines[i][3]);
                            float x2_Change =   abs(Lines[j][0]-Lines[j][2]);
                            float y2_Change =   abs(Lines[j][1]-Lines[j][3]);

                            float Dot = x1_Change*x2_Change + y1_Change*y2_Change;

                            float Norm1 =  sqrt(x1_Change*x1_Change+y1_Change*y1_Change);
                            float Norm2 =  sqrt(x2_Change*x2_Change+y2_Change*y2_Change);

                            float Cosine = Dot/Norm1/Norm2;

                            float Line_Angle = 360;
                            if(Dot/(Norm1*Norm2) > 1){
                                Line_Angle = acos(1)*180/M_PI;
                            }
                            else if(Dot/(Norm1*Norm2) < 0){
                                Line_Angle = acos(0)*180/M_PI;
                            }
                            else{
                                Line_Angle = acos(Cosine)*180/M_PI;
                            }

                            if(abs(Line_Angle) < Parrallel_Threshold){
                                Bucket.push_back(Lines[j]);
                            }
                        }
                    }
                    Bucketed_Vectors.push_back(Bucket);
                }
            }

            // Make average vectors
            vector<Vec4i> New_Lines;
            for(int i = 0; i < Bucketed_Vectors.size(); i++){
                int Bucket_x1 = 0;
                int Bucket_y1 = 0;
                int Bucket_x2 = 0;
                int Bucket_y2 = 0;
                for(int j = 0; j < Bucketed_Vectors[i].size(); j++){
                    Bucket_x1 += Bucketed_Vectors[i][j][0];
                    Bucket_y1 += Bucketed_Vectors[i][j][1];
                    Bucket_x2 += Bucketed_Vectors[i][j][2];
                    Bucket_y2 += Bucketed_Vectors[i][j][3];
                }
                Bucket_x1 /= Bucketed_Vectors[i].size();
                Bucket_y1 /= Bucketed_Vectors[i].size();
                Bucket_x2 /= Bucketed_Vectors[i].size();
                Bucket_y2 /= Bucketed_Vectors[i].size();
                New_Lines.push_back({Bucket_x1,Bucket_y1,Bucket_x2,Bucket_y2});
            }

            // Keep lines with best 90 degree angle match
            int Closest_Dist = 360;
            int Closest_Index_1 = 0;
            int Closest_Index_2 = 0;

            for(int i = 0; i < New_Lines.size();i++){
                for(int j = 0; j < New_Lines.size(); j++){
                    if(i != j){
                        float x1_Change =   abs(New_Lines[i][0]-New_Lines[i][2]);
                        float y1_Change =   abs(New_Lines[i][1]-New_Lines[i][3]);
                        float x2_Change =   abs(New_Lines[j][0]-New_Lines[j][2]);
                        float y2_Change =   abs(New_Lines[j][1]-New_Lines[j][3]);

                        float Dot = x1_Change*x2_Change + y1_Change*y2_Change;

                        float Norm1 =  sqrt(x1_Change*x1_Change+y1_Change*y1_Change);
                        float Norm2 =  sqrt(x2_Change*x2_Change+y2_Change*y2_Change);

                        float Cosine = Dot/Norm1/Norm2;

                        float Line_Angle = 360;
                        if(Dot/(Norm1*Norm2) > 1){
                            Line_Angle = acos(1)*180/M_PI;
                        }
                        else if(Dot/(Norm1*Norm2) < 0){
                            Line_Angle = acos(0)*180/M_PI;
                        }
                        else{
                            Line_Angle = acos(Cosine)*180/M_PI;
                        }

                        if(abs(Line_Angle-90) < Closest_Dist){
                            Closest_Dist = abs(Line_Angle-90);
                            Closest_Index_1 = i;
                            Closest_Index_2 = j;
                        }
                    }
                }
            }

            vector<Vec4i> Best_Lines;
            Best_Lines.push_back(New_Lines[Closest_Index_1]);
            Best_Lines.push_back(New_Lines[Closest_Index_2]);

            // Calculate angles to center line

            float Vertical_x_Change = (Center[0])-(Center[0]);
            float Vertical_y_Change = (Center[1]+20)-(Center[1]-20);

            float Vertical_Norm = sqrt(Vertical_x_Change*Vertical_x_Change+Vertical_y_Change*Vertical_y_Change);
            vector<float> Angles;
            for(int i = 0; i < Best_Lines.size(); i++){
                float x_Change = abs(Best_Lines[i][0]-Best_Lines[i][2]);
                float y_Change = abs(Best_Lines[i][1]-Best_Lines[i][3]);
                float Dot = x_Change*Vertical_x_Change + y_Change*Vertical_y_Change;
                float Norm =  sqrt(x_Change*x_Change+y_Change*y_Change);

                float Current_Angle = 360;
                if(Dot/(Norm*Vertical_Norm) > 1){
                    Current_Angle = acos(1)*180/M_PI;
                }
                else if(Dot/(Norm*Vertical_Norm) < 0){
                    Current_Angle = acos(0)*180/M_PI;
                }
                else{
                Current_Angle = acos(Dot/(Norm*Vertical_Norm))*180/M_PI;
                }
                // Detect if minus or plus
                if((Best_Lines[i][0] < Best_Lines[i][2] && Best_Lines[i][1] < Best_Lines[i][3]) || (Best_Lines[i][2] < Best_Lines[i][0] && Best_Lines[i][3] < Best_Lines[i][1])){
                    Current_Angle = -Current_Angle;
                }
                Angles.push_back(Current_Angle);
            }

            // find average absolute angle
            float Average_Angle = 0;
            for(int i = 0; i < Angles.size(); i++){
                Average_Angle += abs(Angles[i]);
            }
            Average_Angle /= Angles.size();

            float Angle_Diff;
            if(Angles[0] >= 0){
                Angle_Diff  = Angles[0]-Average_Angle;
            }
            else{
                Angle_Diff  = Angles[0]+Average_Angle;
            }

            if(abs(Angle_Diff-45) < abs(Angle_Diff+45)){
                Angle = Angle_Diff-45;
            }
            else{
                Angle = Angle_Diff+45;
            }

            return Angle;
        }

        float Get_Screw_Angle(Mat Image, int Screw_Type){
            // Variables
            int Cutoff_Radius = 10;
            int Gaussian_Kernel_Size = 3;
            int SigmaX = 0; // Standard diviation in x direction for kernel
            int Lower_Canny = 20; // Thresholds for hysteresis
            int Upper_Canny = 50;

                // Cross screw needs different thresholds
            if(Screw_Type == 1){
                Lower_Canny = 20;
                Upper_Canny = 50;
                Cutoff_Radius = 12;
            }
            else if(Screw_Type == 2){
                Cutoff_Radius = 14;
            }

            // Output initialization
            float Angle = 360;

            // Grayscale
            Mat Grayscale_Image;
            cvtColor(Image,Grayscale_Image,COLOR_BGR2GRAY);

            // Add Gaussian blur
            Mat Blurred_Image;
            GaussianBlur(Grayscale_Image, Blurred_Image,Size(Gaussian_Kernel_Size,Gaussian_Kernel_Size),SigmaX);

            // Perform Canny edge detection
            Mat Edge_Image;
            Canny(Blurred_Image, Edge_Image, Lower_Canny, Upper_Canny);

            // Cut out everything but center of circle
            Mat Reduced_Edge_Image = Reduce_Image(Edge_Image,Cutoff_Radius,1);

            // Use screw specific methods
            if(Screw_Type == 0){
                Angle = Flat_Head_Angle(Reduced_Edge_Image,Cutoff_Radius);
            }
            else if(Screw_Type == 1){
                // Get angle
                Angle = Cross_Head_Angle(Reduced_Edge_Image,Cutoff_Radius);
            }
            else if(Screw_Type == 2){
                Angle = Square_Head_Angle(Reduced_Edge_Image,Cutoff_Radius);
            }

            cout << "Angle: " << Angle << endl;

            return Angle;
        }

        void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received image!");

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            inp_img = cv_ptr->image;

            //std::cout << inp_img.type() << std::endl;
            Angle.data = Get_Screw_Angle(inp_img,Screw_Type);
            RCLCPP_INFO(this->get_logger(), "Angle found now publishing");
            Angle_Publisher_->publish(Angle);
            RCLCPP_INFO(this->get_logger(), "Angle published");
        }

        void onScrewMsg(const std_msgs::msg::Int16::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received screw type");
            Screw_Type = msg->data;
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
