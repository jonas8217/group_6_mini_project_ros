#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>

using namespace std;
using namespace cv;

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
            if((i == Middle_Col-Radius || i == Middle_Col+Radius || j == Middle_Row-Radius || j == Middle_Row+Radius)){
//                if(Reduced_Image.at<uchar>(Point(i,j)) != 255){
//                    Reduced_Image.at<uchar>(Point(i,j)) = 100;
//                }
            }
        }
    }

    return Reduced_Image;
}

struct Angle_Data{
    Mat Image;
    float Angle;
};

Angle_Data Flat_Head_Angle(Mat Image, int Radius, Mat Draw_Image){
    // Get image size
    int Rows = Image.rows;
    int Cols = Image.cols;

    // Outputs
    float Angle = 360;
    Mat Angle_Image = Image.clone();
    Angle_Data Outputs;

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

    // Find all lines with a distance of above 20 pixels (square has diameter of 10+10)
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
        Outputs.Angle = Angle;
        Outputs.Image = Angle_Image;
        return Outputs;
    }

    // Find line with most parallels
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

    // Remove points from image
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            if(Image.at<uchar>(Point(i,j)) == 255){
                bool Found = false;
                for(int k = 0; k < Final_Points.size(); k++){
                    if(Final_Points[k][0] == i && Final_Points[k][1] == j){
                        Found = true;
                    }
                }
                if(Found == false){
                    Angle_Image.at<uchar>(Point(i,j)) = 0;
                }
            }
        }
    }

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

    // Draw lines
    Mat Out_Image = Draw_Image.clone();
    line(Out_Image,Point(Center[0],Center[1]+20),Point(Center[0],Center[1]-20),Scalar(255,0,0),1,LINE_AA);
    line(Out_Image,Point(Final_Points[0][0],Final_Points[0][1]),Point(Final_Points[1][0],Final_Points[1][1]),Scalar(0,255,0),1,LINE_AA);



    Outputs.Angle = Angle;
    Outputs.Image = Out_Image;
    return Outputs;
}

Angle_Data Cross_Head_Angle(Mat Image, int Radius, Mat Draw_Image){
    // Get image size
    int Rows = Image.rows;
    int Cols = Image.cols;

    // Find center of image
    vector<int> Center = {Cols/2,Rows/2};

    // Outputs
    float Angle = 360;
    Mat Angle_Image = Image.clone();
    Angle_Data Outputs;

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
        Outputs.Angle = Angle;
        Outputs.Image = Angle_Image;

        return Outputs;
    }

    // Remove all points from image to focus on hough lines
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            if(Angle_Image.at<uchar>(Point(i,j)) == 255){
                Angle_Image.at<uchar>(Point(i,j)) = 0;
            }
        }
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

    // Draw image
    for(int i = 0; i < Best_Lines.size(); i++){
        float a = (float)(Best_Lines[i][3]-Best_Lines[i][1])/(Best_Lines[i][2]-Best_Lines[i][0]);
        Point p1(0,0), p2(Cols,Rows);

        p1.y = -(Best_Lines[i][0]-p1.x)*a+Best_Lines[i][1];
        p2.y = -(Best_Lines[i][2]-p2.x)*a+Best_Lines[i][3];

        line(Draw_Image,p1,p2,Scalar(0,255,0),1,LINE_AA);
    }
    line(Draw_Image,Point(Center[0],0),Point(Center[0],Rows),Scalar(255,0,0),1,LINE_AA);

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

    Outputs.Angle = Angle;
    Outputs.Image = Draw_Image;

    return Outputs;
}

Angle_Data Square_Head_Angle(Mat Image, int Radius, Mat Draw_Image){
    // Get image size
    int Rows = Image.rows;
    int Cols = Image.cols;

    // Find center of image
    vector<int> Center = {Cols/2,Rows/2};

    // Outputs
    float Angle = 360;
    Mat Angle_Image = Image.clone();
    Angle_Data Outputs;
    Mat Out_Image = Draw_Image.clone();

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
        Outputs.Angle = Angle;
        Outputs.Image = Angle_Image;

        return Outputs;
    }

    // Remove all points from image to focus on hough lines
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            if(Angle_Image.at<uchar>(Point(i,j)) == 255){
                Angle_Image.at<uchar>(Point(i,j)) = 0;
            }
        }
    }

    // Draw lines
    for(int i = 0; i < Lines.size(); i++){
        line(Angle_Image,Point(Lines[i][0],Lines[i][1]),Point(Lines[i][2],Lines[i][3]),Scalar(100,100,100),1,LINE_AA);
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

    // Draw lines
    for(int i = 0; i < New_Lines.size(); i++){
        line(Out_Image,Point(New_Lines[i][0],New_Lines[i][1]),Point(New_Lines[i][2],New_Lines[i][3]),Scalar(0,100,0),1,LINE_AA);
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

    // Draw image
    for(int i = 0; i < Best_Lines.size(); i++){
        float a = (float)(Best_Lines[i][3]-Best_Lines[i][1])/(Best_Lines[i][2]-Best_Lines[i][0]);
        Point p1(0,0), p2(Cols,Rows);

        p1.y = -(Best_Lines[i][0]-p1.x)*a+Best_Lines[i][1];
        p2.y = -(Best_Lines[i][2]-p2.x)*a+Best_Lines[i][3];

        //line(Angle_Image,p1,p2,Scalar(255,255,255),1,LINE_AA);
        line(Out_Image,Point(Best_Lines[i][0],Best_Lines[i][1]),Point(Best_Lines[i][2],Best_Lines[i][3]),Scalar(0,255,0),1,LINE_AA);
    }
    //line(Angle_Image,Point(Center[0],0),Point(Center[0],Rows),Scalar(100,100,100),1,LINE_AA);

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

    line(Out_Image,Point(Center[0],0),Point(Center[0],Rows),Scalar(255,0,0),1,LINE_AA);


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

    Outputs.Angle = Angle;
    Outputs.Image = Out_Image;

    return Outputs;
}

Angle_Data Penta_Head_Angle(Mat Image, int Radius, Mat Draw_Image){
    // Get image size
    int Rows = Image.rows;
    int Cols = Image.cols;

    // Find center of image
    vector<int> Center = {Cols/2,Rows/2};

    // Outputs
    float Angle = 360;
    Mat Angle_Image = Image.clone();
    Angle_Data Outputs;
    Mat Out_Image = Draw_Image.clone();

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
        Outputs.Angle = Angle;
        Outputs.Image = Angle_Image;

        return Outputs;
    }

    // Remove all points from image to focus on hough lines
    for(int i = 0; i < Cols; i++){
        for(int j = 0; j < Rows; j++){
            if(Angle_Image.at<uchar>(Point(i,j)) == 255){
                Angle_Image.at<uchar>(Point(i,j)) = 0;
            }
        }
    }

    // Draw lines
    for(int i = 0; i < Lines.size(); i++){
        line(Angle_Image,Point(Lines[i][0],Lines[i][1]),Point(Lines[i][2],Lines[i][3]),Scalar(100,100,100),1,LINE_AA);
    }

    // For each line rotate it 360 degrees in 72 degree interval and take the version closest to the top
    vector<Vec4i> Best_Lines;
    for(int i = 0; i < Lines.size(); i++){
        Vec4i Best_Line = Lines[i];
        int Best_y = 100;
        for(int j = 0; j < 5; j++){
            int x1 = 0;
            int y1 = 0;
            int x2 = 0;
            int y2 = 0;

            if(j == 0){
                x1 = Lines[i][0];
                y1 = Lines[i][1];
                x2 = Lines[i][2];
                y2 = Lines[i][3];
            }
            else{
                float Theta = (72*M_PI/180)*j;
                x1 = (Lines[i][0]-Center[0])*cos(Theta)-(Lines[i][1]-Center[1])*sin(Theta) + Center[0];
                y1 = (Lines[i][0]-Center[0])*sin(Theta)+(Lines[i][1]-Center[1])*cos(Theta) + Center[1];
                x2 = (Lines[i][2]-Center[0])*cos(Theta)-(Lines[i][3]-Center[1])*sin(Theta) + Center[0];
                y2 = (Lines[i][2]-Center[0])*sin(Theta)+(Lines[i][3]-Center[1])*cos(Theta) + Center[1];
            }

            // Calculate center y
            int Center_y = (y1+y2)/2;

            // If highest center y keep line
            if(Center_y < Best_y){
                Best_Line = {x1,y1,x2,y2};
                Best_y = Center_y;
            }

        }
        Best_Lines.push_back(Best_Line);
    }

    // For each kept line take the one closest to 90 degrees from the center vertical axis
    float Vertical_x_Change = (Center[0])-(Center[0]);
    float Vertical_y_Change = (Center[1]+20)-(Center[1]-20);

    float Draw_col = 0;
    float Draw_row = 0;
    float Vertical_Norm = sqrt(Vertical_x_Change*Vertical_x_Change+Vertical_y_Change*Vertical_y_Change);
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

        // If angle closer to 90 degrees than best angle, keep the new one
        if(abs(abs(Current_Angle)-90) < abs(Angle)){
            Angle = abs(abs(Current_Angle)-90);
            Out_Image = Draw_Image.clone();
            line(Out_Image,Point(Best_Lines[i][0],Best_Lines[i][1]),Point(Best_Lines[i][2],Best_Lines[i][3]),Scalar(0,255,0),1,LINE_AA);
            Draw_col = 0;
            Draw_row = Best_Lines[i][1];
        }
    }

    line(Out_Image,Point(Center[0],0),Point(Center[0],Rows),Scalar(255,0,0),1,LINE_AA);
    line(Out_Image,Point(0,Draw_row),Point(Cols,Draw_row),Scalar(255,0,0),1,LINE_AA);


    Outputs.Angle = Angle;
    Outputs.Image = Out_Image;

    return Outputs;
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
    else if(Screw_Type == 3){
        Cutoff_Radius = 18;
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
    Angle_Data Data;
    if(Screw_Type == 0){
        // Get angle
        Data = Flat_Head_Angle(Reduced_Edge_Image,Cutoff_Radius, Image);
    }
    else if(Screw_Type == 1){
        // Get angle
        Data = Cross_Head_Angle(Reduced_Edge_Image,Cutoff_Radius, Image);
    }
    else if(Screw_Type == 2){
        Data = Square_Head_Angle(Reduced_Edge_Image,Cutoff_Radius, Image);
    }
    else if(Screw_Type == 3){
        Data = Penta_Head_Angle(Reduced_Edge_Image,Cutoff_Radius, Image);
    }

    cout << "Angle: " << Data.Angle << endl;
    Angle = Data.Angle;
    imshow("Image", Image);
    imshow("Gray blurred", Blurred_Image);
    imshow("Edge", Edge_Image);
    imshow("Reduced edge", Reduced_Edge_Image);
    imshow("Result image", Data.Image);
    waitKey(0);

    return Angle;
}


int main()
{
    // Read image using openCV
    Mat Image = imread("/home/benjamin/Penta_Screw_40.png");

    float Angle = Get_Screw_Angle(Image,3);

    return 0;
}
