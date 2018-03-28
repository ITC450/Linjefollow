//better use cstdio and cstdlib to replace stdio.h and stdlib.h in c++
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <time.h>

#include <opencv2/opencv.hpp>

#ifdef __arm__
  #include "motor.hpp"
#else
  #include "motor_virt.hpp"
#endif

using namespace std;
using namespace cv;
int speed;

//rows and cols return the height and width of the stream in pixels
int mat_rows(Mat mat){
    int rows = mat.rows;
    Size s = mat.size();
    rows = s.height;
    cout << rows << '\n';
    return rows;
}
int mat_cols(Mat mat){
    int cols = mat.cols;
    Size s = mat.size();
    cols = s.width;
    cout << cols << '\n';
    return cols;
}

//Prepprocessing of the stream, returns a sub Mat of the original stream Mat
Mat pre_proc(Mat mat, int y_akse, int x_akse, int slice){
    Rect firkant = Rect(0,y_akse*(0.125*(slice-1)),x_akse,y_akse/8);
    Mat Bund = mat(firkant);
    return Bund;
}

//Finds the center of mass of a Mat and draws it on a given Mat
int find_point(Mat cameraFrame,int rows,int cols, int slice){
    int afvigelse;
    //Mats and containers
    Mat cvt;
    Mat blur;
    Mat thres;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int largest_area=0;
    int largest_contour_index=0;

    //Top slice
    Mat Bund = pre_proc(cameraFrame, rows, cols ,slice);

    //Masks and find contours
    GaussianBlur(Bund, blur,Size(5,5),0,0);
    threshold(blur, thres,70,255,THRESH_BINARY_INV);
    findContours(thres,contours, hierarchy,1,CHAIN_APPROX_NONE);

    //Check if no contours
    if (contours.empty()){
        return -1;
    }

    //Find largest contour
    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        double a = contourArea(contours[i], false);  //  Find the area of contour
        if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;                //Store the index of largest contour
        }
    }

    //Find center of mass(area)
    Moments mu;
    mu = moments( contours[largest_contour_index], false );

    Point2f mc;
    mc = Point2f( mu.m10/mu.m00, mu.m01/mu.m00 );

    afvigelse = mc.x-(cols/2);

    //Draw the center and contour outline
    drawContours( Bund, contours, largest_contour_index, Scalar(255,255,255), 1, 4, hierarchy, 0, Point() );
    circle( Bund, mc, 4, Scalar(255,255,255), 1, 8, 0 );

    return afvigelse;
}

void MotorFollowLine(int err, Mat mat, int rows, int cols){
    double error = err * 0.5;
    //std::cout << error << "\n";

    if(err < 0){
        LeftMotor(FORWARD, speed, mat, rows, cols);
        RightMotor(FORWARD, speed + int(abs(error))  , mat, rows, cols);
    }
    if(err > 0){
        RightMotor(FORWARD, speed, mat, rows, cols);
        LeftMotor(FORWARD, speed + int(abs(error))  , mat, rows, cols);
    }
    if(err == 0){
        RightMotor(FORWARD, speed, mat, rows, cols);
        LeftMotor(FORWARD, speed, mat, rows, cols);
    }
}

int CV_motor_control(){
    //Init/setup
    MotorInit();
    speed = 50;
    //Video from camera
    VideoCapture stream1(0);
    if(!stream1.isOpened()) {
        std::cerr << "cannot open camera" << std::endl;
        return -1;
    }
    //.set is for controlling size of stream and the stream mat
    //stream1.set(CV_CAP_PROP_FRAME_WIDTH,2000);
    //stream1.set(CV_CAP_PROP_FRAME_HEIGHT,1500);

    //Setup mat for source frame and insert feed into mat
    Mat cameraFrame;
    stream1 >> cameraFrame;
    cvtColor(cameraFrame, cameraFrame, CV_BGR2GRAY);
    //Gets the resolution of the feed
    cout << "Resolution: " << '\n';
    int rows=mat_rows(cameraFrame);
    int cols=mat_cols(cameraFrame);
    //Save as  settings
    VideoWriter video("linefollower.avi",CV_FOURCC('M','J','P','G'),30, Size(cols,rows));

    while (true) {
        //Insert feed into frame mat
        stream1 >> cameraFrame;
        //Check if feed has stopped
        if(cameraFrame.empty()) {
            std::cerr<<"the frame is empty"<<std::endl;
            break;
        }

        cvtColor(cameraFrame, cameraFrame, CV_BGR2GRAY);

        //Find to center points in the lower half of the frame
        int center_point1=find_point(cameraFrame, rows, cols, 7);
        int center_point2=find_point(cameraFrame, rows, cols, 8);

        cvtColor(cameraFrame, cameraFrame, CV_GRAY2BGR);

        MotorFollowLine(center_point2, cameraFrame, rows, cols);
        //std::cout << center_point1 << ',';
        //std::cout << center_point2 << '\n';

        //UI, bottom half
        rectangle( cameraFrame,Point(0,rows*0.75),Point(cols-1,rows-1),Scalar( 0, 255, 0 ),1);
        rectangle( cameraFrame,Point(0,rows*0.875),Point(cols-1,rows-1),Scalar( 0, 255, 0 ),1);

        //Show the image/frame
        namedWindow( "Frame", CV_WINDOW_AUTOSIZE );
        imshow("Frame", cameraFrame);
        video.write(cameraFrame);
        //imshow("Threshold", thres);

        //Esc to close
        char c=(char)waitKey(1);
        if(c==27)
            break;
    }
    //Clean up
    stream1.release();
    destroyAllWindows();
    return 0;
}

int main()
{
    CV_motor_control();
}
