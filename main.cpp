//better use cstdio and cstdlib to replace stdio.h and stdlib.h in c++
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#ifdef __arm__
#include "motor.hpp"
#else
#include "motor_virt.hpp"
#endif

using namespace std;
using namespace cv;
//int speed;

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
    Mat Bund = pre_proc(cameraFrame, rows, cols,slice);

    //Masks and find contours
    cvtColor(Bund, cvt, CV_BGR2GRAY);
    GaussianBlur(cvt, blur,Size(5,5),0,0);
    threshold(blur, thres,70,255,THRESH_BINARY_INV);
    findContours(thres,contours, hierarchy,1,CHAIN_APPROX_NONE);

    //Check if no contours
    if (contours.empty()) {
        return -1;
    }

    //Find largest contour
    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        double a = contourArea(contours[i], false); //  Find the area of contour
        if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;    //Store the index of largest contour
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

int distEsti(std::vector<std::vector<cv::Point2f>> corners){
    int dist,focal=500;
    float arucosize=67.78;
    //float focal=(afstand til camera*pixel count)/diagonal længde;
    //float focal=(35*)/67.78;
    Point2f pt1, pt2;
    pt1=corners[0][0];
    pt2=corners[0][2];
    //std::cout << corners[0] << "\n";
    //std::cout << pt1.x << ", " << pt1.y << "\n";
    //std::cout << pt2.x << ", " << pt2.y << "\n";
    double pix = norm(pt1-pt2);
    dist=(arucosize*focal)/pix;
    //std::cout << dist << "mm" << "\n";
    return dist;
}

int focal(std::vector<std::vector<cv::Point2f>> corners){
    int dist, focal, fysDist=200;
    float arucosize=67.78;
    Point2f pt1, pt2;
    pt1=corners[0][0];
    pt2=corners[0][2];
    double pix = norm(pt1-pt2);
    focal=(fysDist*pix)/arucosize;
    //std::cout << focal << "\n";
    return focal;
}

void MotorFollowLine(int err, Mat mat, int rows, int cols, std::vector<int> id,int speed){
    double error = err * 0.5;
    //std::cout << error << "\n";
    if(err < 0) {
        LeftMotor(FORWARD, speed, mat, rows, cols);
        RightMotor(FORWARD, speed + int(abs(error)), mat, rows, cols);
    }
    if(err > 0) {
        RightMotor(FORWARD, speed, mat, rows, cols);
        LeftMotor(FORWARD, speed + int(abs(error)), mat, rows, cols);
    }
    if(err == 0) {
        RightMotor(FORWARD, speed, mat, rows, cols);
        LeftMotor(FORWARD, speed, mat, rows, cols);
    }
}

int CV_motor_control(VideoCapture &stream1){
    //Init/setup
    //aruco marker stuff
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    int estimate;
    string text;
    int status;

    int center_point1;
    int center_point2;

    MotorInit();
    int speed = 50;
    //Video from camera
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
        aruco::detectMarkers(cameraFrame, dictionary, corners, ids);
        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(cameraFrame, corners, ids);
            estimate=distEsti(corners);
            //estimate=focal(corners);
            text=to_string(estimate);
            putText(cameraFrame, "Dist: ", cvPoint(30,30), FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(200,200,250), 1, CV_AA);
            putText(cameraFrame, text, cvPoint(85,30), FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(200,200,250), 1, CV_AA);
        }

        if (ids.size() > 0) {
            switch (ids[0]){
                //Kill
                case 0:
                    RightMotor(BACK, 0, cameraFrame, rows, cols);
                    LeftMotor(BACK, 0, cameraFrame, rows, cols);
                    std::cout << "Case 0 - Exit program" << '\n';
                    exit(0);
                //Stop
                case 1:
                    RightMotor(FORWARD, 0, cameraFrame, rows, cols);
                    LeftMotor(FORWARD, 0, cameraFrame, rows, cols);
                    std::cout << "Case 1 - Stop motors" << '\n';
                    break;
                //Slow
                case 2:
                    speed=50;
                    std::cout << "Case 2 - Motor speed 50" << '\n';
                    break;
                //Fast
                case 3:
                    speed=100;
                    std::cout << "Case 3 - Motor speed 100" << '\n';
                    break;
                //Left
                case 4:
                    std::cout << "Case 4 - TBD" << '\n';
                    break;
                //Right
                case 5:
                    std::cout << "Case 5 - TBD" << '\n';
                    break;
                default:
                    RightMotor(FORWARD, 0, cameraFrame, rows, cols);
                    LeftMotor(FORWARD, 0, cameraFrame, rows, cols);
                    break;
            }
        }
        if (ids.size() == 0) {
          center_point1=find_point(cameraFrame, rows, cols, 7);
          center_point2=find_point(cameraFrame, rows, cols, 8);
        }

        //MotorFollowLine(center_point2, cameraFrame, rows, cols, ids, speed);
        //std::cout << center_point1 << ',';
        //std::cout << center_point2 << '\n';

        //UI, bottom half
        //rectangle( cameraFrame,Point(0,rows*0.75),Point(cols-1,rows-1),Scalar( 0, 255, 0 ),1);
        //rectangle( cameraFrame,Point(0,rows*0.875),Point(cols-1,rows-1),Scalar( 0, 255, 0 ),1);
#ifdef __x86_64
        //Show the image/frame
        namedWindow( "Frame", CV_WINDOW_AUTOSIZE );
        imshow("Frame", cameraFrame);
 #endif
        video.write(cameraFrame);
        //imshow("Threshold", thres);

        //Esc to close
        char c=(char)waitKey(1);
        if(c==27)
            break;
    }
    return 0;
}

int main()
{
    VideoCapture stream1(0);
    CV_motor_control(stream1);
    //Clean up
    stream1.release();
    destroyAllWindows();
}
