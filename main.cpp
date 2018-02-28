//better use cstdio and cstdlib to replace stdio.h and stdlib.h in c++
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    VideoCapture stream1(0);
    if(!stream1.isOpened()){
        std::cerr << "cannot open camera" << std::endl;
        return -1;
    }


    Mat cameraFrame;
    Mat cvt;
    Mat blur;
    Mat thres;
    Mat contour;

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    while (true) {
        stream1 >> cameraFrame;

        if(cameraFrame.empty()){
            std::cerr<<"the frame is empty"<<std::endl;
            break;
        }

        cvtColor(cameraFrame, cvt, CV_BGR2GRAY);
        GaussianBlur(cvt, blur,Size(9,9),2,2);
        //Canny( blur, thres, 100, 100*2, 3 );
        threshold(blur, thres,70,255,THRESH_BINARY_INV);
        findContours(thres,contours, hierarchy,1,CHAIN_APPROX_NONE);

        vector<Moments> mu(contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        { mu[i] = moments( contours[i], false ); }

        vector<Point2f> mc( contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

        for(auto i = 0; i < contours.size(); i++ )
        {
            drawContours( cameraFrame, contours, i, Scalar(178,102,255), 2, 8, hierarchy, 0, Point() );
            circle( cameraFrame, mc[i], 4, Scalar(0,255,255), -1, 8, 0 );
        }

        namedWindow( "Frame", CV_WINDOW_AUTOSIZE );
        imshow("Threshold", thres);
        imshow("Frame", cameraFrame);
        waitKey(1);
    }
    return 0;
}
