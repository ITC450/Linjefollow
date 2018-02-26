//better use cstdio and cstdlib to replace stdio.h and stdlib.h in c++
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int main()
{
    cv::VideoCapture stream1(0);
    if(!stream1.isOpened()){
        std::cerr << "cannot open camera" << std::endl;
        return -1;
    }

    //declare the variables outside of the loop
    //to decrease the number of memory allocation
    cv::Mat cannyImage;
    cv::Mat cameraFrame;
    std::vector<cv::Vec4i> lines;
    while (true) {
        stream1 >> cameraFrame;

        if(cameraFrame.empty()){
            std::cerr<<"the frame is empty"<<std::endl;
            break;
        }

        //imshow("Source Input", cameraFrame);
        cvtColor(cameraFrame, cannyImage, CV_BGR2GRAY);
        Canny( cannyImage, cannyImage, 100, 200, 3 );
        //imshow("Canny", cannyImage);

        HoughLinesP(cannyImage, lines, 1, CV_PI/180, 75, 20, 20);

        for( size_t i = 0; i < lines.size(); i++ )
            {
              Vec4i l = lines[i];
              line(cameraFrame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(178,102,255), 3, LINE_AA);
            }
        imshow("Source Input", cameraFrame);


        if (waitKey(30) >= 0)
            break;

    }
    return 0;
}
