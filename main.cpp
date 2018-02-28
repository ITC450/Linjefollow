//better use cstdio and cstdlib to replace stdio.h and stdlib.h in c++
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int mat_rows(Mat mat){
    int rows = mat.rows;
    Size s = mat.size();
    rows = s.height;
    //std::cout << rows << '\n';
    return rows;
}
int mat_cols(Mat mat){
    int cols = mat.cols;
    Size s = mat.size();
    cols = s.width;
    //std::cout << cols << '\n';
    return cols;
}

Mat pre_proc(Mat mat, int y_akse, int x_akse){
    Rect firkant = Rect(0,y_akse/2,x_akse,y_akse/4);
    Mat Bund = mat(firkant);
    return Bund;
}
Mat pre_proc2(Mat mat, int y_akse, int x_akse){
    Rect firkant = Rect(0,y_akse*(0.75),x_akse,y_akse/4-1);
    Mat Bund = mat(firkant);
    return Bund;
}

void find_line(Mat cameraFrame){
    int rows=mat_rows(cameraFrame);
    int cols=mat_cols(cameraFrame);

    //Top slice
    Mat cvt;
    Mat blur;
    Mat thres;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    //Bottom slice
    Mat cvt2;
    Mat blur2;
    Mat thres2;
    vector<vector<Point> > contours2;
    vector<Vec4i> hierarchy2;

    //Top slice
    Mat Bund = pre_proc(cameraFrame, rows, cols);
    int largest_area=0;
    int largest_contour_index=0;
    //Bottom slice
    Mat Bund2 = pre_proc2(cameraFrame, rows, cols);
    int largest_area2=0;
    int largest_contour_index2=0;

    //Top slice
    cvtColor(Bund, cvt, CV_BGR2GRAY);
    GaussianBlur(cvt, blur,Size(9,9),2,2);
    //Canny( blur, thres, 100, 100*2, 3 );
    threshold(blur, thres,70,255,THRESH_BINARY_INV);
    findContours(thres,contours, hierarchy,1,CHAIN_APPROX_NONE);

    //Bottom slice
    cvtColor(Bund2, cvt2, CV_BGR2GRAY);
    GaussianBlur(cvt2, blur2,Size(9,9),2,2);
    //Canny( blur2, thres2, 100, 100*2, 3 );
    threshold(blur2, thres2,70,255,THRESH_BINARY_INV);
    findContours(thres2,contours2, hierarchy2,1,CHAIN_APPROX_NONE);

    //Top slice
    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        double a = contourArea(contours[i], false);  //  Find the area of contour
        if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;                //Store the index of largest contour
        }
    }

    //Bottom slice
    for( int i = 0; i< contours2.size(); i++ ) // iterate through each contour.
    {
        double a = contourArea(contours2[i], false);  //  Find the area of contour
        if (a > largest_area2) {
            largest_area2 = a;
            largest_contour_index2 = i;                //Store the index of largest contour
        }
    }



    //Top slice
    Moments mu;
    mu = moments( contours[largest_contour_index], false );

    Point2f mc;
    mc = Point2f( mu.m10/mu.m00, mu.m01/mu.m00 );

    //Bottom slice
    Moments mu2;
    mu2 = moments( contours2[largest_contour_index2], false );

    Point2f mc2;
    mc2 = Point2f( mu2.m10/mu2.m00, mu2.m01/mu2.m00 );

    //Top slice
    drawContours( Bund, contours, largest_contour_index, Scalar(255,0,0), 2, 8, hierarchy, 0, Point() );
    circle( Bund, mc, 4, Scalar(0,255,255), -1, 8, 0 );

    //Bottom slice
    drawContours( Bund2, contours2, largest_contour_index2, Scalar(255,0,0), 2, 8, hierarchy2, 0, Point() );
    circle( Bund2, mc2, 4, Scalar(0,255,255), -1, 8, 0 );

    //Show whole frame with all lower half parts
    Point2f line_offset(0.0f,float(rows/4));
    arrowedLine(cameraFrame, mc2+line_offset*3, mc+line_offset*2, Scalar(0,0,255), 2, 8, 0);
    rectangle( cameraFrame,Point(0,rows/2),Point(cols-1,rows-1),Scalar( 0, 255, 0 ),1);
    rectangle( cameraFrame,Point(0,rows*0.75),Point(cols-1,rows-1),Scalar( 0, 255, 0 ),1);

    //Window
    namedWindow( "Frame", CV_WINDOW_AUTOSIZE );
    imshow("Frame", cameraFrame);
    //imshow("Threshold", thres);
}

int main()
{
    VideoCapture stream1(0);
    if(!stream1.isOpened()) {
        std::cerr << "cannot open camera" << std::endl;
        return -1;
    }

    Mat cameraFrame;
    stream1 >> cameraFrame;


    while (true) {
        stream1 >> cameraFrame;
        if(cameraFrame.empty()) {
            std::cerr<<"the frame is empty"<<std::endl;
            break;
        }
        find_line(cameraFrame);

        waitKey(1);
    }
    return 0;
}
