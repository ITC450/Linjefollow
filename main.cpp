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
  std::cout << rows << '\n';
  return rows;
}
int mat_cols(Mat mat){
  int cols = mat.cols;
  Size s = mat.size();
  cols = s.width;
  std::cout << cols << '\n';
  return cols;
}

Mat pre_proc(Mat mat, int y_akse, int x_akse){
  Rect firkant = Rect(0,y_akse/2,x_akse,y_akse/2);
  Mat Bund = mat(firkant);
  return Bund;
}
Mat pre_proc2(Mat mat, int x_akse, int y_akse){
  Rect firkant = Rect(0,x_akse/2,y_akse,x_akse/2);
  Mat Bund = mat(firkant);
  return Bund;
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
        int rows=mat_rows(cameraFrame);
        int cols=mat_cols(cameraFrame);

        Mat cvt;
        Mat blur;
        Mat thres;
        Mat contour;

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        while (true) {
                stream1 >> cameraFrame;

                if(cameraFrame.empty()) {
                        std::cerr<<"the frame is empty"<<std::endl;
                        break;
                }

                Mat Bund = pre_proc(cameraFrame, rows, cols);
                Mat Bund2 = pre_proc2(cameraFrame, rows, cols);

                cvtColor(Bund, cvt, CV_BGR2GRAY);
                GaussianBlur(cvt, blur,Size(9,9),2,2);
                //Canny( blur, thres, 100, 100*2, 3 );
                threshold(blur, thres,70,255,THRESH_BINARY_INV);
                findContours(thres,contours, hierarchy,1,CHAIN_APPROX_NONE);

                vector<Moments> mu(contours.size() );
                for( int i = 0; i < contours.size(); i++ )
                { mu[i] = moments( contours[i], false ); }

                vector<Point2f> mc( contours.size() );
                for( int i = 0; i < contours.size(); i++ )
                { mc[i] = Point2f( mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00 ); }

                for(auto i = 0; i < contours.size(); i++ )
                {
                        drawContours( Bund, contours, i, Scalar(178,102,255), 2, 8, hierarchy, 0, Point() );
                        circle( Bund, mc[i], 4, Scalar(0,255,255), -1, 8, 0 );
                }
                rectangle( cameraFrame,Point(0,rows/2),Point(cols-1,rows-1),Scalar( 0, 0, 255 ),1);
                namedWindow( "Frame", CV_WINDOW_AUTOSIZE );
                imshow("Frame", cameraFrame);
                //imshow("Threshold", thres);
                waitKey(1);
        }
        return 0;
}
