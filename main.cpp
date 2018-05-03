//better use cstdio and cstdlib to replace stdio.h and stdlib.h in c++
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <time.h>
#include <string.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>

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

//Function for resizeing Mats
Mat reSize(Mat input){
    Mat output;
    Size size(55,55);//the dst image size,e.g.100x100
    resize(input,output,size);//resize image
    return output;
}

//Finds the center of mass of a Mat and draws it on a given Mat
int vej_foelger(Mat cameraFrame,int rows,int cols, int slice){
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

static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

struct sortArea {
    bool operator() (vector<Point> pt1, vector<Point> pt2) { return (contourArea(pt1,false) > contourArea(pt2, false));}
} mySortArea;

static void findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

//s    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    //pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    //pyrUp(pyr, timg, image.size());


    // blur will enhance edge detection
    Mat timg(image);
    medianBlur(image, timg, 3);
    cvtColor(timg, timg, CV_BGR2GRAY);

    vector<vector<Point> > contours;

    threshold(timg, timg,70,255,THRESH_BINARY_INV);



    //imshow("test", timg);
    // find contours and store them all as a list
    findContours(timg, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

    vector<Point> approx;

    // test each contour
    for( size_t i = 0; i < contours.size(); i++ )
    {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if( approx.size() == 4 &&
            fabs(contourArea(Mat(approx))) > 1000 &&
            isContourConvex(Mat(approx)) )
        {
            double maxCosine = 0;

            for( int j = 2; j < 5; j++ )
            {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = MAX(maxCosine, cosine);
            }

            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            if( maxCosine < 0.3 )
                squares.push_back(approx);
        }


    }
    if(squares.size()!=0) {
        sort(squares.begin(), squares.end(), mySortArea);
    }
}

static void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];

        int n = (int)squares[i].size();
        //dont detect the border
        if (p-> x > 3 && p->y > 3)
            polylines(image, &p, &n, 1, true, Scalar(0,255,0), 1, LINE_AA);
    }

    //imshow("features", image);
}

struct sortY {
    bool operator() (cv::Point2f pt1, cv::Point2f pt2) { return (pt1.y < pt2.y);}
} mySortY;
struct sortX {
    bool operator() (cv::Point2f pt1, cv::Point2f pt2) { return (pt1.x < pt2.x);}
} mySortX;

void pers_corr(Mat image, const vector<vector<Point> >& squares){
    static int filename=1;
    Mat fixed;
    Mat h;
    vector<Point2f> warped_dst;
    vector<Point2f> src_2f;
    warped_dst.clear();
    src_2f.clear();

    warped_dst.push_back(Point2f(0,0));
    warped_dst.push_back(Point2f(399,0));
    warped_dst.push_back(Point2f(0,399));
    warped_dst.push_back(Point2f(399,399));

    if (squares.size()>0){

        src_2f.push_back(Point2f(squares[0][0]));
        src_2f.push_back(Point2f(squares[0][1]));
        src_2f.push_back(Point2f(squares[0][2]));
        src_2f.push_back(Point2f(squares[0][3]));

        std::sort(src_2f.begin(),src_2f.end(),mySortY);
        std::sort(src_2f.begin(),src_2f.begin()+2,mySortX);
        std::sort(src_2f.begin()+2,src_2f.end(),mySortX);

        h = getPerspectiveTransform(src_2f, warped_dst);
    }

    //cout << h << h.empty() << "\n";
    if (h.empty()!=1) {
        warpPerspective(image, fixed, h, Size(400,400));
        fixed=reSize(fixed);
        string ff = "/home/pi/projekt/Linjefollow/imgtest/10/"+to_string(filename)+".png";
        imwrite(ff,fixed);
        filename++;
        sleep(1);
    }
}

void scan(Mat image){
    Mat features=image.clone();
    vector<vector<Point> > squares;
    findSquares(features, squares);
    cout << squares.size() << "\n";
    pers_corr(image,squares);
    drawSquares(image, squares);
}

int CV_motor_control(VideoCapture &stream1){

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
        if (cameraFrame.empty()) {
            std::cerr << "the frame is empty" << std::endl;
            break;
        }

        scan(cameraFrame);


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
