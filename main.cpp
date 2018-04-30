//better use cstdio and cstdlib to replace stdio.h and stdlib.h in c++
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

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

Mat scan(Mat image, Mat descriptors2, vector<KeyPoint> keypoints2, Mat reference){
    Mat sign;
    Mat cvt;
    Mat blur;
    Mat thres;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    double polygonalApproxAccuracyRate = 0.05;

    vector< Point > approxCurve;

    cvtColor(image, cvt, CV_BGR2GRAY);
/*    GaussianBlur(cvt, blur,Size(5,5),0,0);
    //Para 6,7: block size and mean subtracted value
    adaptiveThreshold(blur, thres,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,11,2);
    findContours(thres,contours, hierarchy,1,CHAIN_APPROX_NONE);

    int largest_area=0;
    int largest_contour_index=0;

    //Check if no contours
    if (contours.empty()) {
        return thres;
    }

    //Find largest contour
    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        double a = contourArea(contours[i], false); //  Find the area of contour
        if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;    //Store the index of largest contour
        }
    }*/

    //######################
    Mat features;

    std::vector<KeyPoint> keypoints1;
    keypoints1.clear();
    Mat descriptors1;

    Ptr<Feature2D> orb = ORB::create(500);
    orb->detectAndCompute(image, Mat(), keypoints1, descriptors1);
    drawKeypoints(image,keypoints1,features);


    std::vector<DMatch> matches;
    matches.clear();
    if (keypoints1.size()!=0) {
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
        matcher->match(descriptors2, descriptors1, matches, Mat());

        sort(matches.begin(), matches.end());

        const int numGoodMatches = matches.size() * 0.15f;
        matches.erase(matches.begin() + numGoodMatches, matches.end());

        drawMatches(reference, keypoints2, image, keypoints1, matches, features);


        // Extract location of good matches
        std::vector<Point2f> points1, points2;
        for (size_t i = 0; i < matches.size(); i++) {
            points1.push_back(keypoints2[matches[i].queryIdx].pt);
            points2.push_back(keypoints1[matches[i].trainIdx].pt);
        }
        Mat h = findHomography(points2, points1, RANSAC);

        //cout << h << h.empty() << "\n";
        Mat imReg;
        if (h.empty()!=1) {
            warpPerspective(image, sign, h, Size(400,400));
            sign=reSize(sign);
            imshow("sign", sign );
        }
    }

    //imshow("features", features );
    //###########################

    //approxPolyDP(contours[largest_contour_index], approxCurve, double(contours[largest_contour_index].size()) * polygonalApproxAccuracyRate, true);
    //drawContours( image, contours, largest_contour_index, Scalar(255,255,255), 1, 4, hierarchy, 0, Point() );
    return features;
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

//####################################
    Mat reference = imread("50kmt.jpg");
    //Mat reference = imread("sign.png");

    std::vector<KeyPoint> keypoints1;
    Mat descriptors1;

    Ptr<Feature2D> orb = ORB::create(500);
    orb->detectAndCompute(reference, Mat(), keypoints1, descriptors1);
    drawKeypoints(reference,keypoints1,reference);
    //imshow("reference", reference );
    //###########################################

    while (true) {
        //Insert feed into frame mat
        stream1 >> cameraFrame;
        //Check if feed has stopped
        if (cameraFrame.empty()) {
            std::cerr << "the frame is empty" << std::endl;
            break;
        }

        imshow("Test", scan(cameraFrame, descriptors1, keypoints1, reference));


#ifdef __x86_64
        //Show the image/frame
        //namedWindow( "Frame", CV_WINDOW_AUTOSIZE );
        //imshow("Frame", cameraFrame);
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
