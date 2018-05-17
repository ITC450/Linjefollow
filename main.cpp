#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <chrono>
#include <condition_variable>

#include "matx.h"
#include "neu.h"

#ifdef __arm__
#include "motor.hpp"
#else

#include "motor_virt.hpp"

#endif

using namespace std;
using namespace cv;
//int speed;

bool quit = true;
bool ready = false;
bool ready2 = false;
bool thread_done = false;
int thread_vej = 2;
int thread_NN = 2;
int thread_state2 = 3;
mutex m;
mutex y;
condition_variable cv1;
condition_variable cv2;
Mat cameraFrame;
vector<int> id;
int point1;
int frames{0};
int status{-1};

//rows and cols return the height and width of the stream in pixels
int mat_rows(Mat mat) {
    int rows = mat.rows;
    Size s = mat.size();

    rows = s.height;
    cout << rows << '\n';
    return (rows);
}

int mat_cols(Mat mat) {
    int cols = mat.cols;
    Size s = mat.size();

    cols = s.width;
    cout << cols << '\n';
    return (cols);
}

//Prepprocessing of the stream, returns a sub Mat of the original stream Mat
Mat pre_proc(Mat mat, int y_akse, int x_akse, int slice) {
    Rect firkant = Rect(0, y_akse * (0.125 * (slice - 1)), x_akse, y_akse / 8);
    Mat Bund = mat(firkant);

    return (Bund);
}

//Function for resizeing Mats
Mat reSize(Mat input) {
    Mat output;
    Size size(20, 20);           //the dst image size,e.g.100x100

    resize(input, output, size); //resize image
    Rect firkant = Rect(2, 2, 16, 16);
    output = output(firkant);
    return (output);
}

static double angle(Point pt1, Point pt2, Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;

    return ((dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10));
}

struct sortArea {
    bool operator()(vector<Point> pt1, vector<Point> pt2) {
        return (contourArea(pt1, false) > contourArea(pt2, false));
    }
} mySortArea;

struct sortY {
    bool operator()(cv::Point2f pt1, cv::Point2f pt2) {
        return (pt1.y < pt2.y);
    }
} mySortY;

struct sortX {
    bool operator()(cv::Point2f pt1, cv::Point2f pt2) {
        return (pt1.x < pt2.x);
    }
} mySortX;

static void findSquares(const Mat &image, vector<vector<Point> > &squares) {
    //https://github.com/alyssaq/opencv/blob/master/squares.cpp
    squares.clear();

    Mat timg(image);
    medianBlur(image, timg, 3);
    cvtColor(timg, timg, CV_BGR2GRAY);

    vector<vector<Point> > contours;

    threshold(timg, timg, 70, 255, THRESH_BINARY_INV);

    findContours(timg, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

    vector<Point> approx;

    // test each contour
    for (size_t i = 0; i < contours.size(); i++) {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if (approx.size() == 4 &&
            fabs(contourArea(Mat(approx))) > 1000 &&
            isContourConvex(Mat(approx))) {
            double maxCosine = 0;

            for (int j = 2; j < 5; j++) {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                maxCosine = MAX(maxCosine, cosine);
            }

            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            if (maxCosine < 0.3) {
                squares.push_back(approx);
            }
        }
    }
    if (squares.size() != 0) {
        sort(squares.begin(), squares.end(), mySortArea);
    }
}

Mat pers_corr(Mat image, const vector<vector<Point> > &squares) {
    Mat fixed;
    Mat h;

    vector<Point2f> warped_dst;
    vector<Point2f> src_2f;
    warped_dst.clear();
    src_2f.clear();

    warped_dst.push_back(Point2f(0, 0));
    warped_dst.push_back(Point2f(399, 0));
    warped_dst.push_back(Point2f(0, 399));
    warped_dst.push_back(Point2f(399, 399));

    if (squares.size() > 0) {
        src_2f.push_back(Point2f(squares[0][0]));
        src_2f.push_back(Point2f(squares[0][1]));
        src_2f.push_back(Point2f(squares[0][2]));
        src_2f.push_back(Point2f(squares[0][3]));

        sort(src_2f.begin(), src_2f.end(), mySortY);
        sort(src_2f.begin(), src_2f.begin() + 2, mySortX);
        sort(src_2f.begin() + 2, src_2f.end(), mySortX);

        h = getPerspectiveTransform(src_2f, warped_dst);
    }
    if (h.empty() != 1) {
        warpPerspective(image, fixed, h, Size(400, 400));
        fixed = reSize(fixed);
        cvtColor(fixed, fixed, CV_BGR2GRAY);
        return (fixed);
    }
    return (image);
}

Mat scan(Mat image, vector<vector<Point> > &squares) {
    Mat features;
    {
        unique_lock<mutex> lk(m);
        cv1.wait(lk, [] { return thread_NN==2;});
        thread_NN--;
        cout << "NN m: " << thread_NN << "\n";
        features = image.clone();
    }
    cv1.notify_all();

    findSquares(features, squares);
    //cout << "Number of squares: " << squares.size() << "\n";
    features = pers_corr(features, squares);
    //Draw squares
    for (size_t i = 0; i < squares.size(); i++) {
        const Point *p = &squares[i][0];

        int n = (int) squares[i].size();
        //dont detect the border
        if (p->x > 3 && p->y > 3) {
            polylines(image, &p, &n, 1, true, Scalar(0, 255, 0), 1, LINE_AA);
        }
    }
    return (features);
}

//Finds the center of mass of a Mat and draws it on a given Mat
void vej_foelger(int rows, int cols, int slice) {
    //int afvigelse;
    //Mats and containers
    Mat cvt;
    Mat blur;
    Mat thres;
    Mat Bund;

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    usleep(3000);

    while (quit) {

        int largest_area = 0;
        int largest_contour_index = 0;

        //Top slice
        {
            unique_lock<mutex> lk(m);
            cv1.wait(lk, [] { return thread_vej==2;});
            thread_vej--;
            cout << "Vej_følger m: " << thread_vej << "\n";
            Bund = pre_proc(cameraFrame, rows, cols, slice);
        }
        cv1.notify_all();

        //Masks and find contours
        cvtColor(Bund, cvt, CV_BGR2GRAY);
        GaussianBlur(cvt, blur, Size(5, 5), 0, 0);
        threshold(blur, thres, 70, 255, THRESH_BINARY_INV);
        findContours(thres, contours, hierarchy, 1, CHAIN_APPROX_NONE);

        //Check if no contours
        if (!contours.empty()) {
            //Find largest contour
            for (int i = 0; i < contours.size(); i++)      // iterate through each contour.
            {
                double a = contourArea(contours[i], false); //  Find the area of contour
                if (a > largest_area) {
                    largest_area = a;
                    largest_contour_index = i;       //Store the index of largest contour
                }
            }

            //Find center of mass(area)
            Moments mu;
            mu = moments(contours[largest_contour_index], false);

            Point2f mc;
            mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
            {
                unique_lock<mutex> lky(y);

                point1 = mc.x - (cols / 2);
                thread_state2--;
                cout << "Vej_følger y: " << thread_state2 << "\n";
            }
            cv2.notify_all();
        }
    }
    return;
}

void fps_counter(chrono::time_point<chrono::high_resolution_clock> start, int &frames) {
    auto end = chrono::system_clock::now();

    chrono::duration<double> elapsed_seconds = end - start;
    cout << "Average fps: " << frames / elapsed_seconds.count() << "\n";
}

//General motor control unit
void motor_kontrol_enhed(int rows, int cols) {
    auto start = chrono::system_clock::now();
    auto pid_start = chrono::system_clock::now();
    int speed = 100;
    int status2;
    vector<int> id2;
    usleep(3000);

    while (quit) {

            {
                unique_lock<mutex> lky(y);
                cv2.wait(lky, [] { return thread_state2 == 1;});
                thread_state2--;
                cout << "Motor_Kontrol m: " << thread_vej << " : " << thread_NN << "\n";
                cout << "Motor_Kontrol y: " << thread_state2 << "\n";
                status2=status;
                id2=id;
            }
            if (id2[0] >= 0) {
                if (id2[0] == status2 && id2[1] == 2) {
                    switch (id[0]) {
                        //Kill
                        case 0:
			                speed = 0;
                            RightMotor(BACK, 0);
                            LeftMotor(BACK, 0);
                            cout << "Case 0 - Stopskilt" << '\n';
                            fps_counter(start, frames);
                            quit = false;
                            break;

                            //Stop
                        case 1:
                            RightMotor(FORWARD, 0);
                            LeftMotor(FORWARD, 0);
                            cout << "Case 1 - Parkeringsskilt" << '\n';
                            break;

                            //Slow
                        case 2:
                            speed = 100;
                            cout << "Case 2 - Ligeud" << '\n';
                            break;

                        case 3:
                            cout << "Case 3 - Højre" << '\n';
                            break;

                        case 4:
                            cout << "Case 4 - Venstre" << '\n';
                            break;

                        case 5:
                            speed = 100;
                            cout << "Case 5 - 10" << '\n';
                            break;

                            //Fast
                        case 6:
                            speed = 175;
                            cout << "Case 6 - 30" << '\n';
                            break;

                            //Left
                        case 7:
                            speed = 250;
                            cout << "Case 7 - 50" << '\n';
                            break;

                            //Right
                        default:
                            RightMotor(FORWARD, 0);
                            LeftMotor(FORWARD, 0);
                            break;
                    }
                }
                {
                    unique_lock<mutex> lky(y);
                    status = id[0];
                }
            } else {
                {
                    unique_lock<mutex> lky(y);
                    status = -1;
                }
            }

            if (status2 != 1) {
                {
                    unique_lock<mutex> lky(y);
                    MotorFollowLine(point1, speed, pid_start);
                }
            }

    }
    ready = true;
    cv1.notify_all();
}

void data_conv(Mat picture, matrix *m1) {
    int i = 0;
    int x{0}, y{0};

    for (int x = 0; x < picture.rows; x++) {
        for (int y = 0; y < picture.cols; y++) {
            elm(m1, i, 0) = picture.at<uchar>(x, y);
            i++;
        }
    }
}


void NN() {

    vector<vector<Point> > squares;
    Mat sign;
    cout << "Setting up NN........";
    matrix *m1;
    initmat(&m1, 256, 1, 0.0); //Vektor til input billed

    mlp_net *nn_net;
    matrix *uext, *nn_out, *normmat;
    int neu = 10;                                      //Antal neuroner
    int input = 256;                                     //Antal input til netværket
    int output = 9;                                       //Antal output fra netværket
    load_bpenet("DataNet.nn", &nn_net, &input, &neu, &output); //Netværk læses ind
    loadMatD("uext", &uext);                                   //Mormereings værdier læses ind
    initmat(&normmat, input, 1, 0.0);                          //Vektor til normering af billed
    initmat(&nn_out, output, 1, 0.0);                          //Vektor til output fra det neurale netværk
    cout << "done\n";
    usleep(3000);
    while (quit) {


            sign = scan(cameraFrame, squares);
        {
            unique_lock<mutex> lky(y);
            id[0] = -1;
            //cout << "Squares " << squares.size() << "\n";
            if (squares.size() != 0) {
                data_conv(sign, m1);                           //Konvertering fra Mat til matrix
                matnormpext(m1, &normmat, uext, 1);            //Normering af data
                bpe_forward(normmat, nn_net, &nn_out);         //Afvikling feed forward netværk
                float value{.0};
                for (int k = 0; k < 9; k++) {
                    if (value < elm(nn_out, k, 0) && elm(nn_out, k, 0) > 0.8) {
                        value = elm(nn_out, k, 0);
                        id[0] = k;
                    }
                }
                if (id[0] == status) {
                    id[1]++;
                } else {
                    id[1] = 0;
                }
                if (id[1] > 2) {
                    id[1] = 3;
                }

            }
            //cout << "Status: " << id[0] << "\n";
            //cout << "Count up: " << id[1] << '\n';
            thread_state2--;
            cout << "NN y:" << thread_state2 << "\n";
        }
        cv2.notify_all();
    }
}

int CV_motor_control(VideoCapture &stream1) {
    //Init/setup

    id.push_back(-1);
    id.push_back(0);
    Mat cameraFrameOrg;
    cout << "Setting up motors....";
    MotorInit();
    cout << "done\n";
    if (!stream1.isOpened()) {
        cerr << "cannot open camera" << endl;
        return (-1);
    }


    stream1 >> cameraFrame;
    //Gets the resolution of the feed
    cout << "Resolution: " << '\n';
    int rows = mat_rows(cameraFrame);
    int cols = mat_cols(cameraFrame);
    //VideoWriter video("linefollower.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(cols, rows));
    thread nn(NN);
    thread vej(vej_foelger, rows, cols, 8);
    thread motor(motor_kontrol_enhed, rows, cols);
    //Video from camera

    //.set is for controlling size of stream and the stream mat
    //stream1.set(CV_CAP_PROP_FRAME_WIDTH,2000);
    //stream1.set(CV_CAP_PROP_FRAME_HEIGHT,1500);

    //Setup mat for source frame and insert feed into mat

    //Save as  settings
    thread_state2 = 3;
    while (quit) {
        //Insert feed into frame mat
        stream1 >> cameraFrameOrg;
        //Check if feed has stopped
        if (cameraFrameOrg.empty()) {
            cerr << "the frame is empty" << endl;
            break;
        }

        {
            unique_lock<mutex> lk(m);
            cameraFrame = cameraFrameOrg.clone();
            if (thread_vej == 1){
                thread_vej=2;
                cout << "Kam vej:     " << thread_vej << "\n";

            }
            if (thread_NN == 1){
                thread_NN=2;
                cout << "Kam NN:     " << thread_NN << "\n";

            }
            if (thread_state2 == 0){
                thread_state2=3;
                cout << "Kam y:     " << thread_state2 << "\n";
                frames++;
            }

            ready = true;
        }
        cv1.notify_all();
            //UI, bottom half
      //      rectangle(cameraFrameOrg, Point(0, rows * 0.875), Point(cols - 1, rows - 1), Scalar(0, 255, 0), 1);
#ifdef __x86_64
            //Show the image/frame
            namedWindow("Frame", CV_WINDOW_AUTOSIZE);
            imshow("Frame", cameraFrame);
#endif
            //video.write(cameraFrame);




        //Esc to close
        char c = (char) waitKey(1);
        if (c == 27) {

            break;
        }
    }
    //ready =true;
    //ready2 = true;
    RightMotor(BACK, 0);
    LeftMotor(BACK, 0);
    quit = false;
    thread_NN=2;
    thread_vej=2;
    thread_state2=3;
    cv1.notify_all();
    cv2.notify_all();
    nn.join();
    vej.join();
    motor.join();

    return (0);
}

int main() {
    cout << "Setting up camera....";
    VideoCapture stream1(0);
    cout << "done\n";
    CV_motor_control(stream1);
    //Clean up

    stream1.release();
    destroyAllWindows();
}
