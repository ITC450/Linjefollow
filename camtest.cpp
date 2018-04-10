#include <opencv2/opencv.hpp>

using namespace cv;

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
                //Check if feed has stopped
                if(cameraFrame.empty()) {
                        std::cerr<<"the frame is empty"<<std::endl;
                        break;
                }
                namedWindow( "Frame", CV_WINDOW_AUTOSIZE );
                imshow("Frame", cameraFrame);
                char c=(char)waitKey(1);
                if(c==27)
                        break;
        }
        //Clean up
        stream1.release();
        destroyAllWindows();
        return 0;
}
