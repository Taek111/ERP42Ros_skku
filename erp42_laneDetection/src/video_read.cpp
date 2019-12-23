#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    std::cout << "OpenCV version : " << CV_VERSION << std::endl;
     std::cout << "Major version : " << CV_MAJOR_VERSION << std::endl;
    ros::init(argc, argv, "opencv_test");
   cv::VideoCapture vc(1);//비디오 경로 입력
    if(!vc.isOpened()) return -1; //연결 실패
    vc.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    vc.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    cv::Mat img;
    cv::namedWindow("video", 1);
    while(1){
        vc >> img;
        if(img.empty()) break;
        cv::imshow("video", img);
        if(cv::waitKey(10) == 27) break; //ESC

    }
    cv::destroyAllWindows();
    return 0;
}
