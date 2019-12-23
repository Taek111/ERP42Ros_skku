#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

class OpenCVWebCam {
	public:
		OpenCVWebCam() {
			cv::VideoCapture cap(CV_CAP_ANY); // open any camera
			if (!cap.isOpened()) {
				std::cout << "Could not open camera\n";
			}

			cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

			while (1) {
				cv::Mat frame;
				if (cap.read(frame)) {
					cv::imshow("Webcam", frame);
				}
				if (cv::waitKey(30) == 27) {
					// if "esc" is pressed end the program
					std::cout << "Closing the program because esc pressed";
					break;
				}
			}
		}
		~OpenCVWebCam() {
			cv::destroyWindow("Webcam");
		}
};

int main(int argc, char** argv) {
	// set up ros
	ros::init(argc, argv, "opencv_test");
	OpenCVWebCam webcam;
	ROS_INFO("Webcam Tested");
	return 0;
}