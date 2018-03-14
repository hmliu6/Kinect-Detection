#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#define Processor_cl 1

using namespace std;
using namespace cv;

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s){
	protonect_shutdown = true;
}

int main(){
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = NULL;
	libfreenect2::PacketPipeline  *pipeline = NULL;

	if(freenect2.enumerateDevices() == 0){
	    std::cout << "no device connected!" << std::endl;
	    return -1;
	}

	string serial = freenect2.getDefaultDeviceSerialNumber();

	int depthProcessor = Processor_cl;
	dev = freenect2.openDevice(serial);

	if(dev == 0){
	    std::cout << "failure opening device!" << std::endl;
	    return -1;
	}

	signal(SIGINT, sigint_handler);
	protonect_shutdown = false;

	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth);
	libfreenect2::FrameMap frames;

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);

	dev->start();

	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

	Mat depthmat;

	cv::namedWindow("depth", WND_PROP_ASPECT_RATIO);

	while(!protonect_shutdown){
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

		cv::imshow("depth", depthmat / 8000.0f);

		// Shutdown on escape
		int key = cv::waitKey(1);
		protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27));
		listener.release(frames);
	}

	dev->stop();
	dev->close();

	delete registration;


	std::cout << "Goodbye World!" << std::endl;
	return 0;
}
