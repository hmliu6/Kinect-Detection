// Standard Library
#include <iostream>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;

IKinectSensor* pSensor = nullptr;
IDepthFrameSource* pFrameSource = nullptr;
IFrameDescription* pFrameDescription = nullptr;
IDepthFrameReader* pFrameReader = nullptr;
int iWidth = 0, iHeight = 0;
UINT16 uDepthMin = 0, uDepthMax = 0;

void kinectInit() {
	// Get default kinect sensor
	GetDefaultKinectSensor(&pSensor);

	// Open sensor
	pSensor->Open();

	// Get frame source
	pSensor->get_DepthFrameSource(&pFrameSource);

	// Get frame description
	pFrameSource->get_FrameDescription(&pFrameDescription);
	pFrameDescription->get_Width(&iWidth);
	pFrameDescription->get_Height(&iHeight);
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// Get some depth only meta
	pFrameSource->get_DepthMinReliableDistance(&uDepthMin);
	pFrameSource->get_DepthMaxReliableDistance(&uDepthMax);
}

void returnKinect() {
	// Release frame reader
	pFrameReader->Release();
	pFrameReader = nullptr;

	// Release frame source
	pFrameSource->Release();
	pFrameSource = nullptr;

	// Close Sensor
	pSensor->Close();

	// Release Sensor
	pSensor->Release();
	pSensor = nullptr;
}

int main(int argc, char** argv){
	kinectInit();
	// Get some depth only meta
	cout << "Reliable Distance: " << uDepthMin << " V " << uDepthMax << endl;

	// Create matrix object with same resolution of depth map
	cv::Mat mDepthImg(iHeight, iWidth, CV_16UC1);
	// mDepthImg = 16bit unsigned, mImg8bit = 8bit unsigned
	cv::Mat mImg8bit(iHeight, iWidth, CV_8UC1);
	cv::namedWindow("Depth Map");

	// Get frame reader
	pFrameSource->OpenReader(&pFrameReader);

	while (true){
		// Get latest frame and check condition
		IDepthFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK){
			// S_OK = execute successfully
			// E_PENDING = not ready for retrieving data
			// We may add timer to prevent busy waiting in state E_PENDING

			// Copy the depth map to cv::Mat mDepthImg object
			pFrame->CopyFrameDataToArray(iWidth * iHeight,
				reinterpret_cast<UINT16*>(mDepthImg.data));

			// Here we can perform some calculation to finish some tasks
			// ...

			// Convert from 16bit to 8bit
			mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax);

			// cv::imshow to display image
			cv::imshow("Depth Map", mImg8bit);
			// Hashing from depth to colour h(0) = (0, 0, 0)

			// Release frame
			pFrame->Release();
		}
		// Break when enter pressed
		if (cv::waitKey(30) == VK_RETURN) {
			break;
		}
	}
	returnKinect();
	return 0;
}