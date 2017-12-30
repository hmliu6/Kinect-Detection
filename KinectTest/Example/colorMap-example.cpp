// Standard Library
#include <iostream>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;

IKinectSensor* pSensor = nullptr;
IColorFrameSource* pFrameSource = nullptr;
IFrameDescription* pFrameDescription = nullptr;
IColorFrameReader* pFrameReader = nullptr;
int iWidth = 0, iHeight = 0;

int initKinect() {
	// Get default Sensor
	cout << "Try to get default sensor" << endl;
	if (GetDefaultKinectSensor(&pSensor) != S_OK) {
		cerr << "Get Sensor failed" << endl;
		return -1;
	}

	// Open sensor
	cout << "Try to open sensor" << endl;
	if (pSensor->Open() != S_OK)
	{
		cerr << "Can't open sensor" << endl;
		return -1;
	}

	// Get frame source
	cout << "Try to get color source" << endl;
	if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK)
	{
		cerr << "Can't get color frame source" << endl;
		return -1;
	}

	// Get frame description
	cout << "get color frame description" << endl;
	if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
	{
		pFrameDescription->get_Width(&iWidth);
		pFrameDescription->get_Height(&iHeight);
	}
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// Get frame reader
	cout << "Try to get color frame reader" << endl;
	if (pFrameSource->OpenReader(&pFrameReader) != S_OK)
	{
		cerr << "Can't get color frame reader" << endl;
		return -1;
	}

	// Release Frame source
	cout << "Release frame source" << endl;
	pFrameSource->Release();
	pFrameSource = nullptr;

	return 0;
}

void releaseKinect() {
	// Release frame reader
	cout << "Release frame reader" << endl;
	pFrameReader->Release();
	pFrameReader = nullptr;

	// Close Sensor
	cout << "close sensor" << endl;
	pSensor->Close();

	// Release Sensor
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;
}

int main(int argc, char** argv){

	int ret = initKinect();
	if (ret == -1)
		return 0;

	// Prepare OpenCV data
	cv::Mat	mImg(iHeight, iWidth, CV_8UC4);
	cv::Mat	mirrored(iHeight, iWidth, CV_8UC4);
	UINT uBufferSize = iHeight * iWidth * 4 * sizeof(BYTE);
	cv::namedWindow("Color Map");

	// Enter main loop
	while (true)
	{
		// Get last frame
		IColorFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK){
			// Copy to OpenCV image
			if (pFrame->CopyConvertedFrameDataToArray(uBufferSize, mImg.data, ColorImageFormat_Bgra) == S_OK){
				cv::flip(mImg, mirrored, 1);
				cv::imshow("Color Map", mirrored);
			}
			else{
				cerr << "Data copy error" << endl;
			}

			// Release frame
			pFrame->Release();
		}

		// Check keyboard input
		if (cv::waitKey(30) == VK_ESCAPE) {
			break;
		}
	}

	releaseKinect();
	return 0;
}