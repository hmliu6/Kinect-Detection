#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <string>

// OpenCV Header for Windows
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>


// For kinect v2
IKinectSensor* pSensor = nullptr;
IDepthFrameSource* pFrameSource = nullptr;
IFrameDescription* pFrameDescription = nullptr;
IDepthFrameReader* pFrameReader = nullptr;
int iWidth = 0, iHeight = 0;
UINT16 uDepthMin = 0, uDepthMax = 0;

// Here we use millimeter
const int fenceHeight = 2500;
const int kinectHeight = 1800;
const int fenceToKinect = 4000;
const int tolerance = 300;
const int maxDepthRange = 8000;


using namespace std;
using namespace cv;

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

void colorCalculation(int *upperColor, int *lowerColor){
    // Pythagoras's theorem, a^2 + b^2 = c^2
    double centreToKinect = sqrt(pow(fenceHeight - kinectHeight, 2) + pow(fenceToKinect, 2));
    // Get the bounding distance which fence lied within
    double upperDistance = centreToKinect + tolerance;
    double lowerDistance = centreToKinect - tolerance;
    // Since we get depth image by dividing 0-255 into number of depthRange pieces
    // So we use bounding distance to obtain corresponding colour range
    *upperColor = int(255.0 * upperDistance/maxDepthRange);
    *lowerColor = int(255.0 * lowerDistance/maxDepthRange);
}

int main(int argc, char **argv){
    int upperColor, lowerColor;

    kinectInit();

	// Create matrix object with same resolution of depth map
	cv::Mat mDepthImg(iHeight, iWidth, CV_16UC1);

	// mDepthImg = 16bit unsigned, mImg8bit = 8bit unsigned
	cv::Mat mImg8bit(iHeight, iWidth, CV_8UC1);
	cv::Mat mirrored(iHeight, iWidth, CV_8UC1);
    
    // Pass by reference
    colorCalculation(&upperColor, &lowerColor);

    // Get frame reader
	pFrameSource->OpenReader(&pFrameReader);

    while(true){
        // Get latest frame and check condition
		IDepthFrame* pFrame = nullptr;

		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK){
			// S_OK = execute successfully
			// E_PENDING = not ready for retrieving data
			// We may add timer to prevent busy waiting in state E_PENDING

			// Copy the depth map to cv::Mat mDepthImg object
			pFrame->CopyFrameDataToArray(iWidth * iHeight,
				reinterpret_cast<UINT16*>(mDepthImg.data));

            // Convert from 16bit to 8bit
			mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / 8000);

			// preFiltering(mImg8bit, upperColor, lowerColor);

			// cv::imshow to display image
			// cv::flip(mImg8bit, mirrored, 1);
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