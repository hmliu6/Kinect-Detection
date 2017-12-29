// Standard Library
#include <iostream>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;

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

void colorCalculation(int *upperColor, int *lowerColor) {
	// Pythagoras's theorem, a^2 + b^2 = c^2
	double centreToKinect = sqrt(pow(fenceHeight - kinectHeight, 2) + pow(fenceToKinect, 2));
	// Get the bounding distance which fence lied within
	double upperDistance = centreToKinect + tolerance;
	double lowerDistance = centreToKinect - tolerance;
	// Since we get depth image by dividing 0-255 into number of depthRange pieces
	// So we use bounding distance to obtain corresponding colour range
	*upperColor = int(255.0 * upperDistance / maxDepthRange);
	*lowerColor = int(255.0 * lowerDistance / maxDepthRange);
}


void drawBoundingArea(cv::Mat rawImage, cv::Mat image, int **whitePoints, int pointCount) {
	int pointAvg[2] = { 0, 0 }, smallestY = image.cols / 2;
	for (int i = 0; i<pointCount; i++) {
		pointAvg[0] += whitePoints[0][i];
		pointAvg[1] += whitePoints[1][i];
	}
	// Centre of bounding circle (x, y)
	cv::Point centre;
	if (pointCount > 0) {
		pointAvg[0] = pointAvg[0] / pointCount;
		pointAvg[1] = pointAvg[1] / pointCount;
		centre.y = pointAvg[0]; centre.x = pointAvg[1];
	}
	else {
		centre.x = 0; centre.y = 0;
	}

	cout << "{ " << pointAvg[0] << ", " << pointAvg[1] << " }" << endl;

	for (int i = 0; i<pointCount; i++) {
		if (whitePoints[0][i] < pointAvg[0] + 5 && whitePoints[0][i] > pointAvg[0] - 5) {
			if (whitePoints[1][i] < smallestY)
				smallestY = i;
		}
	}
	int boundingRadius = pointAvg[0] - smallestY;

	int listOfHeight = 0, countList = 0;
	bool blackZone = false;
	for (int i = 0; i<boundingRadius; i++) {
		int count = 0;
		for (int j = 0; j<pointCount; j++) {
			int dY = pow((whitePoints[0][j] - (pointAvg[0] - i)), 2);
			int dX = pow((whitePoints[1][j] - pointAvg[1]), 2);
			if (dX + dY < pow(20, 2))
				count += 1;
		}
		if (countList > 10)
			blackZone = true;
		if (count >= 20 && blackZone)
			break;
		else if (count <= 10) {
			listOfHeight += (pointAvg[0] - i);
			countList += 1;
		}
	}

	cout << countList << endl;

	if (countList > 0)
		centre.y = listOfHeight / countList;
	int largestRadius = 0;
	for (int i = 0; i<boundingRadius; i++) {
		int count = 0;
		for (int j = 0; j<pointCount; j++) {
			int dY = pow((whitePoints[0][j] - centre.y), 2);
			int dX = pow((whitePoints[1][j] - centre.x), 2);
			if (dX + dY < pow(i, 2))
				count += 1;
		}
		if (count <= 10 && i > largestRadius) {
			largestRadius = i;
		}
	}
	cout << largestRadius << endl;
	cv::circle(rawImage, centre, largestRadius, CV_RGB(255, 255, 255), 2);
}

void preFiltering(cv::Mat rawImage, int upperColorRange, int lowerColorRange) {
	cv::Mat image;
	rawImage.copyTo(image);
	int *whitePoints[2], pointCount = 0;
	whitePoints[0] = (int *)malloc(10000 * sizeof(int));
	whitePoints[1] = (int *)malloc(10000 * sizeof(int));
	// Filter out unrelated pixels
	for (int j = 0; j<image.cols; j++) {
		for (int i = 0; i<image.rows; i++) {
			if (i >= image.cols / 2)
				image.at<uchar>(i, j) = 0;
			else if (j <= image.rows / 8)
				image.at<uchar>(i, j) = 0;
			else if (j >= image.rows * 7 / 8)
				image.at<uchar>(i, j) = 0;
			else if (image.at<uchar>(i, j) <= lowerColorRange)
				image.at<uchar>(i, j) = 0;
			else if (image.at<uchar>(i, j) >= upperColorRange)
				image.at<uchar>(i, j) = 0;
			else {
				image.at<uchar>(i, j) = 255;
				whitePoints[0][pointCount] = i;
				whitePoints[1][pointCount] = j;
				pointCount += 1;
			}
		}
	}
	if(pointCount > 0)
	    drawBoundingArea(rawImage, image, whitePoints, pointCount);
}


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
	int upperColor, lowerColor;
	kinectInit();

	// Get some depth only meta
	cout << "Reliable Distance: "
		<< uDepthMin << " ¡V " << uDepthMax << endl;

	// Create matrix object with same resolution of depth map
	cv::Mat mDepthImg(iHeight, iWidth, CV_16UC1);

	// mDepthImg = 16bit unsigned, mImg8bit = 8bit unsigned
	cv::Mat mImg8bit(iHeight, iWidth, CV_8UC1);

	cv::namedWindow("Depth Map");

	// Pass by reference
	colorCalculation(&upperColor, &lowerColor);

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
			mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / 8000);

			preFiltering(mImg8bit, upperColor, lowerColor);

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