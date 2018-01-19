// Standard Library
#include <iostream>
#include <string>
#include <bits/stdc++.h>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;

typedef struct {
	cv::Point centre;
	int distance;
} circleInfo;

IKinectSensor* pSensor = nullptr;
IDepthFrameSource* pFrameSource = nullptr;
IFrameDescription* pFrameDescription = nullptr;
IDepthFrameReader* pFrameReader = nullptr;
int iWidth = 0, iHeight = 0;
UINT16 uDepthMin = 0, uDepthMax = 0;

// Here we use millimeter
const int fenceHeight = 2500;
const int kinectHeight = 1700;
const int fenceToKinect = 3000;
const int tolerance = 200;
const int maxDepthRange = 8000;
const int rodLength = 42;
const int rodWidth = 5;

class Queue{
	public:
		void queue(int inputSize){
			validElement = 0;
			circleArray = (circleInfo *)malloc(sizeof(circleInfo) * inputSize);
			for(int i=0; i<inputSize; i++){
				circleArray[i].centre.x = -1;
				circleArray[i].centre.y = -1;
				circleArray[i].distance = -1;
			}
			arraySize = inputSize;
		}
		void enqueue(cv::Point input){
			if(validElement < arraySize){
				circleArray[validElement].centre.x = input.x;
				circleArray[validElement].centre.y = input.y;
				circleArray[validElement].distance = pow(input.x, 2) + pow(input.y, 2);
				validElement += 1;
			}
			else{
				for(int i=1; i<arraySize; i++){
					circleArray[i - 1].centre.x = circleArray[i].centre.x;
					circleArray[i - 1].centre.y = circleArray[i].centre.y;
					circleArray[i - 1].distance = circleArray[i].distance;
				}
				circleArray[arraySize - 1].centre.x = input.x;
				circleArray[arraySize - 1].centre.y = input.y;
				circleArray[arraySize - 1].distance = pow(input.x, 2) + pow(input.y, 2);
			}
		}
		cv::Point medianDistance(){
			int index;
			if(validElement == 1)
				return circleArray[validElement - 1].centre;
		
			int *tempDistance = (int *)malloc(sizeof(int) * validElement);
			for(int i=0; i<validElement; i++)
				tempDistance[i] = circleArray[i].distance;
			stable_sort(tempDistance, tempDistance + validElement);

			desiredValue = tempDistance[int(validElement / 2)];
			for(index=0; index<validElement; index++)
				if(circleArray[index].distance == desiredValue)
					break;
			
			return circleArray[index].centre;
		}

	private:
		circleInfo *circleArray;
		int validElement;
		int arraySize;
}

const int arraySize = 10;
Queue locationQueue(arraySize);

// Calculate minimum meaningful colour range
void colorCalculation(int *lowerColor){
    // Pythagoras's theorem, a^2 + b^2 = c^2
    double centreToKinect = sqrt(pow(fenceHeight - kinectHeight, 2) + pow(fenceToKinect, 2));
    double lowerDistance = centreToKinect - tolerance;
    // Map from depth value to grayscale
    *lowerColor = int(255.0 * lowerDistance/maxDepthRange);
}

int getRodCoordinates(cv::Mat image, int **whitePoints, int pointCount){
    // Initialize array with all zero to store satisfied x-coordinates
    int *rowWhiteNumber = (int *)malloc(image.rows * sizeof(int));
    int rowWhiteCount = 0, yCoordinates = 0;
    for(int i=0; i<10; i++)
        rowWhiteNumber[i] = -1;

    int currentX = 0, consecutiveCount = 0, lastHeight = 0;
    bool chosen = false, garbage = false;
    // Finding x-coordinate of inner circle in set of white points
    for(int i=0; i<pointCount; i++){
        // if(whitePoints[1][i] > 180 && whitePoints[1][i] < 190)
        //     cout << "{" << whitePoints[1][i] << ", " << whitePoints[0][i] << "}" << endl;
        // All points are already sorted according to x-coordinate
        if(whitePoints[0][i] < image.rows/2){
            currentX = 0;
            continue;
        }
        if(currentX != whitePoints[1][i]){
            // Re-count if x-coordinate changed
            currentX = whitePoints[1][i];
            lastHeight = whitePoints[0][i];
            consecutiveCount = 0;
            chosen = false;
            garbage = false;
        }
        else{
            // Give tolerance since poor accuracy for long distance
            if(whitePoints[0][i] - lastHeight <= 2 && !garbage){
                // Keep counting if we detected consecutive white
                consecutiveCount += 1;
                lastHeight = whitePoints[0][i];
            }
            // else
            //     garbage = true;
            // Large consecutive count means that it is probably part of rod
            if(consecutiveCount >= rodLength && !chosen){
                // Mark it and denoted as chosen to prevent re-adding
                rowWhiteNumber[rowWhiteCount] = currentX;
                rowWhiteCount += 1;
                chosen = true;
            }
        }
    }

    if(rowWhiteCount > 0){
        for(int i=0; i<rowWhiteCount; i++)
            cout << rowWhiteNumber[i] << endl;
        // Take sum of average to become x-coordinate of centre
        // for(int i=0; i<rowWhiteCount; i++){
        //     cout << rowWhiteNumber[i] << endl;
        //     yCoordinates += rowWhiteNumber[i];
        // }
        // yCoordinates = int(yCoordinates / rowWhiteCount);

        // New Algorithm: Find width of all objects in line and ignore unreasonable width
        int temp = 1;
        for(int i=temp; i<rowWhiteCount; i++){
            if(rowWhiteNumber[i] - rowWhiteNumber[i-1] != 1 || (i == rowWhiteCount - 1 && rowWhiteNumber[i] - rowWhiteNumber[i-1] == 1)){
                // Numbers represent pixels, so width has to +1
                // cout << "A: " << rowWhiteNumber[i-1] << endl;
                // cout << "B: " << rowWhiteNumber[temp-1] << endl;
                int width = rowWhiteNumber[i-1] - rowWhiteNumber[temp-1] + 1;
                if(i == rowWhiteCount - 1 && rowWhiteNumber[i] - rowWhiteNumber[i-1] == 1)
                    width = rowWhiteNumber[i] - rowWhiteNumber[temp-1] + 1;
                cout << width << endl;
                if(width > 1 && width <= rodWidth){
                    // Simply take middle one in the consecutive numbers
                    yCoordinates = rowWhiteNumber[temp - 1 + int(width/2)];

                    // Free used objects to prevent overflow
                    free(rowWhiteNumber);
                    return yCoordinates;
                }
                else{
                    temp = i + 1;
                }
            }
        }
        // Free used objects to prevent overflow
        free(rowWhiteNumber);
        return -1;
    }
    else{
        free(rowWhiteNumber);
        return -1;
    }
}

int getCircleCoordinates(cv::Mat image, int **whitePoints, int pointCount, int centreX){
    int listOfHeight = 0, countList = 0, centreY = 0;
    bool blackZone = false;
    for(int i=image.cols/2; i>=0; i--){
        int count = 0;
        for(int j=0; j<pointCount; j++){
            // Test every points if they are inside circle, Increase count
            int dY = pow((whitePoints[0][j] - i), 2);
            int dX = pow((whitePoints[1][j] - centreX), 2);
            // Circle that must have smaller radius than inner one
            if(dX + dY < pow(15, 2))
                count += 1;
        }
        // If we get enough amounts of count, it means that now it is looping in the black area
        if(countList > 10)
            blackZone = true;
        // Once we get a circle with too many white points, then we stop looping
        if(count > 10 && blackZone)
            break;
        else if(count <= 10){
            listOfHeight += i;
            countList += 1;
        }
    }
    // Take sum of average to get y-coordinate of centre
    if(countList > 0){
        centreY = int(listOfHeight / countList);
        return centreY;
    }
    else
        return -1;
}

int getCircleRadius(int **whitePoints, int pointCount, cv::Point centre){
    // Test for the maximum acceptable radius
    int largestRadius = 0;
    // 50 can be other values which is sufficiently large enough
    for(int i=0; i<50; i++){
        int count = 0;
        for(int j=0; j<pointCount; j++){
            // Test every points if they are inside circle, Increase count
            int dY = pow((whitePoints[0][j] - centre.y), 2);
            int dX = pow((whitePoints[1][j] - centre.x), 2);
            if(dX + dY < pow(i, 2))
                count += 1;
        }
        // Once we get a circle with too many white points, then we stop looping
        if(count > 10)
            break;
        // Keep storing largest radius value
        else if(count <= 10 && i > largestRadius){
            largestRadius = i;
        }
    }
    if(largestRadius > 0)
        return largestRadius;
    else
        return -1;
}

// Draw function for finding inner circle
void drawBoundingArea(cv::Mat rawImage, cv::Mat image, int **whitePoints, int pointCount) {
	// Centre of bounding circle (x, y)
	cv::Point centre;

	if (pointCount == 0) {
		cout << "Not found" << endl;
		return;
	}

	centre.x = getRodCoordinates(image, whitePoints, pointCount);
	if (centre.x == -1) {
		cout << "Cannot locate rod" << endl;
		return;
	}

	centre.y = getCircleCoordinates(image, whitePoints, pointCount, centre.x);
	if (centre.y == -1) {
		cout << "Cannot find centre" << endl;
		return;
	}

	locationQueue.enqueue(centre);
	cv::Point medianCentre = locationQueue.medianDistance();

	int largestRadius = getCircleRadius(whitePoints, pointCount, medianCentre);
	if (largestRadius == -1) {
		cout << "Cannot find radius" << endl;
		return;
	}

	// Draw circle in raw image instead of processed image
	circle(rawImage, medianCentre, largestRadius, CV_RGB(255, 255, 255), 2);
}

void preFiltering(cv::Mat rawImage, int lowerColorRange) {
	cv::Mat image;
	rawImage.copyTo(image);
	int *whitePoints[2], pointCount = 0;
	// whitePoints[0] = set of y-coordinates
	whitePoints[0] = (int *)malloc(10000 * sizeof(int));
	// whitePoints[1] = set of x-coordinates
	whitePoints[1] = (int *)malloc(10000 * sizeof(int));
	// Filter out unrelated pixels
	for(int j=0; j<image.cols; j++){
		for(int i=0; i<image.rows; i++){
			// Assume that the circle must be higher than image centre
			if(i >= image.cols/2)
				image.at<uchar>(i, j) = 0;
			// Trim out leftmost and rightmost 1/8 image to reduce noise
			else if(j <= image.rows/8)
				image.at<uchar>(i, j) = 0;
			else if(j >= image.rows* 7/8)
				image.at<uchar>(i, j) = 0;
			// Set all smaller than minimum colour value points to zero
			else if(image.at<uchar>(i, j) <= lowerColorRange)
				image.at<uchar>(i, j) = 0;
			else{
				// Set it to white and add to array for faster calculation
				whitePoints[0][pointCount] = i;
				whitePoints[1][pointCount] = j;
				pointCount += 1;
			}
		}
	}
	// Keep processing if there is at least one point
	if(pointCount > 0)
		drawBoundingArea(rawImage, image, whitePoints, pointCount);
	free(whitePoints[0]);
	free(whitePoints[1]);
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
	int lowerColor, key_pressed;
	int imageNumber = 1;
	kinectInit();

	// Create matrix object with same resolution of depth map
	cv::Mat mDepthImg(iHeight, iWidth, CV_16UC1);

	// mDepthImg = 16bit unsigned, mImg8bit = 8bit unsigned
	cv::Mat mImg8bit(iHeight, iWidth, CV_8UC1);

	cv::namedWindow("Depth Map");

	// Pass by reference
	colorCalculation(&lowerColor);

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

			preFiltering(mImg8bit, lowerColor);

			// cv::imshow to display image
			cv::imshow("Depth Map", mImg8bit);
			// Hashing from depth to colour h(0) = (0, 0, 0)

			// Release frame
			pFrame->Release();
		}

		key_pressed = cv::waitKey(30);

		// Break when enter pressed
		if (key_pressed == VK_RETURN) {
			string filename = "image" + std::to_string(imageNumber) + ".png";
			imwrite("throwing-ball\\" + filename, mImg8bit);
			imageNumber += 1;
		}
		else if (key_pressed == VK_ESCAPE)
			break;
	}

	returnKinect();

	return 0;
}