#define IMAGE_FORMAT uchar

// Standard Library
#include <iostream>
#include <string>
#include <algorithm>
#include <math.h>
#include <stdlib.h>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;
using namespace cv;

cv::Mat rawImage, imageForBall, rodImage;

typedef struct {
	cv::Point centre;
	int distance;
    int maxRadius;
} circleInfo;

typedef struct {
    cv::Point ballCentre;
    int zDistance;
} ballInfo;

typedef struct {
    cv::Point point1;
    int z1;
    cv::Point point2;
    int z2;
    cv::Point2d intersect;
    double twoPointDistance;
    double z_interpolation;
} intersectionPoint;

IKinectSensor* pSensor = nullptr;
IDepthFrameSource* pFrameSource = nullptr;
IFrameDescription* pFrameDescription = nullptr;
IDepthFrameReader* pFrameReader = nullptr;
int iWidth = 0, iHeight = 0;
UINT16 uDepthMin = 0, uDepthMax = 0;

// Here we use millimeter
const int fenceHeight = 2500;
// Kinect height may be changed
const int kinectHeight = 1300;
// Smallest value here
const int fenceToKinect = 3000;
const int tolerance = 200;
const int maxDepthRange = 8000;
const int rodLength = 35;
const int rodWidth = 7;

const int medianBlurValue = 5;
const int cannyLower = 10;
const int cannyUpper = 150;

// Global results of Goal and Ball tracing
int detectedBall = 0, recordedPos = 0, zPos = -1;
ballInfo ballPath[20];
circleInfo outputCircle;
bool circleExist = false;

class Queue{
	public:
		Queue(int inputSize){
            // Constructor
			validElement = 0;
			circleArray = new circleInfo[inputSize];
			for(int i=0; i<inputSize; i++){
				circleArray[i].centre.x = -1;
				circleArray[i].centre.y = -1;
				circleArray[i].distance = -1;
			}
			arraySize = inputSize;
		}
        
		void enqueue(cv::Point input, int maxRadius){
            // Dequeue and enqueue input element if full
			if(validElement < arraySize){
				circleArray[validElement].centre.x = input.x;
				circleArray[validElement].centre.y = input.y;
				circleArray[validElement].distance = pow(input.x, 2) + pow(input.y, 2);
                circleArray[validElement].maxRadius = maxRadius;
				validElement += 1;
			}
			else{
				for(int i=1; i<arraySize; i++){
					circleArray[i - 1].centre.x = circleArray[i].centre.x;
					circleArray[i - 1].centre.y = circleArray[i].centre.y;
					circleArray[i - 1].distance = circleArray[i].distance;
                    circleArray[i - 1].maxRadius = circleArray[i].maxRadius;
				}
				circleArray[arraySize - 1].centre.x = input.x;
				circleArray[arraySize - 1].centre.y = input.y;
				circleArray[arraySize - 1].distance = pow(input.x, 2) + pow(input.y, 2);
                circleArray[arraySize - 1].maxRadius = maxRadius;
			}
		}

		circleInfo medianDistance(){
			int index;
			if(validElement == 1)
				return circleArray[validElement - 1];

            // Sort according to centre distance with (0, 0) and radius
			int *tempDistance = new int[validElement];
			for(int i=0; i<validElement; i++)
				tempDistance[i] = circleArray[i].distance * 1000 + circleArray[i].maxRadius;
			sort(tempDistance, tempDistance + validElement);

            // Take most reasonable one to be centre
            int desiredList[3], desiredValue;
            for(int i=0; i<3; i++)
                desiredList[i] = tempDistance[int(validElement / 2) - 1 + i];
            for(int i=0; i<3; i++)
                if(abs((desiredList[i] % 1000) - (desiredList[(i + 1) % 3] % 1000)) < 2){
                    if(desiredList[i] > desiredList[(i + 1) % 3])
                        desiredValue = desiredList[i];
                    else
                        desiredValue = desiredList[(i + 1) % 3];
                }

			for(index=0; index<validElement; index++)
				if(circleArray[index].distance * 1000 + circleArray[index].maxRadius == desiredValue)
					break;
			
            delete[] tempDistance;
			return circleArray[index];
		}

        void deleteAllocation(){
            delete[] circleArray;
        }

	private:
		circleInfo *circleArray;
		int validElement;
		int arraySize;
};

int arraySize = 10;
Queue *locationQueue = new Queue(arraySize);

// Calculate minimum meaningful colour range
void colorCalculation(int *lowerColor){
    // Pythagoras's theorem, a^2 + b^2 = c^2
    double centreToKinect = sqrt(pow(fenceHeight - kinectHeight, 2) + pow(fenceToKinect, 2));
    double lowerDistance = centreToKinect - tolerance;
    // Map from depth value to grayscale, change here if using raw 16 bits
    *lowerColor = int(255.0 * lowerDistance/maxDepthRange);
}

int getRodCoordinates(int imageWidth, int **whitePoints, int pointCount){
    // Initialize array with all zero to store satisfied x-coordinates
    int *rowWhiteNumber = new int[imageWidth];
    int rowWhiteCount = 0, yCoordinates = 0;
    for(int i=0; i<10; i++)
        rowWhiteNumber[i] = -1;

    int currentX = 0, consecutiveCount = 0, lastHeight = 0;
    bool chosen = false, garbage = false;
    // Finding x-coordinate of inner circle in set of white points
    for(int i=0; i<pointCount; i++){
        // All points are already sorted according to x-coordinate
        if(whitePoints[0][i] < imageWidth / 2){
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
        // New Algorithm: Find width of all objects in line and ignore unreasonable width
        int temp = 1;
        for(int i=temp; i<rowWhiteCount; i++){
            if(rowWhiteNumber[i] - rowWhiteNumber[i-1] != 1 || (i == rowWhiteCount - 1 && rowWhiteNumber[i] - rowWhiteNumber[i-1] == 1)){
                // Numbers represent pixels, so width has to +1
                int width = rowWhiteNumber[i-1] - rowWhiteNumber[temp-1] + 1;
                if(i == rowWhiteCount - 1 && rowWhiteNumber[i] - rowWhiteNumber[i-1] == 1)
                    width = rowWhiteNumber[i] - rowWhiteNumber[temp-1] + 1;
                if(width > 1 && width <= rodWidth){
                    // Simply take middle one in the consecutive numbers
                    yCoordinates = rowWhiteNumber[temp - 1 + int(width/2)];

                    // Free used objects to prevent overflow
                    delete[] rowWhiteNumber;
                    return yCoordinates;
                }
                else{
                    temp = i + 1;
                }
            }
        }
        // Free used objects to prevent overflow
        delete[] rowWhiteNumber;
        return -1;
    }
    else{
        delete[] rowWhiteNumber;
        return -1;
    }
}

int getCircleCoordinates(int imageHeight, int **whitePoints, int pointCount, int centreX){
    int listOfHeight = 0, countList = 0, centreY = 0, threshold = 15;
    bool blackZone = false;
    for(int i=imageHeight; i>=0; i--){
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
        if(countList > threshold)
            blackZone = true;
        // Once we get a circle with too many white points, then we stop looping
        if(count > threshold && blackZone)
            break;
        else if(count <= threshold){
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
    int largestRadius = 0, threshold = 15;
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
        if(count > threshold)
            break;
        // Keep storing largest radius value
        else if(count <= threshold && i > largestRadius){
            largestRadius = i;
        }
    }
    if(largestRadius > 0)
        return largestRadius;
    else
        return -1;
}

// Draw function for finding inner circle
void drawBoundingArea(cv::Mat rawImage, cv::Mat rodImage, int **whitePoints, int pointCount){
    // Centre of bounding circle (x, y)
    cv::Point centre;

    if (pointCount == 0) {
		cout << "Not found" << endl;
		return;
	}
    
    centre.x = getRodCoordinates(rodImage.rows, whitePoints, pointCount);
    if(centre.x == -1){
        cout << "Cannot locate rod" << endl;
        return;
    }

    centre.y = getCircleCoordinates(rodImage.cols / 2, whitePoints, pointCount, centre.x);
    if(centre.y == -1){
        cout << "Cannot find centre" << endl;
        return;
    }

    int largestRadius = getCircleRadius(whitePoints, pointCount, centre);
	if (largestRadius == -1) {
		cout << "Cannot find radius" << endl;
		return;
	}
    
    locationQueue->enqueue(centre, largestRadius);
	outputCircle = locationQueue->medianDistance();

	// Draw circle in raw image instead of processed image
    if(outputCircle.centre.x > 0 && outputCircle.centre.y > 0 && outputCircle.maxRadius > 0){
        circleExist = true;
        circle(rawImage, outputCircle.centre, outputCircle.maxRadius, CV_RGB(255, 255, 255), 2);

        // Locate z-Position for goal circle
        for(int j=0; j<pointCount; j++){
            // Test every points if they are on circle
            int dY = pow((whitePoints[0][j] - outputCircle.centre.y), 2);
            int dX = pow((whitePoints[1][j] - outputCircle.centre.x), 2);
            if(dX + dY == pow(outputCircle.maxRadius, 2)){
                int tempPosition = int(rodImage.at<IMAGE_FORMAT>(whitePoints[0][j], whitePoints[1][j]));
                if(tempPosition > zPos)
                    zPos = tempPosition;
            }
        }
    }
}

void preFiltering(cv::Mat rawImage, cv::Mat rodImage, int lowerColorRange){
    int *whitePoints[2], pointCount = 0;
    // Reset to -1 since it will be updated each frame
    outputCircle.centre = cv::Point(-1, -1);
    outputCircle.maxRadius = -1;
    // whitePoints[0] = set of y-coordinates
    whitePoints[0] = new int[100000];
    // whitePoints[1] = set of x-coordinates
    whitePoints[1] = new int[100000];
    // Filter out unrelated pixels, change here if using raw 16 bits
    for(int j=0; j<rodImage.cols; j++){
        for(int i=0; i<rodImage.rows; i++){
            // Assume that the circle must be higher than image centre
            if(i >= rodImage.cols/2)
                rodImage.at<IMAGE_FORMAT>(i, j) = 0;
            // Trim out leftmost and rightmost 1/8 image to reduce noise
            else if(j <= rodImage.rows/8)
                rodImage.at<IMAGE_FORMAT>(i, j) = 0;
            else if(j >= rodImage.rows* 7/8)
                rodImage.at<IMAGE_FORMAT>(i, j) = 0;
            // Set all smaller than minimum colour value points to zero
            else if(rodImage.at<IMAGE_FORMAT>(i, j) <= lowerColorRange)
                rodImage.at<IMAGE_FORMAT>(i, j) = 0;
            else{
                // Set it to white and add to array for faster calculation
                if(pointCount < 100000){
                whitePoints[0][pointCount] = i;
                whitePoints[1][pointCount] = j;
                pointCount += 1;
            }
        }
    }
    }
    // Keep processing if there is at least one point
    if(pointCount > 0)
        drawBoundingArea(rawImage, rodImage, whitePoints, pointCount);
    delete[] whitePoints[0];
    delete[] whitePoints[1];
}

void goalDetection(){
    bool result = false, intersection = false;
    double threshold = 0.1;
    int pointCount = 0;
    intersectionPoint *pointsOnLine = new intersectionPoint[3];
    // Case 1: Get two intersection points for path
    for(int i=1; i<recordedPos; i++){
        // The ball should fly with increasing z-distance
        // cout << ballPath[i].zDistance << endl;
        if(ballPath[i].zDistance - ballPath[i - 1].zDistance < 0)
            result = false;

        // At least one point intersection between circle and path only considering xy-plane
        // Here Point(x, y) is correct
        double lineSlope, lineIntercept, aCoefficient, bCoefficient, cCoefficient, delta;
		if (ballPath[i].ballCentre.x - ballPath[i - 1].ballCentre.x == 0)
			return;
        lineSlope = (ballPath[i].ballCentre.y - ballPath[i - 1].ballCentre.y) / (ballPath[i].ballCentre.x - ballPath[i - 1].ballCentre.x);
        lineIntercept = -1 * lineSlope * ballPath[i - 1].ballCentre.x + ballPath[i - 1].ballCentre.y;
        aCoefficient = 1 + pow(lineSlope, 2);
        bCoefficient = 2.0 * lineSlope * (lineIntercept - outputCircle.centre.y) - 2.0 * outputCircle.centre.x;
        cCoefficient = pow(outputCircle.centre.x, 2) + pow(lineIntercept - outputCircle.centre.y, 2) - pow(outputCircle.maxRadius, 2);

        delta = pow(bCoefficient, 2) - 4.0 * aCoefficient * cCoefficient;
        if(delta >= 0){
            cv::Point2d intersect1, intersect2;
            // Retrieve two intersection points
            intersect1.x = (-1 * bCoefficient + sqrt(delta)) / (2.0 * aCoefficient);
            intersect1.y = lineSlope * intersect1.x + lineIntercept;
            intersect2.x = (-1 * bCoefficient - sqrt(delta)) / (2.0 * aCoefficient);
            intersect2.y = lineSlope * intersect2.x + lineIntercept;

            double twoPointDistance, intersect1DistanceSum, intersect2DistanceSum;
            // Check the solutions are within line segments or not
            twoPointDistance = sqrt(pow(ballPath[i].ballCentre.x - ballPath[i - 1].ballCentre.x, 2) + pow(ballPath[i].ballCentre.y - ballPath[i - 1].ballCentre.y, 2));
            // If distance of new point with two points is larger than distance of two points
            intersect1DistanceSum = sqrt(pow(intersect1.x - ballPath[i].ballCentre.x, 2) + pow(intersect1.y - ballPath[i].ballCentre.y, 2))
                                  + sqrt(pow(intersect1.x - ballPath[i - 1].ballCentre.x, 2) + pow(intersect1.y - ballPath[i - 1].ballCentre.y, 2));
            intersect2DistanceSum = sqrt(pow(intersect2.x - ballPath[i].ballCentre.x, 2) + pow(intersect2.y - ballPath[i].ballCentre.y, 2))
                                  + sqrt(pow(intersect2.x - ballPath[i - 1].ballCentre.x, 2) + pow(intersect2.y - ballPath[i - 1].ballCentre.y, 2));
            
            // Return only points lied within line segments
            if(fabs(intersect1DistanceSum - twoPointDistance) < threshold){
                // cv::line(rawImage, cv::Point(intersect1.x - 5, intersect1.y), cv::Point(intersect1.x + 5, intersect1.y), Scalar(255, 255, 0), 2);
                // cv::line(rawImage, cv::Point(intersect1.x, intersect1.y - 5), cv::Point(intersect1.x, intersect1.y + 5), Scalar(255, 255, 0), 2);
                pointsOnLine[pointCount].point1 = ballPath[i - 1].ballCentre;
                pointsOnLine[pointCount].z1 = ballPath[i - 1].zDistance;
                pointsOnLine[pointCount].point2 = ballPath[i].ballCentre;
                pointsOnLine[pointCount].z2 = ballPath[i].zDistance;
                pointsOnLine[pointCount].intersect = intersect1;
                pointsOnLine[pointCount].twoPointDistance = twoPointDistance;
                pointCount += 1;
            }
            else if(fabs(intersect2DistanceSum - twoPointDistance) < threshold){
                // cv::line(rawImage, cv::Point(intersect2.x - 5, intersect2.y), cv::Point(intersect2.x + 5, intersect2.y), Scalar(255, 255, 0), 2);
                // cv::line(rawImage, cv::Point(intersect2.x, intersect2.y - 5), cv::Point(intersect2.x, intersect2.y + 5), Scalar(255, 255, 0), 2);
                pointsOnLine[pointCount].point1 = ballPath[i - 1].ballCentre;
                pointsOnLine[pointCount].z1 = ballPath[i - 1].zDistance;
                pointsOnLine[pointCount].point2 = ballPath[i].ballCentre;
                pointsOnLine[pointCount].z2 = ballPath[i].zDistance;
                pointsOnLine[pointCount].intersect = intersect2;
                pointsOnLine[pointCount].twoPointDistance = twoPointDistance;
                pointCount += 1;
            }
        }
    }

    if(pointCount < 2)
        result = false;
    else{
        // Interpolate all points lied on line
        for(int i=0; i<pointCount; i++){
            int zDiff = pointsOnLine[i].z1 - pointsOnLine[i].z2;
            double distanceWithPoint1 = sqrt(pow(pointsOnLine[i].point1.x - pointsOnLine[i].intersect.x, 2) + pow(pointsOnLine[i].point1.y - pointsOnLine[i].intersect.y, 2));
            pointsOnLine[i].z_interpolation = pointsOnLine[i].z1 - zDiff * distanceWithPoint1 / pointsOnLine[i].twoPointDistance;
            cout << pointsOnLine[i].z_interpolation << endl;
        }
        // z-value: pointsOnLine[0].z_interpolation < zPos < pointsOnLine[1].z_interpolation
        cout << "Rod Position: " << zPos << endl;
        if(pointsOnLine[0].z_interpolation < zPos && zPos < pointsOnLine[1].z_interpolation)
            result = true;
    }

    // Case 2: Get only one intersection points for path
    if(result == false){
        double lineSlope, lineIntercept, aCoefficient, bCoefficient, cCoefficient, delta;
		if (ballPath[recordedPos - 1].ballCentre.x - ballPath[recordedPos - 2].ballCentre.x == 0)
			return;
        lineSlope = (ballPath[recordedPos - 1].ballCentre.y - ballPath[recordedPos - 2].ballCentre.y) / (ballPath[recordedPos - 1].ballCentre.x - ballPath[recordedPos - 2].ballCentre.x);
        lineIntercept = -1 * lineSlope * ballPath[recordedPos - 2].ballCentre.x + ballPath[recordedPos - 2].ballCentre.y;
        aCoefficient = 1 + pow(lineSlope, 2);
        bCoefficient = 2.0 * lineSlope * (lineIntercept - outputCircle.centre.y) - 2.0 * outputCircle.centre.x;
        cCoefficient = pow(outputCircle.centre.x, 2) + pow(lineIntercept - outputCircle.centre.y, 2) - pow(outputCircle.maxRadius, 2);

        delta = pow(bCoefficient, 2) - 4.0 * aCoefficient * cCoefficient;
        if(delta >= 0){
            cv::Point2d intersect1, intersect2;
            // Retrieve two intersection points
            intersect1.x = (-1 * bCoefficient + sqrt(delta)) / (2.0 * aCoefficient);
            intersect1.y = lineSlope * intersect1.x + lineIntercept;
            intersect2.x = (-1 * bCoefficient - sqrt(delta)) / (2.0 * aCoefficient);
            intersect2.y = lineSlope * intersect2.x + lineIntercept;

            // Interpolate z-distance value
            double twoPointDistance, intersect1DistanceSum, intersect2DistanceSum;
            // Check the solutions are within line segments or not
            twoPointDistance = sqrt(pow(ballPath[recordedPos - 1].ballCentre.x - ballPath[recordedPos - 2].ballCentre.x, 2) + pow(ballPath[recordedPos - 1].ballCentre.y - ballPath[recordedPos - 2].ballCentre.y, 2));
            // If distance of new point with two points is larger than distance of two points
            intersect1DistanceSum = sqrt(pow(intersect1.x - ballPath[recordedPos - 1].ballCentre.x, 2) + pow(intersect1.y - ballPath[recordedPos - 1].ballCentre.y, 2))
                                  + sqrt(pow(intersect1.x - ballPath[recordedPos - 2].ballCentre.x, 2) + pow(intersect1.y - ballPath[recordedPos - 2].ballCentre.y, 2));
            intersect2DistanceSum = sqrt(pow(intersect2.x - ballPath[recordedPos - 1].ballCentre.x, 2) + pow(intersect2.y - ballPath[recordedPos - 1].ballCentre.y, 2))
                                  + sqrt(pow(intersect2.x - ballPath[recordedPos - 2].ballCentre.x, 2) + pow(intersect2.y - ballPath[recordedPos - 2].ballCentre.y, 2));
            
            // Return only points lied within line segments
            int zDiff = ballPath[recordedPos - 2].distance - ballPath[recordedPos - 1].distance;
            double distanceWithPoint1 = sqrt(pow(ballPath[recordedPos - 2].ballCentre.x - intersect1.x, 2) + pow(ballPath[recordedPos - 2].ballCentre.y - intersect1.y, 2));
            double intersect1_z = ballPath[recordedPos - 2].distance - zDiff * distanceWithPoint1 / twoPointDistance;
            double distanceWithPoint2 = sqrt(pow(ballPath[recordedPos - 2].ballCentre.x - intersect2.x, 2) + pow(ballPath[recordedPos - 2].ballCentre.y - intersect2.y, 2));
            double intersect2_z = ballPath[recordedPos - 2].distance - zDiff * distanceWithPoint2 / twoPointDistance;
            // z-value: pointsOnLine[0].z_interpolation < zPos < pointsOnLine[1].z_interpolation
            cout << "Rod Position: " << zPos << endl;
            if(fabs(intersect1DistanceSum - twoPointDistance) < threshold)
                if(intersect1_z < zPos && zPos < intersect2_z){
                    cout << "Cause by one point to return goal" << endl;
                    result = true;
                }
            else if(fabs(intersect2DistanceSum - twoPointDistance) < threshold)
                if(intersect1_z < zPos && zPos < intersect2_z){
                    cout << "Cause by one point to return goal" << endl;
                    result = true;
                }
        }
    }

    // Put text on displayed image
    if(result == true && recordedPos > 0)
        putText(rawImage, string("Goal"), Point(430, 30), 0, 1, Scalar(0, 127, 255), 2);
    else if(result == false && recordedPos > 0)
        putText(rawImage, string("Fail"), Point(430, 30), 0, 1, Scalar(0, 127, 255), 2);
    delete[] pointsOnLine;
}

void ballFilter(){
    cv::Mat cannyEdge, temp;
    vector<vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;

	if (circleExist == false)
		return;

    // Trim out lower half of image
    for(int j=0; j<imageForBall.cols; j++){
        for(int i=0; i<imageForBall.rows; i++){
            // Assume that the ball must be higher than image centre, change here if using raw 16 bits
            if(i >= imageForBall.rows/2)
                imageForBall.at<IMAGE_FORMAT>(i, j) = 0;
        }
    }

    // Function(sourceImage, destImage, params);
    medianBlur(imageForBall, temp, 2 * medianBlurValue + 1);
    Canny(temp, cannyEdge, cannyLower, cannyUpper);
	cv::imshow("Miedian Blur", temp);
    findContours(cannyEdge, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Draw all contours with filled colour
    int maxContour = 0;
    Scalar color(255, 0, 0);
    for(int i = 0; i < contours.size(); i++){ // Iterate through each contour
        drawContours(rawImage, contours, i, color, CV_FILLED, 8, hierarchy);
        if(i > 0){
            if(contours[i].size() > contours[maxContour].size())
                maxContour = i;
        }
    }

    if(contours.size() > 0){
        vector<RotatedRect> minEllipse(contours.size());
        // Get minimum ellipse of contours
        for(int i = 0; i < contours.size(); i++){
            if(contours[i].size() > 5){
                minEllipse[i] = fitEllipse(Mat(contours[i]));
                cout << "minimum enclosing ellipse: " << minEllipse[i].center << endl;
                cv::line(rawImage, cv::Point(minEllipse[i].center.x - 5, minEllipse[i].center.y), cv::Point(minEllipse[i].center.x + 5, minEllipse[i].center.y), Scalar(255, 255, 0), 2);
                cv::line(rawImage, cv::Point(minEllipse[i].center.x, minEllipse[i].center.y - 5), cv::Point(minEllipse[i].center.x, minEllipse[i].center.y + 5), Scalar(255, 255, 0), 2);
            }
        }
        
        // Draw centre on image
        // cout << "{ " << massCentre[0].x << ", " << massCentre[0].y << " }" << endl;
        // cv::line(rawImage, cv::Point(massCentre[0].x - 5, massCentre[0].y), cv::Point(massCentre[0].x + 5, massCentre[0].y), Scalar(255, 255, 0), 2);
        // cv::line(rawImage, cv::Point(massCentre[0].x, massCentre[0].y - 5), cv::Point(massCentre[0].x, massCentre[0].y + 5), Scalar(255, 255, 0), 2);

        // Record current first point to vector array
		if (minEllipse[maxContour].center.x > 0 && minEllipse[maxContour].center.y > 0) {
			ballPath[recordedPos].ballCentre = minEllipse[maxContour].center;
			ballPath[recordedPos].zDistance = (int)imageForBall.at<IMAGE_FORMAT>(int(minEllipse[maxContour].center.y), int(minEllipse[maxContour].center.x));
			recordedPos += 1;
			detectedBall = 1;
		}
    }
    else{
        detectedBall -= 1;
    }

    // Reset vector array if cannot detect ball in consecutive 5 frames
    if(detectedBall == -4){
        recordedPos = 0;
    }

    // Draw trace line with mutex lock to achieve mutually exclusive
    for(int i=1; i<recordedPos; i++)
        cv::line(rawImage, ballPath[i-1].ballCentre, ballPath[i].ballCentre, Scalar(0, 255, 255), 2);
	cannyEdge.release();
	temp.release();
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

void imageProcessing(cv::Mat rodImage, int lowerColorRange){
    // Create thread to perform two separated tasks
    circleExist = false;
    preFiltering(rawImage, rodImage, lowerColorRange);
	ballFilter();

	if (detectedBall == -3 && recordedPos > 0) {
		cout << "Goal Detection is running" << endl;
		goalDetection();
	}

    zPos = -1;
}

int main(int argc, char** argv){
	int lowerColor, key_pressed;
	int imageNumber = 1;
	kinectInit();

	cv::namedWindow("Depth Map");

	// Pass by reference
	colorCalculation(&lowerColor);

	// Get frame reader
	pFrameSource->OpenReader(&pFrameReader);

	while (true){
		// Create matrix object with same resolution of depth map
		cv::Mat realImage(iHeight, iWidth, CV_16UC1);

		// mDepthImg = 16bit unsigned, mImg8bit = 8bit unsigned
		cv::Mat imageDisplay(iHeight, iWidth, CV_8UC1);

		// Get latest frame and check condition
		IDepthFrame* pFrame = nullptr;

		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK){
			// S_OK = execute successfully
			// E_PENDING = not ready for retrieving data
			// We may add timer to prevent busy waiting in state E_PENDING

			// Copy the depth map to cv::Mat mDepthImg object
			pFrame->CopyFrameDataToArray(iWidth * iHeight,
				reinterpret_cast<UINT16*>(realImage.data));

			// Here we can perform some calculation to finish some tasks
			// ...

			// Convert from 16bit to 8bit
			// unsigned int = 4 bytes, unsigned char = 1 bytes
			realImage.convertTo(imageDisplay, CV_8U, 255.0f / 8000);

			// Change here if using raw 16 bits
			imageDisplay.copyTo(rawImage);
            cvtColor(rawImage, rawImage, COLOR_GRAY2BGR);
            imageDisplay.copyTo(rodImage);
			imageDisplay.copyTo(imageForBall);

			// Create thread to perform two separated tasks
			imageProcessing(rodImage, lowerColor);

			// cv::imshow to display image
			cv::imshow("Depth Map", rawImage);
			// Hashing from depth to colour h(0) = (0, 0, 0)

			// Release frame
			pFrame->Release();
		}

		key_pressed = cv::waitKey(30);
		rawImage.release();
		imageDisplay.release();
		realImage.release();
		rodImage.release();
		imageForBall.release();
	}

	returnKinect();
	locationQueue->deleteAllocation();
    delete locationQueue;
	return 0;
}