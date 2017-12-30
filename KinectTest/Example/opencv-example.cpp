

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main()
{
	char fileName[] = "C:\\\\robocon\\test-image\\index.jpg";
	cout << fileName << endl;
	IplImage *rawImage;
	rawImage = cvLoadImage(fileName, CV_LOAD_IMAGE_UNCHANGED);

	if (!rawImage)
		cout << "Cannot find document!!!" << endl;
	else
	{	
		IplImage *image = cvtColor(rawImage, cv2.COLOR_BGR2GRAY);
		cvShowImage("Test", image);
		cvWaitKey(0);
	}

	system("pause");
	return 0;
}