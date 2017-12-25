

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main()
{
	char fileName[] = "C:\\\\robocon\\test-image\\index.jpg";
	cout << fileName << endl;
	IplImage *image;
	image = cvLoadImage(fileName, CV_LOAD_IMAGE_UNCHANGED);

	if (!image)
		cout << "cannot find document!!!" << endl;
	else
	{
		cvShowImage("Test", image);
		cvWaitKey(0);
	}

	system("pause");
	return 0;
}