// Standard Library
#include <iostream>

// Kinect for Windows SDK Header
#include <Kinect.h>

int main(int argc, char** argv)
{
	// 1a. Get default Sensor
	IKinectSensor* pSensor = nullptr;
	GetDefaultKinectSensor(&pSensor);

	// 1b. Open sensor
	pSensor->Open();

	// 2a. Get frame source
	IDepthFrameSource* pFrameSource = nullptr;
	pSensor->get_DepthFrameSource(&pFrameSource);

	// 3a. get frame reader
	IDepthFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);

	// Enter main loop
	size_t uFrameCount = 0;
	while (uFrameCount < 100)
	{
		// 4a. Get last frame
		IDepthFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			// 4b. Get frame description
			int        iWidth = 0;
			int        iHeight = 0;
			IFrameDescription* pFrameDescription = nullptr;
			pFrame->get_FrameDescription(&pFrameDescription);
			pFrameDescription->get_Width(&iWidth);
			pFrameDescription->get_Height(&iHeight);
			pFrameDescription->Release();
			pFrameDescription = nullptr;

			// 4c. Get image buffer
			UINT    uBufferSize = 0;
			UINT16*    pBuffer = nullptr;
			pFrame->AccessUnderlyingBuffer(&uBufferSize, &pBuffer);

			// 4d. Output depth value
			int x = iWidth / 2,
				y = iHeight / 2;
			size_t idx = x + iWidth * y;
			std::cout << pBuffer[idx] << std::endl;

			// 4e. release frame
			pFrame->Release();
			pFrame = nullptr;

			++uFrameCount;
		}
	}

	// 3b. release frame reader
	pFrameReader->Release();
	pFrameReader = nullptr;

	// 2b. release Frame source
	pFrameSource->Release();
	pFrameSource = nullptr;

	// 1c. Close Sensor
	pSensor->Close();

	// 1d. Release Sensor
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}