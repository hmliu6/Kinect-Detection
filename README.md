## Kinect Detection

This project is developed under Windows 64bit environment. Probably able to convert to Linux environment but needs testing.

### Dependencies

* Windows 10 64bit
* Kinect for Windows SDK v2
* OpenCV 3.3.1
* Kinect v2 connected to **USB 3.0**

## Installation

### Kinect for Windows SDK v2

Download and install from Microsoft webpage and link up to VS project

1. Right click project and choose **Properties**
2. C/C++ &rarr; General &rarr; Additional Include Directories
3. Add `$(KINECTSDK20_DIR)\inc` inside `Additional Include Directories`
4. Linker &rarr; General &rarr; Additional Library Directories
5. Add `kinect20.lib` inside `Additional Library Directories`
6. Linker &rarr; Input &rarr; Additional Directories
7. Add `kinect20.lib` inside `Additional Directories`

Check there exists any error when `#include <kinect.h>`

Reference : ![Kinect for Windows SDK v2 C++ API](https://kheresy.wordpress.com/2015/01/06/k4w-sdk-v2-cpp-api-intro/)

### OpenCV 3.3.1

Download version 3.3.1 from ![OpenCV official website](https://opencv.org/releases.html)

Extract the executable file and place it any path, here denotes as ${OPENCV_PATH}

Reference: ![OpenCV installation and settings for Visual Studio](http://oblivious9.pixnet.net/blog/post/200316565-opencv-%E5%AE%89%E8%A3%9D%E5%92%8C%E8%A8%AD%E5%AE%9A%28for-visual-studio-%29)

## Credit

* ![Kinect for Windows v2 C++ Development](https://kheresy.wordpress.com/kinect-for-windows-v2-cpp-index/)
* ![KHeresy/KinectForWindows2Sample](https://github.com/KHeresy/KinectForWindows2Sample)
* ![UnaNancyOwen/Kinect2Sample](https://github.com/UnaNancyOwen/Kinect2Sample)
* ![OpenCV 3.3.1 Documentation](https://docs.opencv.org/3.3.1/)
* ![OpenCV findContours](http://monkeycoding.com/?p=615)

## Arthur

If you have any problem, please contact below developers

* Liu Ho Man(hmliu6@gmail.com)

Created at 2017/12/25