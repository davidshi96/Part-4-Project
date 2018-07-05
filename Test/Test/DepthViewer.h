#pragma once

#include "Tara.h"

using namespace Tara;
using namespace cv;
using namespace cv::ximgproc;

#define SEE3CAM_STEREO_AUTO_EXPOSURE		1

class DepthViewer
{
public:

	void CameraStreaming();
	void CircleDetection();
	void DisparityCalculations(int *X, int *Y, int *DEPTH);
	void init();

private:

	int ManualExposure;

	//Object to access Disparity
	Disparity _Disparity;	

	cv::Mat LeftImage, RightImage;
	int depth, XDist, YDist;
	int prevX, prevY;
	float DepthValue = 0;
	int XMiddle = 0, YMiddle = 0;
	bool circlesFound;
};