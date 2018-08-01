#pragma once

#include "Tara.h"
#include <math.h>
#include <chrono>

using namespace Tara;
using namespace cv;
using namespace cv::ximgproc;

#define SEE3CAM_STEREO_AUTO_EXPOSURE		1

class DepthViewer
{
public:

	void CameraStreaming();
	void CircleDetection(int *X, int *Y, int *DEPTH, int *foundCircle);
	void DisparityCalculations();
	void init();

private:

	int ManualExposure;

	//Object to access Disparity
	Disparity _Disparity;	

	cv::Mat LeftImage, RightImage;
	int depth = 0, XDist = 0, YDist = 0;
	int prevX = 0, prevY = 0;
	float DepthValue = 0;
	int XMiddle = 0, YMiddle = 0;
	bool circlesFound = false;

	//disparity calc
	Mat LDisp, RDisp;
	Mat gDisparityMap, gDisparityMap_viz;

	int prevDepth = 0;

	//circle detect
	vector<Vec3f> circles;
	Mat imageToProcess, scaledImg;
	double scale = 1;
	int XPix = 0;
	int YPix = 0;

	float imgScale = .6666667;

	long frames = 0;

	SOCKET clientSocket;

};