// Test.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "DepthViewer.h"
#include <opencv.hpp>
#include <iostream>
#include <thread>

using namespace std;
using namespace cv;
using namespace Tara;

//Object creation
DepthViewer _DepthViewer;

void depthCalc()
{
	_DepthViewer.DisparityCalculations();
}

//Main Application
int main()
{
	int ReturnStatus = -1;
	_DepthViewer.init();
	thread first(depthCalc);
	_DepthViewer.CameraStreaming();

	return 1;
}

