// Test.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "DepthViewer.h"
#include "OpenCVViewer.h"
#include <opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;
using namespace Tara;

/*
int main()
{
	Mat img = imread("bae.jpg");
	namedWindow("image", WINDOW_NORMAL);
	imshow("image", img);
	waitKey(0);
	return 0;
}*/

//Main Application
int main()
{
	
	//PrintDebug(DEBUG_ENABLED, L"Depth Viewer");
	int ReturnStatus = -1;

	//Object creation
	DepthViewer _DepthViewer;

	//Initialises the depth view
	ReturnStatus = _DepthViewer.Init();

	//Object to access the application to view the stereo images  
	//OpenCVViewer _OpenCVViewer;

	//Initialises to run the viewer
	//ReturnStatus = _OpenCVViewer.Init();

	if (!ReturnStatus) //check for a valid return
	{
		cout << endl << "Press any key to exit!" << endl << endl;
		_getch();
	}
	return 0;
}

