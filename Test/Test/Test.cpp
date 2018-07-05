// Test.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Server.h"
#include "DepthViewer.h"
#include <iostream>
#include <thread>


using namespace std;
using namespace cv;
using namespace Tara;

//Object creation
DepthViewer _DepthViewer;
Server server;
int x=0, y=0, depth=0;

void depthCalc()
{
	_DepthViewer.DisparityCalculations(&x, &y, &depth);
	
}

void findCircles()
{
	_DepthViewer.CircleDetection();
}

void sendData()
{
	while(1)
	{
		server.sendData(x, y, depth);
		Sleep(100);
	}
	
}
//Main Application
int main()
{
	server.initialiseServer();
	_DepthViewer.init();
	thread first(findCircles);
	thread second(sendData);
	_DepthViewer.DisparityCalculations(&x, &y, &depth);
	//_DepthViewer.CameraStreaming();
	return 1;
}

