// Test.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Server.h"
#include "DepthViewer.h"
#include <iostream>
#include <fstream>
#include <thread>


using namespace std;
using namespace cv;
using namespace Tara;

//Object creation
DepthViewer _DepthViewer;
Server server;
int x=0, y=0, depth=0, foundCircle=0;
bool stop = false;
char WaitKeyStatus;
ofstream disparityTime;
ofstream circleTime;

unsigned long frames = 0, circleFrames = 0;
void findCircles()
{
	while (1)
	{
		if (circleFrames <= frames)
		{
			//auto started = chrono::high_resolution_clock::now();
			_DepthViewer.CircleDetection(&x, &y, &depth, &foundCircle);
			server.sendData(x, y, depth, foundCircle);
			circleFrames++;
			//auto done = chrono::high_resolution_clock::now();
			//circleTime << chrono::duration_cast<chrono::milliseconds>(done - started).count() << "\n";
			
		}
		if (stop)
		{
			break;
		}
	}	
}
/*
void depthCalc()
	{
	while (1)
	{
		auto started = chrono::high_resolution_clock::now();

		_DepthViewer.DisparityCalculations();
		//waitKey(1);
		if (stop)
		{
			break;
		}
		auto done = chrono::high_resolution_clock::now();

		disparityTime << chrono::duration_cast<chrono::milliseconds>(done - started).count() << "\n";
	}
}

void sendData()
{
	while(1)
	{
		server.sendData(x, y, depth, foundCircle);
		Sleep(15);
		if (stop)
		{
			break;
		}
	}
}
*/
//Main Application
int main()
{
	circleTime.open("circle.txt");
	disparityTime.open("disparity.txt");
	cout << "waiting for unity program....." << endl;
	server.initialiseServer();
	_DepthViewer.init();
	thread first(findCircles);
	//thread second(depthCalc);
	//thread third(sendData);
	while (1)
	{
		auto started = chrono::high_resolution_clock::now();
		_DepthViewer.DisparityCalculations(&frames);
		WaitKeyStatus = waitKey(1);
		if (WaitKeyStatus == 'q' || WaitKeyStatus == 'Q' || WaitKeyStatus == 27) //Quit
		{
			destroyAllWindows();
			cout << "q pressed" << endl;
			stop = true;
			break;
		}

		auto done = chrono::high_resolution_clock::now();
		disparityTime << chrono::duration_cast<chrono::milliseconds>(done - started).count() << "\n";
	}
	
	server.closeServer();
	cout << "closed server" << endl;
	first.join();
	//second.join();
	//third.join();
	cout << "all threads joined" << endl;
	circleTime.close();
	disparityTime.close();
	return 1;
}

