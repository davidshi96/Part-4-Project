/**********************************************************************
 DepthViewer: Displays the depth of the point selected by the user
			   using the disparity image computed.
**********************************************************************/

#include "stdafx.h"
#include "DepthViewer.h"
#include <chrono>

//Local point to access the user selected value
Point g_SelectedPoint(-1, -1);
Mat LeftImage, RightImage;

void DepthViewer::init()
{
	if (!_Disparity.InitCamera(true, true))
	{
		return;
	}
	_Disparity.GrabFrame(&LeftImage, &RightImage);
	//Window Creation
	namedWindow("Left Image", WINDOW_AUTOSIZE);
	namedWindow("Right Image", WINDOW_AUTOSIZE);
	namedWindow("Disparity Map", WINDOW_AUTOSIZE);

	cout << endl << "Press q/Q/Esc on the Image Window to quit the application!" << endl;
	cout << endl << "Press b/B on the Image Window to change the brightness of the camera" << endl;
	cout << endl << "Press t/T on the Image Window to change to Trigger Mode" << endl;
	cout << endl << "Press m/M on the Image Window to change to Master Mode" << endl;
	cout << endl << "Press a/A on the Image Window to change to Auto exposure  of the camera" << endl;
	cout << endl << "Press e/E on the Image Window to change the exposure of the camera" << endl;
	setMouseCallback("Disparity Map", DepthPointSelection);

}



void DepthViewer::DisparityCalculations() {

	float DepthValue = 0;
	Mat gDisparityMap, gDisparityMap_viz;
	//Mouse callback set to disparity window
	while (1)
	{
		auto started = std::chrono::high_resolution_clock::now();

		//Get disparity
		_Disparity.GetDisparity(LeftImage, RightImage, &gDisparityMap, &gDisparityMap_viz);
		
		//Estimate the Depth of the point selected
		_Disparity.EstimateDepth(g_SelectedPoint, &DepthValue);

		if(g_SelectedPoint.x > -1 && DepthValue > 0) //Mark the point selected by the user
		circle(gDisparityMap_viz, g_SelectedPoint, 3, Scalar::all(0), 3, 8);

		if(DepthValue > 0)
		{
		stringstream ss;
		ss << DepthValue / 10 << " cm\0" ;
		//DisplayText(gDisparityMap_viz, ss.str(), g_SelectedPoint);
		cout << ss.str() << endl;
		}
		
		//Display the Images
		imshow("Disparity Map", gDisparityMap_viz);
		waitKey(1);
		auto done = std::chrono::high_resolution_clock::now();
		std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done - started).count() << std::endl;
	}

}



//Streams the input from the camera
void DepthViewer::CameraStreaming()
{
	cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	string Inputline;
	char WaitKeyStatus;
	int BrightnessVal = 4;		//Default value

	//Dispalys the filtered disparity, the depth of the point selected is displayed
	while (1)
	{
		if (!_Disparity.GrabFrame(&LeftImage, &RightImage)) //Reads the frame and returns the rectified image
		{
			destroyAllWindows();
			break;
		}

		//Display the Images
		imshow("Left Image", LeftImage);
		imshow("Right Image", RightImage);

		//waits for the Key input
		WaitKeyStatus = waitKey(1);
		if (WaitKeyStatus == 'q' || WaitKeyStatus == 'Q' || WaitKeyStatus == 27) //Quit
		{
			destroyAllWindows();
			break;
		}
		//Sets up the mode
		else if (WaitKeyStatus == 'T' || WaitKeyStatus == 't') //Stream Mode 0 - Trigger Mode 1 - Master Mode
		{
			if (_Disparity.SetStreamMode(TRIGGERMODE))
			{
				cout << endl << "Switching to Trigger Mode!!" << endl;
			}
			else
			{
				cout << endl << "Selected mode and the current mode is the same!" << endl;
			}
		}

		//Sets up the mode
		else if (WaitKeyStatus == 'M' || WaitKeyStatus == 'm') //Stream Mode 0 - Trigger Mode 1 - Master Mode
		{
			if (_Disparity.SetStreamMode(MASTERMODE))
			{
				cout << endl << "Switching to Manual Mode!!" << endl;
			}
			else
			{
				cout << endl << "Selected mode and the current mode is the same!" << endl;
			}
		}
		//Sets up Auto Exposure
		else if (WaitKeyStatus == 'a' || WaitKeyStatus == 'A') //Auto Exposure
		{
			_Disparity.SetAutoExposure();
		}
		else if (WaitKeyStatus == 'e' || WaitKeyStatus == 'E') //Set Exposure
		{
			cout << endl << "Enter the Exposure Value Range(10 to 1000000 micro seconds): " << endl;

			ManualExposure = 0;

			while (getline(std::cin, Inputline)) //To avoid floats and Alphanumeric strings
			{
				std::stringstream ss(Inputline);
				if (ss >> ManualExposure)
				{
					if (ss.eof())
					{
						if (ManualExposure >= SEE3CAM_STEREO_EXPOSURE_MIN && ManualExposure <= SEE3CAM_STEREO_EXPOSURE_MAX)
						{
							//Setting up the exposure
							_Disparity.SetExposure(ManualExposure);
						}
						else
						{
							cout << endl << " Value out of Range - Invalid!!" << endl;
						}
						break;
					}
				}
				ManualExposure = -1;
				break;
			}

			if (ManualExposure == -1)
			{
				cout << endl << " Value out of Range - Invalid!!" << endl;
			}
		}
		else if (WaitKeyStatus == 'b' || WaitKeyStatus == 'B') //Brightness
		{
			cout << endl << "Enter the Brightness Value, Range(1 to 7): " << endl;

			BrightnessVal = 0;

			while (getline(std::cin, Inputline)) //To avoid floats and Alphanumeric strings
			{
				std::stringstream ss(Inputline);
				if (ss >> BrightnessVal)
				{
					if (ss.eof())
					{
						//Setting up the brightness of the camera
						if (BrightnessVal >= 1 && BrightnessVal <= 7)
						{
							//Setting up the exposure
							_Disparity.SetBrightness(BrightnessVal);
						}
						else
						{
							cout << endl << " Value out of Range - Invalid!!" << endl;
						}
						break;
					}
				}
				BrightnessVal = -1;
				break;
			}

			if (BrightnessVal == -1)
			{
				cout << endl << " Value out of Range - Invalid!!" << endl;
			}
		}
	}
}

//Call back function
void DepthPointSelection(int MouseEvent, int x, int y, int flags, void* param)
{
	if (MouseEvent == CV_EVENT_LBUTTONDOWN)  //Clicked
	{
		g_SelectedPoint = Point(x, y);
	}
}
