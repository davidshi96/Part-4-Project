/**********************************************************************
 DepthViewer: Displays the depth of the point selected by the user
			   using the disparity image computed.
**********************************************************************/



#include "stdafx.h"
#include "DepthViewer.h"
#include <math.h>
#include <chrono>



//Local point to access the user selected value
Point g_SelectedPoint(-1, -1);

void DepthViewer::init()
{
	if (!_Disparity.InitCamera(true, true))
	{
		return;
	}

	_Disparity.GrabFrame(&LeftImage, &RightImage);
	//Window Creation
	//namedWindow("Left Image", WINDOW_AUTOSIZE);
	//namedWindow("Right Image", WINDOW_AUTOSIZE);
	namedWindow("Disparity Map", WINDOW_AUTOSIZE);
	//setMouseCallback("Disparity Map", DepthPointSelection);

	//setting up camera modes
	_Disparity.SetStreamMode(MASTERMODE);
	//_Disparity.SetAutoExposure();
	_Disparity.SetBrightness(5);
	
}



void DepthViewer::DisparityCalculations(int *X, int *Y, int *DEPTH) 
{
	
	Mat gDisparityMap, gDisparityMap_viz;

	int prevDepth = 0;
	/*
	//setup for my own disparity filter
	cv::Ptr<cv::StereoBM> bm_left;
	cv::Ptr<cv::StereoMatcher> bm_right;
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

	cv::Mat left_disp, right_disp;
	cv::Mat LeftScaleImage, RightScaleImage;
	cv::Mat filtered_disp, raw_disp_vis, filtered_disp_vis;

	bm_left = StereoBM::create(16, 15);
	wls_filter = createDisparityWLSFilter(bm_left);
	bm_right = StereoBM::create(16, 15);
	*/
	
	while (1)
	{
		//auto started = std::chrono::high_resolution_clock::now();

		//Get disparity
		if (!_Disparity.GrabFrame(&LeftImage, &RightImage)) //Reads the frame and returns the rectified image
		{
			destroyAllWindows();
			break;
		}
		_Disparity.GetDisparity(LeftImage, RightImage, &gDisparityMap, &gDisparityMap_viz);
		if (XMiddle == 0 && YMiddle == 0)
		{
			XMiddle = gDisparityMap_viz.cols / 2;
			YMiddle = gDisparityMap_viz.rows / 2;
		}
		line(gDisparityMap_viz, Point(0, YMiddle), Point(gDisparityMap_viz.cols, YMiddle), Scalar(0, 0, 0), 1);
		line(gDisparityMap_viz, Point(XMiddle, 0), Point(XMiddle, gDisparityMap_viz.rows), Scalar(0, 0, 0), 1);

		/*
		//resizing image
		resize(LeftImage, LeftScaleImage, cv::Size(), 0.5, 0.5, INTER_AREA);
		resize(RightImage, RightScaleImage, cv::Size(), 0.5, 0.5, INTER_AREA);
		
		//performing matching
		bm_left->compute(LeftScaleImage, RightScaleImage, left_disp);
		bm_right->compute(RightScaleImage, LeftScaleImage, right_disp);
		
		//performing filtering
		wls_filter->setLambda(8000.00);
		wls_filter->setSigmaColor(1.5);
		wls_filter->filter(left_disp, LeftImage, filtered_disp, right_disp);

		getDisparityVis(filtered_disp, filtered_disp_vis, 5.0);
		*/

		// Estimate the Depth of the point selected
		_Disparity.EstimateDepth(g_SelectedPoint, &DepthValue);
			

		if(DepthValue > 0)
		{
			if (prevDepth == 0)
			{
				prevDepth = round(DepthValue);
			}

			if (circlesFound == true)
			{
				depth = round(DepthValue);
				prevDepth = round(DepthValue);

				/*
				if ((DepthValue < 1.25*prevDepth) && (DepthValue > 0.75*prevDepth))
				{
					depth = round(DepthValue);
					prevDepth = round(DepthValue);
				}
				else
				{
					depth = prevDepth;
				}
				*/
				stringstream ss;
				ss << "depth = " + to_string(depth) + " X = " + to_string(XDist) + " Y = " + to_string(YDist) << " mm\0";
				cv::circle(gDisparityMap_viz, g_SelectedPoint, 3, Scalar::all(0), 3, 8);
				putText(gDisparityMap_viz, ss.str(), g_SelectedPoint, 2, 0.5, Scalar(0, 0, 0), 2, 8, false);
				*X = XDist;
				*Y = YDist;
				*DEPTH = depth;
			}
			else
			{
				depth = prevDepth;
			}
			//cout << "Actual Depth =  " + to_string(DepthValue) + " prevDepth = " + to_string(prevDepth) << endl;
		}
		
		
		//Display the Images
		imshow("Disparity Map", gDisparityMap_viz);
		waitKey(1);
		//auto done = std::chrono::high_resolution_clock::now();
		//std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done - started).count() << std::endl;
	}

}

void DepthViewer::CircleDetection() 
{
	vector<Vec3f> circles;
	Mat imageToProcess;
	double scale;
	int XPix = 0;
	int YPix = 0;
	while (1)
	{

		if (!LeftImage.empty() && !RightImage.empty())
		{
			//auto started = std::chrono::high_resolution_clock::now();

			//GaussianBlur(LeftImage, imageToProcess, Size(9, 1), 2, 2);
			//GaussianBlur(imageToProcess, imageToProcess, Size(1, 9), 2, 2);
			blur(LeftImage, imageToProcess, Size(7, 7), Point(-1, -1), BORDER_DEFAULT);

			// Apply the Hough Transform to find the circles
			HoughCircles(imageToProcess, circles, CV_HOUGH_GRADIENT, 1, imageToProcess.rows / 2, 100, 50, 5, 200);
			// cout << to_string(LeftImage.rows) + "    " + to_string(LeftImage.cols) << endl;
			if (circles.size() > 0) {
				if ((cvRound(circles[0][0]) > 0) && (cvRound(circles[0][1]) > 0))
				{
					circlesFound = true;
					g_SelectedPoint = Point(cvRound(circles[0][0]), cvRound(circles[0][1]));
					YPix = -(cvRound(circles[0][1]) - YMiddle);
					XPix = cvRound(circles[0][0]) - XMiddle;

					//cout << "Y AXIS = " + to_string(YPix) + " X AXIS = " + to_string(XPix) << endl;

					scale = exp(log(DepthValue)*-0.9932 + 6.5735);

					YDist = YPix / scale;
					XDist = XPix / scale;

				}

				//cout << "center of detected circle = " + to_string(cvRound(circles[0][0])) + " , " + to_string(cvRound(circles[0][1])) +
				//	"  Y AXIS = " + to_string(YDist) + " X AXIS = " + to_string(XDist) << endl;
			}
			else
			{
				circlesFound = false;
			}
			circles.clear();
			//cout <<"Y AXIS = " + to_string(YPix) + " X AXIS = " + to_string(XPix) << endl;
			//auto done = std::chrono::high_resolution_clock::now();
			//std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done - started).count() << std::endl;
		}
	}
	
}


//Streams the input from the camera
void DepthViewer::CameraStreaming()
{
	Mat left, right;
	while (1)
	{
		//auto started = std::chrono::high_resolution_clock::now();
		if (!_Disparity.GrabFrame(&LeftImage, &RightImage)) //Reads the frame and returns the rectified image
		{
			destroyAllWindows();
			break;
		} 
		//flip(left, LeftImage, 1);
		//flip(right, RightImage, 1);
		//Display the Images
		imshow("Left Image", LeftImage);
		imshow("Right Image", RightImage);
		waitKey(1);
		//auto done = std::chrono::high_resolution_clock::now();
		//std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done - started).count() << std::endl;
	}
}

/*
//Call back function
void DepthPointSelection(int MouseEvent, int x, int y, int flags, void* param)
{
	if (MouseEvent == CV_EVENT_LBUTTONDOWN)  //Clicked
	{
		g_SelectedPoint = Point(x, y);
	}
}
*/