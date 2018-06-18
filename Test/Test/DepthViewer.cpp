/**********************************************************************
 DepthViewer: Displays the depth of the point selected by the user
			   using the disparity image computed.
**********************************************************************/

#include "stdafx.h"
#include "DepthViewer.h"
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
	namedWindow("Left Image", WINDOW_AUTOSIZE);
	namedWindow("Right Image", WINDOW_AUTOSIZE);
	namedWindow("Disparity Map", WINDOW_AUTOSIZE);
	setMouseCallback("Disparity Map", DepthPointSelection);

	//setting up camera modes
	_Disparity.SetStreamMode(MASTERMODE);
	_Disparity.SetAutoExposure();
	_Disparity.SetBrightness(5);



}



void DepthViewer::DisparityCalculations() 
{
	float DepthValue = 0;
	Mat gDisparityMap, gDisparityMap_viz;

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
		_Disparity.GetDisparity(LeftImage, RightImage, &gDisparityMap, &gDisparityMap_viz);
		
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
		/*
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
		*/
		
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
	while (1)
	{
		GaussianBlur(LeftImage, imageToProcess, Size(9, 9), 2, 2);
		// Apply the Hough Transform to find the circles
		HoughCircles(imageToProcess, circles, CV_HOUGH_GRADIENT, 1, imageToProcess.rows / 8, 130, 65);

		for (size_t i = 0; i < circles.size(); i++) {
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			// circle center
			circle(LeftImage, center, 3, Scalar(255, 255, 0), -1, 8, 0);
		}
		imshow("Left Image", LeftImage);
		waitKey(1);
	}
	
}


//Streams the input from the camera
void DepthViewer::CameraStreaming()
{
	
	while (1)
	{
		//auto started = std::chrono::high_resolution_clock::now();
		if (!_Disparity.GrabFrame(&LeftImage, &RightImage)) //Reads the frame and returns the rectified image
		{
			destroyAllWindows();
			break;
		}

		//Display the Images
		//imshow("Left Image", LeftImage);
		imshow("Right Image", RightImage);
		waitKey(1);
		//auto done = std::chrono::high_resolution_clock::now();
		//std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done - started).count() << std::endl;
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
