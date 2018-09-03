/**********************************************************************
 DepthViewer: Displays the depth of the point selected by the user
			   using the disparity image computed.
**********************************************************************/



#include "stdafx.h"
#include "DepthViewer.h"


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
	//namedWindow("Right Image", WINDOW_AUTOSIZE);
	namedWindow("Disparity Map", WINDOW_AUTOSIZE);
	//setMouseCallback("Disparity Map", DepthPointSelection);

	//setting up camera modes
	_Disparity.SetStreamMode(MASTERMODE);
	//_Disparity.SetAutoExposure();
	_Disparity.SetBrightness(5);
	
}



void DepthViewer::DisparityCalculations(unsigned long *frames) 
{
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

	//Get disparity
	if (_Disparity.GrabFrame(&LeftImage, &RightImage))
	{
		frame = *frames + 1;
		*frames = frame;
		//imshow("Left Image", LeftImage);
		//imshow("Right Image", RightImage);

		//scale left and right images, it should rescale the disparity map back into its original size. 
		cv::resize(LeftImage, LDisp, cv::Size(), imgScale, imgScale);
		cv::resize(RightImage, RDisp, cv::Size(), imgScale, imgScale);
		_Disparity.GetDisparity(LDisp, RDisp, &gDisparityMap, &gDisparityMap_viz);
		/*
		//resizing image
		resize(LeftImage, LeftScaleImage, cv::Size(), 0.25, 0.25, INTER_AREA);
		resize(RightImage, RightScaleImage, cv::Size(), 0.25, 0.25, INTER_AREA);

		//performing matching
		bm_left->compute(LeftScaleImage, RightScaleImage, left_disp);
		bm_right->compute(RightScaleImage, LeftScaleImage, right_disp);

		//performing filtering
		wls_filter->setLambda(8000.00);
		wls_filter->setSigmaColor(1.5);
		wls_filter->filter(left_disp, LeftImage, gDisparityMap, right_disp);

		getDisparityVis(gDisparityMap, gDisparityMap_viz, 5.0);
 
		line(gDisparityMap_viz, Point(0, YMiddle), Point(gDisparityMap_viz.cols, YMiddle), Scalar(0, 0, 0), 1);
		line(gDisparityMap_viz, Point(XMiddle, 0), Point(XMiddle, gDisparityMap_viz.rows), Scalar(0, 0, 0), 1);

		*/
		if (circlesFound)
		{
			stringstream ss;
			ss << "X = " + to_string(XDist) + " Y = " + to_string(YDist) + " depth = " + to_string(depth) << " mm\0";
			//cv::circle(gDisparityMap_viz, g_SelectedPoint, 3, Scalar::all(0), 3, 8);
			circle(gDisparityMap_viz, g_SelectedPoint, pixRadius, Scalar(0, 0, 255), 3, 8, 0);
			putText(gDisparityMap_viz, ss.str(), g_SelectedPoint, 2, 0.5, Scalar(0, 0, 0), 2, 8, false);
		}
		
	}
	cv::imshow("Disparity Map", gDisparityMap_viz);
}

void DepthViewer::CircleDetection(int *X, int *Y, int *DEPTH, int *foundCircle, int *rad)
{
	
	if (XMiddle == 0 && YMiddle == 0)
	{
		XMiddle = gDisparityMap_viz.cols / 2;
		YMiddle = gDisparityMap_viz.rows / 2;
	}
	imshow("Left Image", LeftImage);
	if (!LeftImage.empty() && !RightImage.empty())
	{
		
		//GaussianBlur(LeftImage, imageToProcess, Size(5, 5), 2, 2);

		//blur(LeftImage, imageToProcess, Size(3, 3), Point(-1, -1), BORDER_DEFAULT);
		
		circles.clear();
		//scale image down
		cv::resize(LeftImage, imageToProcess, cv::Size(), imgScale, imgScale, INTER_CUBIC);

		// Apply the Hough Transform to find the circles
		HoughCircles(imageToProcess, circles, CV_HOUGH_GRADIENT, 1, imageToProcess.cols, 120, 60); //static_cast<int>(10*imgScale), static_cast<int>(240*imgScale)

		if (circles.size() > 0) 
		{
			if ((cvRound(circles[0][0]/ imgScale) > 160) && (cvRound(circles[0][1]/ imgScale) > 0))
			{
				circlesFound = true;
				g_SelectedPoint = Point(cvRound(circles[0][0]/ imgScale), cvRound(circles[0][1]/ imgScale));
				_Disparity.EstimateDepth(g_SelectedPoint, &DepthValue);
				DepthValue = DepthValue * imgScale;

				//once we have the depth value, we can then use that and the radius to find the actual diameter of the ball.

				//getting the radius of the ball in pixels
				pixRadius = cvRound(circles[0][2]/ imgScale);
				//cout << "pixel radius = " + to_string(pixRadius) + "depth value = " + to_string(DepthValue) << endl;
				//converting pixels to mm
				//calculating actual radius using iterative feedback

				radius = (0.0013f*DepthValue + 0.0073f)*pixRadius / (1 - 0.0013f*pixRadius);

				if (DepthValue > 0 && DepthValue < 10000)
				{
					if (prevDepth == 0)
					{
						prevDepth = static_cast<int>(DepthValue) + static_cast<int>(radius);
					}
				
					depth = static_cast<int>(DepthValue) + static_cast<int>(radius);
					prevDepth = static_cast<int>(DepthValue) + static_cast<int>(radius);
					
					YPix = -(cvRound(circles[0][1]) - YMiddle);
					XPix = cvRound(circles[0][0]) - XMiddle;

					scale = 0.0013*depth + 0.0073;
					YDist = static_cast<int>(YPix * scale);
					XDist = static_cast<int>(XPix * scale);


					*X = XDist;
					*Y = YDist;
					*DEPTH = depth;
					*foundCircle = 1;
					*rad = static_cast<int>(radius);
					
				}
			}
		}
		else
		{
			circlesFound = false;
			depth = prevDepth;
			*foundCircle = 0;
		}
		
	}
	
}

//Streams the input from the camera
void DepthViewer::CameraStreaming()
{
	_Disparity.GrabFrame(&LeftImage, &RightImage); //Reads the frame and returns the rectified image

	//Display the Images
	//imshow("Left Image", LeftImage);
	//imshow("Right Image", RightImage);
}