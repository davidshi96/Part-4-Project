///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016, e-Con Systems.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS.
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT/INDIRECT DAMAGES HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
////////////////////////  ///////////////////////////////////////////////////

/**********************************************************************
	Tara.cpp : Defines the exported functions for the DLL application.
	Disparity: Defines the methods to compute disparity map, estimate
				the depth of a point selected.
	TaraCamParameters: Defines the methods to load the callibrated
				file, rectify images
	CameraEnumeration : Enumerates the camera devices connected to the
				system and gives the resolutions supported.
**********************************************************************/
#include "Tara.h"

#define DEFAULT_FRAME_WIDTH			640
#define DEFAULT_FRAME_HEIGHT		480

namespace Tara
{
//Constructor
TaraCamParameters::TaraCamParameters(void)
{
	//Default resolution(higher) used in case of remap
	gImageWidth  = 752;
	gImageHeight = 480;
}

//Destructor
TaraCamParameters::~TaraCamParameters(void)
{
	gImageWidth  = -1;
	gImageHeight = -1;
}

//Constructor
BOOL TaraCamParameters::Init()
{
	//Loads all the matrix related to the camera
	return LoadCameraMatrix();
}

//Loading the camera param
BOOL TaraCamParameters::LoadCameraMatrix()
{
	unsigned char *IntrinsicBuffer, *ExtrinsicBuffer;
	int LengthIntrinsic, LengthExtrinsic;

	//Read the data from the flash
	if(StereoCalibRead(&IntrinsicBuffer, &ExtrinsicBuffer, &LengthIntrinsic, &LengthExtrinsic))
	{
		PrintDebug(DEBUG_ENABLED, L"Read Intrinsic and Extrinsic Files!!");
		printf("\nRead Intrinsic and Extrinsic Files!!\n");
	}
	else
	{
		PrintDebug(DEBUG_ENABLED, L"Failed Reading Intrinsic and Extrinsic Files");
		printf("\nFailed Reading Intrinsic and Extrinsic Files\n");
		return 0;
	}
	FILE *IntFile, *ExtFile;

	IntFile = fopen("intrinsics.yml", "wb");
	ExtFile = fopen("extrinsics.yml", "wb");

	if(IntFile == NULL || ExtFile == NULL)
	{
		PrintDebug(DEBUG_ENABLED, L"Failed Opening Intrinsic and Extrinsic Files");
		printf("\nFailed Opening Intrinsic and Extrinsic Files\n");
		return 0;
	}

	if(LengthIntrinsic < 0 || LengthExtrinsic < 0)
	{
		PrintDebug(DEBUG_ENABLED, L"Invalid Intrinsic and Extrinsic File Length");
		printf("\nInvalid Intrinsic and Extrinsic File Length\n");
		return 0;
	}

	fwrite(IntrinsicBuffer, 1, LengthIntrinsic, IntFile);
	fwrite(ExtrinsicBuffer, 1, LengthExtrinsic, ExtFile);

	fclose(IntFile);
	fclose(ExtFile);

	const char* intrinsic_filename = "intrinsics.yml";
	const char* extrinsic_filename = "extrinsics.yml";

	//reading intrinsic parameters
    cv::FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        PrintDebug(DEBUG_ENABLED, L"Failed Loading Intrinsic Data");
		return 0;
    }

	cv::Mat D1_301, D2_301;

    fs["M1"] >> M1;
    fs["D1"] >> D1_301;
    fs["M2"] >> M2;
    fs["D2"] >> D2_301;

	//To avoid version issues of OpenCV
	GetMatforCV(D1_301, &D1);
	GetMatforCV(D2_301, &D2);

	// reading Extrinsic parameters
    fs.open(extrinsic_filename, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
         PrintDebug(DEBUG_ENABLED, L"Failed Loading Extrinsic Data");
        return 0;
    }

    fs["R"] >> R;
    fs["T"] >> T;

	//Computes the Q Matrix from the Files loaded
	ComputeRectifyPrams();

	PrintDebug(DEBUG_ENABLED, L"Loaded Extrinsic and Intrinsic files..!!!");

	return true;
}

//to support lower version of OpenCV
BOOL TaraCamParameters::GetMatforCV(cv::Mat Src, cv::Mat *Dest)
{
	if(Src.empty())
	{
		PrintDebug(DEBUG_ENABLED, L"Input Matrix is empty!");
		return 0;
	}
	if(CV_MAJOR_VERSION == 3)
	{
		if(CV_MINOR_VERSION == 0)
		{
			PrintDebug(DEBUG_ENABLED, L"OpenCV Major Version : %d ,Minor Version : %d \n", CV_MAJOR_VERSION,CV_MINOR_VERSION);

			const int Maxval=12;
			*Dest = cv::Mat(Src.rows, Maxval, Src.type());
			for (int Col = 0; Col < Maxval; Col++)
			{
				Dest->at<double>(0, Col) = Src.at<double>(0, Col);
			}
		}
		else
		{
			PrintDebug(DEBUG_ENABLED, L"301 No changes needed! OpenCV Major Version : %d ,Minor Version : %d \n", CV_MAJOR_VERSION,CV_MINOR_VERSION);
			*Dest = Src;
		}
	}
	else
	{
		PrintDebug(DEBUG_ENABLED, L"OpenCV Major Version : %d ,Minor Version : %d \n", CV_MAJOR_VERSION,CV_MINOR_VERSION);

		const int Maxval = 8;
		*Dest = cv::Mat(Src.rows, Maxval, Src.type());
		for (int Col = 0; Col < Maxval; Col++)
		{
			Dest->at<double>(0,Col) = Src.at<double>(0,Col);
		}
	}
	return true;
}

//Computing the Q Matrix
BOOL TaraCamParameters::ComputeRectifyPrams()
{
	PrintDebug(DEBUG_ENABLED, L"Q Matrix Computation !!");

	cv::Mat R1, P1, R2, P2;
	cv::Rect roi1, roi2;
	cv::Size img_size(gImageWidth, gImageHeight);

	cv::Mat M1_l, M2_l;
	M1_l = M1.clone();
	M2_l = M2.clone();

	stereoRectify( M1_l, D1, M2_l, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, img_size, &roi1, &roi2 );

	initUndistortRectifyMap(M1_l, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2_l, D2, R2, P2, img_size, CV_16SC2, map21, map22);

	return 1;
}

//Rectifying the images
BOOL TaraCamParameters::RemapStereoImage(cv::Mat *mCamRightFrame, cv::Mat *mCamLeftFrame)
{
	cv::Mat _rRightImage, _rLeftImage;

	int actualWidth = mCamRightFrame->cols;
	int actualHeight = mCamRightFrame->rows;

	//Resize to the higher resolution and rectify the image
	if(actualWidth != (gImageWidth) ||actualHeight != gImageHeight)
	{
		resize(*mCamLeftFrame,  *mCamLeftFrame,  cv::Size(gImageWidth, gImageHeight));
		resize(*mCamRightFrame, *mCamRightFrame, cv::Size(gImageWidth, gImageHeight));
	}

	remap(*mCamRightFrame, _rRightImage, map11, map12, cv::INTER_LINEAR);
	remap(*mCamLeftFrame, _rLeftImage, map21, map22, cv::INTER_LINEAR);

	//Resize back to the original size
	if(actualWidth != gImageWidth || actualHeight != gImageHeight)
	{
		resize(_rRightImage, *mCamRightFrame,  cv::Size(actualWidth, actualHeight));
		resize(_rLeftImage, *mCamLeftFrame, cv::Size(actualWidth, actualHeight));
	}
	else
	{
		*mCamLeftFrame =  _rLeftImage.clone();
		*mCamRightFrame = _rRightImage.clone();
	}

	return true;
}

//Constructor
Disparity::Disparity()
{
	//Image Resolutions
	ImageSize.width = 0;   //CHECk Width
	ImageSize.height = 0;

	//Default
	e_DisparityOption = 1;
}

//Destructor
Disparity::~Disparity()
{
	//Release the camera device
	_CameraDevice.release();

	//Relase the vector
	vector<cv::Mat>().swap(StereoFrames);

	//Deinitialise the extension unit
	DeinitExtensionUnit();
}

BOOL Disparity::InitCamera(bool GenerateDisparity, bool FilteredDisparityMap)
{
	//Device ID thats to be streamed
	int DeviceID;

	//Read the device ID to stream
	CameraEnumeration _CameraEnumeration(&DeviceID, &ImageSize);

	if(DeviceID < 0)//Check for a valid device ID
	{
		PrintDebug(DEBUG_ENABLED, L"Invalid Device");
		return 0;
	}

	//Open the device selected by the user.
	_CameraDevice.open(DeviceID);

	//Camera Device
	if(!_CameraDevice.isOpened())
	{
		PrintDebug(DEBUG_ENABLED, L"No Camera Device Found");
		printf("\nNo Camera Device Found\n");
		return 0;
	}

	if (DEFAULT_FRAME_WIDTH == ImageSize.width && DEFAULT_FRAME_HEIGHT == ImageSize.height)
	{
		_CameraDevice.set(CV_CAP_PROP_FRAME_WIDTH, 752);
		_CameraDevice.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	}

	//Setting up Y16 Format
	_CameraDevice.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '1', '6', ' '));

	//Setting up FrameRate
	_CameraDevice.set(CV_CAP_PROP_FPS, FRAMERATE);

	//Setting width and height
	_CameraDevice.set(CV_CAP_PROP_FRAME_WIDTH, ImageSize.width);
	_CameraDevice.set(CV_CAP_PROP_FRAME_HEIGHT, ImageSize.height);

	PrintDebug(DEBUG_ENABLED, L"_CameraEnumeration.DeviceInstanceID: %s", _CameraEnumeration.DeviceInstanceID);

	//Init the extension units
	if(!InitExtensionUnit(_CameraEnumeration.DeviceInstanceID))
	{
		PrintDebug(DEBUG_ENABLED, L"Extension Unit Initialisation Failed");
		printf("\nExtension Unit Initialisation Failed\n");
		return 0;
	}

	//Setting up the camera in Master mode
	if(!SetStreamModeStereo(MASTERMODE))
	{
		PrintDebug(DEBUG_ENABLED, L"Setting up Stream Mode Failed");
		printf("\nSetting up stream mode Failed, initiating in the default mode!\n");
	}

	//Setting to default Brightness
	SetBrightness(DEFAULTBRIGHTNESS);

	//Setting to default exposure
	SetExposure(SEE3CAM_STEREO_EXPOSURE_DEF);

	//Choose whether the disparity is filtered or not
	gFilteredDisparity = FilteredDisparityMap;

	//Initialise the disparity options
	if(!Init(GenerateDisparity))
	{
		PrintDebug(DEBUG_ENABLED, L"InitCamera - Camera Matrix Initialisation Failed!!");
		return 0;
	}

	//Mat creation
	InterleavedFrame.create(ImageSize.height, ImageSize.width, CV_8UC2);

	return 1;
}

//Grabs the frame, converts it to 8 bit, splits the left and right frame and returns the rectified frame
BOOL Disparity::GrabFrame(cv::Mat *LeftImage, cv::Mat *RightImage)
{
	//Read the frame from camera
	//Y16 ==> CV_16UC1 2
	_CameraDevice.read(InputFrame10bit);

	//Invalid Frame
	if(InputFrame10bit.empty())
	{
		PrintDebug(DEBUG_ENABLED, L"No Frame Received! Camera is Unavailable!");
		cout << endl << endl << "No Frame Received! Camera is Unavailable!" << endl << endl;
		return 0;
	}
	//copy the data to the two channel image
	InterleavedFrame.data = InputFrame10bit.data;

	//Splitting the data
	split(InterleavedFrame, StereoFrames);

	//Rectify Frames
	_TaraCamParameters.RemapStereoImage(&StereoFrames[0], &StereoFrames[1]);

	//Copies the Frames
	*LeftImage =  StereoFrames[0].clone();
	*RightImage = StereoFrames[1].clone();

	return 1;
}

//initialise all the variables and create the Disparity parameters
BOOL Disparity::Init(bool GenerateDisparity)
{
	//Init to read the Camera Matrix
	if(!_TaraCamParameters.Init())
	{
		PrintDebug(DEBUG_ENABLED, L"Camera Matrix Initialisation Failed!!");
		return 0;
	}

	//Depth Map
	if(_TaraCamParameters.Q.empty()) //Matrix loading failed
	{
		PrintDebug(DEBUG_ENABLED, L"Q Mat Computation Failed!!");
		return 0;
	}

	//Copying to the local value
	DepthMap = _TaraCamParameters.Q;

	//Initialises only when the disparity option is enabled
	if(GenerateDisparity)
	{
		//BM method Parameters for computing Disparity Map
		bm_preFilterSize	=	5;
		bm_preFilterCap		=	25;
		bm_SADWindowSize	=	21; // must be odd, within 5..255 and not larger than image width or height
		bm_minDisparity		=	0;
		bm_textureThreshold	=	5;
		bm_uniquenessRatio	=	1;
		bm_speckleRange		=	31;
		bm_disp12MaxDiff	=	1;
		bm_numberOfDisparities	=	4;  // = val * 16
		bm_speckleWindowSize	=	350;

		//SGBM method Parameters for computing Disparity Map
		sgbm_preFilterCap	=	61;
		sgbm_SADWindowSize	=	8;
		sgbm_minDisparity	=	0;
		sgbm_speckleRange	=	31;
		sgbm_disp12MaxDiff	=	1;
		sgbm_uniquenessRatio	 =	0;
		sgbm_speckleWindowSize	 =	200;
		sgbm_numberOfDisparities =	4;

		e_DWSLFLambda	=	8000.0;
		e_DWSLFSigma	=	1.5;
		e_ScaleDispMap	=	5.0;
		e_ScaleImage	=	0.60;

		if(DISPARITY_OPTION) // Disparity Map Quality and Frame Rate
		{
			e_DisparityOption = 1;  // 1 - Stereo_SGBM
		}
		else
		{
			e_DisparityOption = 0;  // 0 - Stereo_BM
		}

		mRange = cv::Mat(cv::Size(50, ImageSize.height), CV_8UC1);
		for (int Row = 0; Row < ImageSize.height; Row++)
		{
			const float scaleR = float(255.0 / (float)ImageSize.height);
			mRange.row(Row).setTo(Row * scaleR);
		}

		//Algorithm Parameters
		SetAlgorithmParam();
	}

	return 1;
}

//Setting up the parameters of Disparity Algorithm
BOOL Disparity::SetAlgorithmParam()
{
	PrintDebug(DEBUG_ENABLED, L"Setting Up the Algorithm Parameters");
	int numberOfDisparities;
	if(!e_DisparityOption)  //STEREO_BM algorithm
	{
		bm_preFilterSize	=	int(LIMIT(bm_preFilterSize, 5, 63));
		bm_SADWindowSize	=	MAX(5, bm_SADWindowSize);
		if(bm_SADWindowSize % 2 == 0)
		{
			bm_SADWindowSize++;
		}

		bm_preFilterCap = int(LIMIT(bm_preFilterCap, 1, 63)); // must be within 1 and 63

		if(bm_preFilterSize % 2 == 0)
		{
			bm_preFilterSize++;
		}

		numberOfDisparities = bm_numberOfDisparities * 16;
		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((ImageSize.width/8) + 15) & -16;

		bm_left = cv::StereoBM::create(numberOfDisparities, bm_SADWindowSize > 0 ? bm_SADWindowSize : 9);

		if(gFilteredDisparity) //Gives a Filtered Disparity
		{
			wls_filter = createDisparityWLSFilter(bm_left);//For Filtering Disparity Map
			bm_right   = createRightMatcher(bm_left);
		}

		bm_left->setROI1(cv::Rect(0, 0, ImageSize.width, ImageSize.height)); //CHECk Width
		bm_left->setROI2(cv::Rect(0, 0, ImageSize.width, ImageSize.height)); //CHECk Width
		bm_left->setPreFilterCap(bm_preFilterCap);
		bm_left->setBlockSize(bm_SADWindowSize > 0 ? bm_SADWindowSize : 9);
		bm_left->setMinDisparity(bm_minDisparity);
		bm_left->setNumDisparities(numberOfDisparities);
		bm_left->setTextureThreshold(bm_textureThreshold);
		bm_left->setUniquenessRatio(bm_uniquenessRatio);
		bm_left->setSpeckleWindowSize(bm_speckleWindowSize);
		bm_left->setSpeckleRange(bm_speckleRange);
		bm_left->setDisp12MaxDiff (bm_disp12MaxDiff);
		bm_left->setPreFilterType(CV_STEREO_BM_XSOBEL);

		#pragma endregion For Stereo BM Parameters

	}
	else //STEREO_3WAY
	{
		numberOfDisparities = sgbm_numberOfDisparities * 16;
		numberOfDisparities = numberOfDisparities 	> 0 ? numberOfDisparities  : ((ImageSize.width/8) + 15) & -16;

		sgbm_left = cv::StereoSGBM::create(0, numberOfDisparities, sgbm_SADWindowSize > 0 ? sgbm_SADWindowSize : 3);

		if(gFilteredDisparity) //Gives a Filtered Disparity
		{
			wls_filter = createDisparityWLSFilter(sgbm_left);
			sgbm_right = createRightMatcher(sgbm_left);
		}

		int cn = 1;
		sgbm_left->setPreFilterCap(sgbm_preFilterCap);
		sgbm_left->setBlockSize (sgbm_SADWindowSize > 0 ? sgbm_SADWindowSize : 3);
		sgbm_left->setP1(8 * cn * sgbm_SADWindowSize * sgbm_SADWindowSize);
		sgbm_left->setP2(32 * cn * sgbm_SADWindowSize * sgbm_SADWindowSize);
		sgbm_left->setNumDisparities(numberOfDisparities);
		sgbm_left->setMinDisparity(sgbm_minDisparity);
		sgbm_left->setUniquenessRatio(sgbm_uniquenessRatio);
		sgbm_left->setSpeckleWindowSize(sgbm_speckleWindowSize);
		sgbm_left->setSpeckleRange(sgbm_speckleRange);
		sgbm_left->setDisp12MaxDiff(sgbm_disp12MaxDiff);

		sgbm_left->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
	}

	if(gFilteredDisparity) //Gives a Filtered Disparity
	{
		wls_filter->setLambda(e_DWSLFLambda);
		wls_filter->setSigmaColor(e_DWSLFSigma);
	}

	return 1;
}

//Estimates the disparity of the camera
BOOL Disparity::GetDisparity(cv::Mat LImage, cv::Mat RImage, cv::Mat *mDisparityMap, cv::Mat *FilteredDisparity)
{
	//Scale value
	e_ScaleImage = LIMIT(e_ScaleImage, 0.20, 1);

	if(e_ScaleImage != 1.0) //Scaling the Input to speed up the process
	{
		resize(LImage, LScaleImage, cv::Size(), e_ScaleImage, e_ScaleImage, cv::INTER_LINEAR_EXACT);
		resize(RImage, RScaleImage, cv::Size(), e_ScaleImage, e_ScaleImage, cv::INTER_LINEAR_EXACT);

		LImage = LScaleImage;
		RImage = RScaleImage;
	}

	if(!e_DisparityOption)  //STEREO_BM algorithm
	{
		bm_left->compute(LImage, RImage, *mDisparityMap);

		if(gFilteredDisparity) //Filtered disparity
		{
			bm_right->compute(RImage, LImage, RDisparity);
		}
	}
    else //STEREO_3WAY algorithm
	{
		sgbm_left->compute(LImage, RImage, *mDisparityMap);

		if(gFilteredDisparity) //Filtered disparity
		{
			sgbm_right->compute(RImage, LImage, RDisparity);
		}
	}

	if(gFilteredDisparity) //filtered
	{
		wls_filter->setLambda(e_DWSLFLambda);
		wls_filter->setSigmaColor(e_DWSLFSigma);
		wls_filter->filter(*mDisparityMap, LImage, disp_filtered, RDisparity);

		*mDisparityMap = disp_filtered;

		//Disparity map to view
		getDisparityVis(*mDisparityMap, *FilteredDisparity, e_ScaleDispMap);

		if(e_ScaleImage  != 1.0) //Scale back the output image
		{
			resize(*mDisparityMap , ScaledDisparityMap, cv::Size(ImageSize.width , ImageSize.height));
			resize(*FilteredDisparity , ScaledDisparityMap_viz, cv::Size(ImageSize.width, ImageSize.height));

			*mDisparityMap = ScaledDisparityMap;
			*FilteredDisparity = ScaledDisparityMap_viz;
		}
	}
	else  //Without Filter
	{
		//Disparity map to view
		getDisparityVis(*mDisparityMap, *FilteredDisparity, e_ScaleDispMap);

		if(e_ScaleImage != 1.0) //Scale back the output image
		{
			resize(*mDisparityMap , ScaledDisparityMap, cv::Size(ImageSize.width, ImageSize.height));
			resize(*FilteredDisparity , ScaledDisparityMap_viz, cv::Size(ImageSize.width, ImageSize.height));

			*mDisparityMap = ScaledDisparityMap;
			*FilteredDisparity = ScaledDisparityMap_viz;
		}
	}

	//Color map for the Disparity image
	hconcat(*FilteredDisparity, mRange, mDisp_WR);
	applyColorMap(mDisp_WR, gDisparityMap_viz, cv::COLORMAP_JET);

	//Copy to the local variable
	gDisparityMap =  mDisparityMap->clone();
	*mDisparityMap = FilteredDisparity->clone();
	*FilteredDisparity = gDisparityMap_viz.clone();

	return true;
}

BOOL Disparity::FastDisparity(cv::Mat LeftImage, cv::Mat RightImage, cv::Mat *FilteredDisparity)
{
	//resizing image
	resize(LeftImage, LeftScaleImage, cv::Size(), e_ScaleImage, e_ScaleImage, INTER_LINEAR_EXACT);
	resize(RightImage, RightScaleImage, cv::Size(), e_ScaleImage, e_ScaleImage, INTER_LINEAR_EXACT);

	//performing matching
	bm_left->compute(LeftScaleImage, RightScaleImage, left_disp);
	bm_right->compute(RightScaleImage, LeftScaleImage, right_disp);

	//performing filtering
	wls_filter->setLambda(e_DWSLFLambda);
	wls_filter->setSigmaColor(e_DWSLFSigma);
	wls_filter->filter(left_disp, LeftImage, filtered_disp, right_disp);

	getDisparityVis(filtered_disp, *FilteredDisparity, e_ScaleDispMap);

	resize(*FilteredDisparity, filtered_disp, cv::Size(ImageSize.width, ImageSize.height));
	*FilteredDisparity = filtered_disp.clone();

	return true;
}


//Estimates the Depth of the point passed.
BOOL Disparity::EstimateDepth(cv::Point Pt, float *DepthValue)
{
	cv::Mat_<float> vec(4, 1);
	cv::Mat disp_leftCrop, disp_32;
	cv::Mat Q_32;

	//Validate the point
	if(Pt.x <= 0  || Pt.y <= 0 || Pt.x > ImageSize.width || Pt.y > ImageSize.height)
	{
		*DepthValue = -1;
		return 0;
	}

	cv::Point WithinImage(-1, -1);

	//Handling points in the corner
	if((Pt.x + 20) > ImageSize.width)
	{
		WithinImage.x = ( ImageSize.width - (Pt.x + 20)) + Pt.x;
		WithinImage.y = Pt.y;
	}

	if((Pt.y + 20) > ImageSize.height)
	{
		WithinImage.y = ( ImageSize.height - (Pt.y + 20)) + Pt.y;
		if(WithinImage.x == -1) //In case x is within the range
			WithinImage.x = Pt.x;
	}

	if(WithinImage.x == -1 || WithinImage.y == -1)
	{
		WithinImage.x = Pt.x;
		WithinImage.y = Pt.y;
	}

	//Average depth of 20x20 around the selected point is taken
	cv::Rect recROI(WithinImage.x, WithinImage.y, 20, 20);

	//Cropping the region
	disp_leftCrop = gDisparityMap(recROI);

	//Converting to 32 bit
	disp_leftCrop.convertTo(disp_32, CV_32FC1, 1.0 / 16); //CHANGES MADE HERE

	//Computing the mean of the selected point
	cv::Scalar MeanDisp = mean(disp_32);

	//Converting to 32 bit
	DepthMap.convertTo(Q_32, CV_32FC1);

	//Intialising the Mat
	vec(0) = 0;
	vec(1) = 0;
	vec(2) = (float)MeanDisp.val[0];

	// Discard points with 0 disparity
	if(vec(2) != 0)
	{
		vec(3) = 1.0;
		vec = Q_32 * vec;
		vec /= vec(3);

		// Discard points that are too far from the camera, and thus are highly unreliable
		*DepthValue = vec(2) * float(e_ScaleImage);
	}

	return 1;
}

//Range Selection
double Disparity::LIMIT(double n, double lower, double upper)
{
	return max(lower, min(n, upper));
}

//Sets the Brightness Val of the  camera
BOOL Disparity::SetBrightness(int BrightnessVal)
{
	//Sets the brightness of the Camera
	return _CameraDevice.set(CV_CAP_PROP_BRIGHTNESS, BrightnessVal);
}

//Sets the exposure of the camera
BOOL Disparity::SetExposure(int ExposureVal)
{
	if(!SetManualExposureStereo(ExposureVal)) //Set the manual exposure
	{
		PrintDebug(DEBUG_ENABLED, L"Setting Manual Exposure Failed!!");
		return 0;
	}
	return 1;
}

//Sets the exposure of the camera
BOOL Disparity::GetExposure(int *ExposureVal)
{
	if(!GetManualExposureStereo(ExposureVal)) //Get the manual exposure
	{
		PrintDebug(DEBUG_ENABLED, L"Reading Exposure Failed!!");
		return 0;
	}
	return 1;
}

//Sets the exposure of the camera
BOOL Disparity::SetAutoExposure()
{
	UINT StreamMode = -1;
	GetStreamMode(&StreamMode); //Read the stream mode

	if(StreamMode == TRIGGERMODE)
	{
		cout << endl << "Switch to Master Mode to set Auto Exposure!!" << endl;
	}
	else
	{
		int CurrentExpValue = 0;
		GetExposure(&CurrentExpValue); //Read the exposure value

		if(CurrentExpValue != AUTOEXPOSURE)
		{
			//Setting up the exposure
			if(SetAutoExposureStereo())
			{
				cout << endl << "Switching to Auto Exposure!!" << endl;
			}
			else
			{
				cout << endl << "Failed!! Switching to Auto Exposure!!" << endl;
			}
		}
		else
		{
			cout << endl << "Already in Auto Exposure!!" << endl;
		}
	}

	return TRUE;
}

//Sets the Stream Mode of the  camera
BOOL Disparity::SetStreamMode(UINT StreamMode)
{
	UINT CurrentMode = -1;
	GetStreamModeStereo(&CurrentMode); //Read the current stream mode

	//checking up if the selected mode is Trigger mode
	if(CurrentMode != StreamMode)
	{
		if(StreamMode == TRIGGERMODE)
		{
			int ExposureValue = 0;
			GetManualExposureStereo(&ExposureValue);

			if(ExposureValue == AUTOEXPOSURE) //Check whether it is in Auto Exposure
			{
				SetExposure(SEE3CAM_STEREO_EXPOSURE_DEF);
				cout << endl << "Changing to Manual Exposure to set to Trigger Mode" << endl;
			}
		}

		//Set the stream mode
		SetStreamModeStereo(StreamMode);
	}
	else
	{
		return 0;
	}
	return 1;
}

//Gets the Stream Mode of the camera
BOOL Disparity::GetStreamMode(UINT *StreamMode)
{
	//Read the current stream mode
	GetStreamModeStereo(StreamMode);
	return TRUE;
}

//Constructor
CameraEnumeration::CameraEnumeration(int *DeviceID, cv::Size *SelectedResolution)
{
	//Gets the Input from the user
	GetDeviceIDeCon(DeviceID, SelectedResolution);
}

//Destructor
CameraEnumeration::~CameraEnumeration()
{
	//Free the vectors
	vector<string>().swap(DeviceInstances);
	vector<vector<cv::Size>>().swap(CameraResolutions);
}

//Reads the Input from the user
BOOL CameraEnumeration::GetDeviceIDeCon(int *DeviceID, cv::Size *SelectedResolution)
{
	PrintDebug(DEBUG_ENABLED, L"Get DeviceID eCon");

	int NoDevicesConnected  = -1;
	int ResolutionID = -1;

	//Get the list of devices connected
	NoDevicesConnected = GetListofDeviceseCon();

	//Check for a valid ID
	if(NoDevicesConnected == 0)
	{
		PrintDebug(DEBUG_ENABLED, L"No Devices connected");
		printf("\n No Devices connected \n");
		*DeviceID = -1;
		return 0;
	}

	PrintDebug(DEBUG_ENABLED, L"Number of Devices Connected: %d", NoDevicesConnected);

	//Print the name of the devices connected
	for(int i = 0; i < NoDevicesConnected; i++)
	{
		wstring ws(deviceNames[i]);
		string DeviceName(ws.begin(), ws.end());
		cout << "Device ID: " << i << ", Device Name: " << DeviceName << endl << endl;
	}

	//User Input of the Device ID
	cout << endl << "Enter the Device ID to Process" << endl;
	cin >> *DeviceID;

	//Check for a valid ID
	if(*DeviceID > (NoDevicesConnected - 1))
	{
		PrintDebug(DEBUG_ENABLED, L"InValid Device ID");
		printf("\n Invalid Device ID\n");

		*DeviceID = -1;
		return 0;
	}

	wstring ws(deviceNames[*DeviceID]);
	string DeviceName(ws.begin(), ws.end());

	//Check whether the selected device is Stereo
	if(strcmp(DeviceName.c_str(), "See3CAM_Stereo") != 0)
	{
		PrintDebug(DEBUG_ENABLED, L"Not a Stereo Camera");
		printf("\nNot a Stereo Camera\n");
		*DeviceID = -1;
		return 0;
	}

	cout << endl << "Resolutions Supported" << endl << endl;

	//Resolution Supported
	for(size_t i = 0; i < CameraResolutions[*DeviceID].size(); i++)
	{
		cout << "ID: " << i << ", Resolution: " << CameraResolutions[*DeviceID][i].width << "x"<< CameraResolutions[*DeviceID][i].height << endl << endl;
	}

	cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	string Inputline;

	//User Input of the Device ID
	cout << endl << "Enter the Resolution ID to Stream: " << endl;
	cin >> ResolutionID;

	if((ResolutionID > int(CameraResolutions[*DeviceID].size() - 1)) || (ResolutionID == -1)) //In case of wrong selection
	{
		cout << endl << "Invalid  ResolutionID!!" << endl;
		*DeviceID = -1;
		return 0;
	}

	cout << endl << "Setting up Resolution ID: " << ResolutionID << endl << endl;

	DeviceInstanceID = (TCHAR*) malloc (sizeof(TCHAR)  * MAX_PATH);
	mbstowcs_s(NULL, DeviceInstanceID, MAX_PATH, DeviceInstances[*DeviceID].c_str(), MAX_PATH);

	//Selected Resolution
	SelectedResolution->width = CameraResolutions[*DeviceID][ResolutionID].width;
	SelectedResolution->height = CameraResolutions[*DeviceID][ResolutionID].height;

	return 1;
}

BOOL CameraEnumeration::GetListofDeviceseCon()
{
	PrintDebug(DEBUG_ENABLED, L"Get List of Devices eCon");

	bool silent = false;
	int deviceCounter = 0;
	IMoniker *m_pmVideo;
	TCHAR *USBInstanceID = new TCHAR[MAX_PATH];

	//COM Library Intialization
    CoInitialize(NULL);

	TCHAR devicePath[MAX_PATH]=_T("");
	IGraphBuilder *pGraph = NULL;
	ICaptureGraphBuilder2 *pBuilder = NULL;

	IBaseFilter *m_pVCap;
    ICreateDevEnum *pDevEnum = NULL;
    IEnumMoniker *pEnum = NULL;

    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL,
        CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, reinterpret_cast<void**>(&pDevEnum));


    if (SUCCEEDED(hr))
    {
        // Create an enumerator for the video capture category.
        hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum, 0);

       if(hr == S_OK)
	   {
            IMoniker *pMoniker = NULL;

            while (pEnum->Next(1, &pMoniker, NULL) == S_OK)
			{
                IPropertyBag *pPropBag;
                hr = pMoniker->BindToStorage(0, 0, IID_IPropertyBag,(void**)(&pPropBag));

                if (FAILED(hr))
				{
                    pMoniker->Release();
                    continue;  // Skip this one, maybe the next one will work.
                }

				VARIANT var;
				var.vt = VT_BSTR;
				hr = pPropBag->Read(L"DevicePath", &var, 0);
				if(hr == S_OK)
				{
					StringCbPrintf(devicePath,MAX_PATH,L"%s",var.bstrVal);
					if(devicePath != NULL)
					{
						//USBInstanceID  = NULL;
						if(IsStereoDeviceAvail(devicePath))
						{
							StringCbPrintf(USBInstanceID, MAX_PATH, L"%s", devicePath);
						}
						//DeviceInstances.push_back(USBInstanceID);

						wstring ws(USBInstanceID);
						string DeviceName(ws.begin(), ws.end());
						DeviceInstances.push_back(DeviceName);


					}
				}
				else
				{
					break;
				}
				SysFreeString(var.bstrVal);

                 // Find the description or friendly name.
                VARIANT varName;
                VariantInit(&varName);

				hr = pPropBag->Read(L"FriendlyName", &varName, 0);

                if (SUCCEEDED(hr))
				{
					hr = pPropBag->Read(L"FriendlyName", &varName, 0);

                    int count = 0;
                    int maxLen = sizeof(deviceNames[0])/sizeof(deviceNames[0][0]) - 2;
                    while( varName.bstrVal[count] != 0x00 && count < maxLen)
					{
                        deviceNames[deviceCounter][count] = varName.bstrVal[count];
                        count++;
                    }
                    deviceNames[deviceCounter][count] = 0;
                }

				m_pmVideo = pMoniker;
				hr = m_pmVideo->BindToStorage(0, 0, IID_IPropertyBag, (void **)&pPropBag);

				if (!SUCCEEDED(hr))
				{
					return 0;
				}
				hr = m_pmVideo->BindToObject(0, 0, IID_IBaseFilter, (void**)&m_pVCap);

				if (!SUCCEEDED(hr))
				{
					return 0;
				}

				// Create the Filter Graph Manager.
				HRESULT hr =  CoCreateInstance(CLSID_FilterGraph, NULL,
					CLSCTX_INPROC_SERVER, IID_IGraphBuilder, (void **)&pGraph);

				if (SUCCEEDED(hr))
				{
					// Create the Capture Graph Builder.
					hr = CoCreateInstance(CLSID_CaptureGraphBuilder2, NULL,	CLSCTX_INPROC_SERVER, IID_ICaptureGraphBuilder2,
						(void **)&pBuilder);
					if (SUCCEEDED(hr))
					{
						pBuilder->SetFiltergraph(pGraph);
					}
				}
				IAMStreamConfig *pConfig = NULL;
				hr = pBuilder->FindInterface(&PIN_CATEGORY_CAPTURE, &MEDIATYPE_Video, m_pVCap, IID_IAMStreamConfig, (void**)&pConfig);

			 vector<cv::Size> Image;
				if (SUCCEEDED(hr))
				{
					int iCount = 0, iSize = 0;
					pConfig->GetNumberOfCapabilities(&iCount, &iSize);
					if (iSize == sizeof(VIDEO_STREAM_CONFIG_CAPS))
					{
						for (int iFormat = 0; iFormat < iCount; iFormat += 2)
						{
							VIDEO_STREAM_CONFIG_CAPS scc;
							AM_MEDIA_TYPE *pmt;
							hr = pConfig->GetStreamCaps(iFormat, &pmt, (BYTE*)&scc);
							if(IsEqualGUID(pmt->subtype, MEDIASUBTYPE_RGB24)) //Skipping data of type RGB24
							{
								continue;
							}
							if (SUCCEEDED(hr))
							{
								Image.push_back(cv::Size(scc.InputSize.cx, scc.InputSize.cy));
							}
						}
					}
				}

				//Resolutions
				CameraResolutions.push_back(Image);
				vector<cv::Size>().swap(Image);
				pPropBag->Release();
                pPropBag = NULL;

                pMoniker->Release();
                pMoniker = NULL;

                deviceCounter++;
            }

            pDevEnum->Release();
            pDevEnum = NULL;

            pEnum->Release();
            pEnum = NULL;
        }
    }

    CoUninitialize();

	return deviceCounter; //Number of devices connected to the pc
}

//check whether the selected device is eCon's Stereo
BOOL CameraEnumeration::IsStereoDeviceAvail(TCHAR *Devicepath)
{
	PrintDebug(DEBUG_ENABLED, L"Is StereoDevice Available check!");
	try
	{
		TCHAR extrctd_vid[10] = _T("");
		TCHAR extrctd_pid[10] = _T("");
		TCHAR *vid_substr;
		TCHAR *pid_substr;

		vid_substr = wcsstr(_wcsupr(Devicepath), TEXT("VID_"));
		if(vid_substr != NULL)
		{
			wcsncpy_s(extrctd_vid,vid_substr+4,4);
			extrctd_vid[5] = '\0';
		}

		pid_substr = wcsstr(_wcsupr(Devicepath), TEXT("PID_"));
		if(pid_substr != NULL)
		{
			wcsncpy_s(extrctd_pid,pid_substr+4,4);
			extrctd_pid[5] = '\0';
		}

		if((wcscmp(_wcsupr(extrctd_vid),_wcsupr(VID))==0) && (wcscmp(_wcsupr(extrctd_pid),_wcsupr(See3CAM_STEREO)) == 0))
		{
			return TRUE;
		}

		return FALSE;
	}
	catch(...)
	{

		return FALSE;
	}
	PrintDebug(DEBUG_ENABLED, L"Stereo Device Available Check");
}

//Prints all the debug messages
void PrintDebug(BOOL bEnable, LPCTSTR szFormat,...)
{
	if(bEnable)
	{
		static TCHAR szBuffer[2048]={0};
		const size_t NUMCHARS = sizeof(szBuffer) / sizeof(szBuffer[0]);
		const int LASTCHAR = NUMCHARS - 1;

		// Format the input string
		va_list pArgs;
		va_start(pArgs, szFormat);

		// Use a bounded buffer size to prevent buffer overruns.  Limit count to
		// character size minus one to allow for a NULL terminating character.
		HRESULT hr = StringCchVPrintf(szBuffer, NUMCHARS - 1, szFormat, pArgs);
		va_end(pArgs);

		// Ensure that the formatted string is NULL-terminated
		szBuffer[LASTCHAR] = TEXT('\0');

		OutputDebugStringW(szBuffer);
	}
}

//Displays the Text on the image passed
BOOL DisplayText(cv::Mat Image, cv::String Text, cv::Point Location)
{
	//Display the text
	putText(Image, Text, Location, 2, 1.0, cv::Scalar(255, 0, 150)  , 2, 8, false);

	return true;
}

}
