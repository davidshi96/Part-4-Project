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
///////////////////////////////////////////////////////////////////////////

// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the TARA_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// ETARA_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.

#ifndef _TARA_H
#define _TARA_H

#ifdef TARA_EXPORTS
#define TARA_API __declspec(dllexport)
#else
#define TARA_API __declspec(dllimport)
#endif

// Windows Header Files:
#include <windows.h>
#include <tchar.h>
#include <iostream>
#include <string>
#include <conio.h>

#include "eCAMFwSw.h"
#include "DShow.h" //Direct Show header

//OpenCV headers
#include "highgui.hpp"
#include "videoio.hpp"
#include "imgproc.hpp"
#include "objdetect.hpp"
#include "ximgproc\disparity_filter.hpp"

//using namespace cv;
using namespace std;
using namespace cv::ximgproc;

#define FRAMERATE 60
#define MASTERMODE 1
#define TRIGGERMODE 0
#define DEBUG_ENABLED 0
#define MAX_CAMERA 10
#define DEFAULTBRIGHTNESS 4
#define AUTOEXPOSURE 1
#define DISPARITY_OPTION 0 // 1 - Best Quality Depth Map and Lower Frame Rate - Stereo_SGBM 3 Way generic Left and Right
						   // 0 - Low  Quality Depth Map and High  Frame Rate - Stereo_BM generic Left and Right

namespace Tara
{

//Prints all the debug messages
void PrintDebug(BOOL bEnable, LPCTSTR szFormat,...);

//Displays the Text on the image passed
int DisplayText(cv::Mat Image, cv::String Text, cv::Point Location);

class TARA_API TaraCamParameters
{
public:

	cv::Mat Q;

	//Constructor
	TaraCamParameters(void);

	//Destructor
	~TaraCamParameters(void);

	//Initialises and reads the camera Matrix
	BOOL Init();

	//Rectifying the images
	BOOL RemapStereoImage(cv::Mat *mCamFrame_Right, cv::Mat *mCamFrame_Left);

private:

	//Maximum width and height of the camera supported
	int gImageWidth, gImageHeight;

	//Variables to incorporate the intrinsic and extrinsic files
	cv::Mat M1, D1, M2, D2;
	cv::Mat R, T;
	cv::Mat map11, map12, map21, map22;

	//Loading the camera param
	BOOL LoadCameraMatrix();

	//to support lower version of OpenCV
	BOOL GetMatforCV(cv::Mat Src, cv::Mat *Dest);

	//Computes the Q matrix
	BOOL ComputeRectifyPrams();

};

class TARA_API Disparity
{
public:

	//Constructor
	Disparity();

	//Destructor
	~Disparity();

	//Local values to be passed to the functions
	cv::Mat gDisparityMap, gDisparityMap_viz;
	cv::Mat DepthMap;

	//Initialises the camera
	BOOL InitCamera(bool GenerateDisparity, bool FilteredDisparityMap);

	//Grabs the frame, converts it to 8 bit, splits the left and right frame and returns the rectified frame
	BOOL GrabFrame(cv::Mat *LeftImage, cv::Mat *RightImage);

	//Estimates the disparity of the camera
	BOOL GetDisparity(cv::Mat LImage, cv::Mat RImage, cv::Mat *mDisparityMap, cv::Mat *disp_filtered);
	BOOL FastDisparity(cv::Mat LeftImage, cv::Mat RightImage, cv::Mat *FilteredDisparity);

	//Estimates the Depth of the point passed.
	BOOL EstimateDepth(cv::Point Pt, float *DepthValue);

	//Sets the exposure of the camera
	BOOL SetExposure(int ExposureVal);

	//Sets the exposure of the camera
	BOOL GetExposure(int *ExposureVal);

	//Sets the exposure of the camera
	BOOL SetAutoExposure();

	//Sets the Brightness Val of the camera
	BOOL SetBrightness(int BrightnessVal);

	//Sets the Stream Mode of the  camera
	BOOL SetStreamMode(UINT StreamMode);

	//Gets the Stream Mode of the camera
	BOOL GetStreamMode(UINT *StreamMode);

private:
	//Disparity algorithm
	cv::Ptr<cv::StereoBM> bm_left;
	cv::Ptr<cv::StereoMatcher> bm_right;

	cv::Ptr<cv::StereoSGBM> sgbm_left;
	cv::Ptr<cv::StereoMatcher> sgbm_right;
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

	//variables needed for fast disparity calculations

	//cv::Mat left_disp, right_disp;
	//cv::Mat LeftScaleImage, RightScaleImage;
	//cv::Mat filtered_disp;

	//BM method Parameters for computing Disparity Map
	int bm_preFilterSize;
	int bm_preFilterCap;
	int bm_SADWindowSize;
	int bm_minDisparity;
	int bm_numberOfDisparities;
	int bm_textureThreshold;
	int bm_uniquenessRatio;
	int bm_speckleWindowSize;
	int bm_speckleRange;
	int bm_disp12MaxDiff;

	//SGBM method Parameters for computing Disparity Map
	int sgbm_preFilterCap;
	int sgbm_SADWindowSize;
	int sgbm_minDisparity;
	int sgbm_numberOfDisparities;
	int sgbm_uniquenessRatio;
	int sgbm_speckleWindowSize;
	int sgbm_speckleRange;
	int sgbm_disp12MaxDiff;

	double e_DWSLFLambda;
	double e_DWSLFSigma;
	double e_ScaleDispMap;
	double e_ScaleImage;

	//Range Selection
	double LIMIT(double n, double lower, double upper);

	//disparity option Selected
	int e_DisparityOption; //{ 0 - Stereo_BM, 1 - Stereo_SGBM }

	//Image Resolution
	cv::Size ImageSize;

	//Option to generate Filtered Disparity or Without filter - USER CHOICE
	bool gFilteredDisparity;

	//Range map to convert to color
	cv::Mat mRange;
	vector<cv::Mat> StereoFrames;
	cv::Mat InputFrame10bit, InputFrame8bit, InterleavedFrame;

	//DeviceID to stream the camera
	int DeviceID;

	//Object to hold Camera device
	cv::VideoCapture _CameraDevice;

	//Setting up the parameters of Disparity Algorithm
	BOOL SetAlgorithmParam();

	//Initialises the Camera Device with the passed width and height
	BOOL Init(bool GenerateDisparity);

	//Object to access the Q matrix connected
	TaraCamParameters _TaraCamParameters;
};

class TARA_API CameraEnumeration
{
private:

	//Stores the device instances of all enumerated devices
	vector<string> DeviceInstances;

	//Stores the resolution of all enumerated devices
	vector<vector<cv::Size>> CameraResolutions;

	//name of the devices connected
	WCHAR deviceNames[MAX_CAMERA][255];

	//function for finding number of devices connected,friendly name
	BOOL GetListofDeviceseCon();

	//Reads the device ID from the camera
	BOOL GetDeviceIDeCon(int *DeviceID, cv::Size *ResolutionSelected);

public:

	TCHAR *DeviceInstanceID; //Instance of the device selected by the user

	//Constructor
	CameraEnumeration(int *DeviceID, cv::Size *ResolutionSelected);

	//Destructor
	~CameraEnumeration();

	//Check for stereo camera
	BOOL IsStereoDeviceAvail(TCHAR *Devicepath);
};
}
#endif // _TARA_H
