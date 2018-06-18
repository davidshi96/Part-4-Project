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
#pragma once
#include "Tara.h"

using namespace Tara;
using namespace cv;
using namespace cv::ximgproc;

#define SEE3CAM_STEREO_AUTO_EXPOSURE		1

class DepthViewer
{
public:

	void CameraStreaming();
	void DisparityCalculations();
	void init();

private:

	int ManualExposure;

	//Object to access Disparity
	Disparity _Disparity;	

	cv::Ptr<cv::StereoBM> bm_left;
	cv::Ptr<cv::StereoMatcher> bm_right;

	cv::Ptr<cv::StereoSGBM> sgbm_left;
	cv::Ptr<cv::StereoMatcher> sgbm_right;
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

	cv::Mat LeftImage, RightImage;
	cv::Mat left_disp, right_disp;
	cv::Mat LeftScaleImage, RightScaleImage;
	cv::Mat filtered_disp, raw_disp_vis, filtered_disp_vis;
};

//Call back function
void DepthPointSelection(int MouseEvent, int x, int y, int flags, void* param) ;