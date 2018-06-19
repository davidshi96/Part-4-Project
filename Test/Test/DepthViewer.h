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
	void CircleDetection();
	void DisparityCalculations();
	void init();

private:

	int ManualExposure;

	//Object to access Disparity
	Disparity _Disparity;	

	cv::Mat LeftImage, RightImage;


};

//Call back function
//void DepthPointSelection(int MouseEvent, int x, int y, int flags, void* param) ;