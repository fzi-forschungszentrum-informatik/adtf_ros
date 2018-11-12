/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/
#pragma once


//*************************************************************************************************
#define CID_CCUSTOMUNDISTORTION_DATA_TRIGGERED_FILTER "custom_undistortion.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

/*! the main class for the custom undistortion module */
class cCustomUndistortion : public cTriggerFunction
{
private:
    //Properties
    /*! Location of Camera Calib File */
    adtf::base::property_variable<cFilename> m_calibFile = cFilename(cString("basler_fisheye_intrinsic_calib.yml"));

    //Pins
    /*! Reader of an InPin. */
    cPinReader m_oReader;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriter;

    //Stream Formats
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_sInputFormat;

    //OpenCV Variables
    /*! true if rectification maps have been build */
    tBool m_rectifyMapsSet = tFalse;
    /*! camera matrix from calibration file */
    cv::Mat m_cameraMatrix;
    /*! distorsion coefficients from calibration file */
    cv::Mat m_distorsionMatrix;
    /*! rectification map 1, x coordinates */
    cv::Mat m_rectifyMap1;
    /*! rectification map 2, y coordinates */
    cv::Mat m_rectifyMap2;

    // GPU processing

/*! true if there is a cuda enabled gpu, currently only linux! */
    tBool m_bGpuAvailable;

    /*! activates the gpu image rectification and scaling, if available. */
    property_variable<tBool> m_bGpuProcessing;

    property_variable<tFloat32> m_fScaling = 1.0f;

    /*! GPU rectification map 1, x coordinates */
    cv::cuda::GpuMat m_GpuRectifyMap1;

    /*! GPU rectification map 2, y coordinates */
    cv::cuda::GpuMat m_GpuRectifyMap2;

    /*! GPU image used to upload the current incoming image. */
    cv::cuda::GpuMat m_GpuImgUpload;

    /*! GPU image used to remap the current image. */
    cv::cuda::GpuMat m_GpuImgRemaped;

    /*! GPU image used to download the current outgoing image. */
    cv::cuda::GpuMat m_GpuImgDownload;


    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;


public:

    /*! Default constructor./ */
    cCustomUndistortion();

    /*! Destructor. */
    virtual ~cCustomUndistortion() = default;

    /*!
     * Overwrites the Configure This is to Read Properties prepare your Trigger Function.
     *
     * \return  Standard Result Code.
     */
    tResult Configure() override;

    /*!
     * Overwrites the Process You need to implement the Reading and Writing of Samples within this
     * function MIND: Do Reading until the Readers queues are empty or use the
     * IPinReader::GetLastSample()
     * This FUnction will be called if the Run() of the TriggerFunction was called.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     *
     * \return  Standard Result Code.
     */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

};


//*************************************************************************************************
