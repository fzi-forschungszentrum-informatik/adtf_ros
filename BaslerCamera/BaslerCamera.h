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

#define CID_BASLER_CAMERA_STREAMING_SOURCE "basler_camera.streaming_source.user.aadc.cid"
#define LABEL_BASLER_CAMERA_STREAMING_SOURCE "Better Basler Camera"

/*! A the main class of the basler camera. */
class cBaslerCamera : public adtf::streaming::cSampleStreamingSource
{
public:
    ADTF_CLASS_ID_NAME(cBaslerCamera,
        CID_BASLER_CAMERA_STREAMING_SOURCE,
        LABEL_BASLER_CAMERA_STREAMING_SOURCE);

    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock), REQUIRE_INTERFACE(adtf::services::IKernel));

private:

    /*! Width of the stream */
    property_variable<tInt>   m_streamWidth = 1280;
    /*! Height of the stream */
    property_variable<tInt>   m_streamHeight = 960;
    /*! The brightness */
    property_variable<tFloat> m_brightness = 0.3;
    /*! The offset */
    property_variable<tInt>   m_xOffset = 440;
    /*! The offset */
    property_variable<tInt>   m_yOffset = 330;
    /*! The width */
    property_variable<tInt>   m_width = 400;
    /*! The height */
    property_variable<tInt>   m_height = 300;
    /*! The frame delay */
    property_variable<tInt64> m_nFrameDelay = 100000;
    //sample pin writer
    cSampleWriter m_oOut;

    /*! Misc. */
    kernel_timer m_oTimer;
    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;
    /*! The camera device */
    CBaslerUsbInstantCamera m_camera;    
    /*! RGB24 -> 3* */
    tInt m_maxByteSize = 3 * m_streamHeight * m_streamWidth;
public:
    /*! Default constructor. */
    cBaslerCamera();

    tResult Construct() override;
    tResult StartStreaming() override;
    tResult StopStreaming() override;

private:

    /*!
     * Timer function.
     *
     * \return  A tVoid.
     */
    tVoid TimerFunc();

    /*!
     * Creates a new frame.
     *
     * \return  The new new frame.
     */
    object_ptr<ISample> CreateNewFrame();


};