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

#include "stdafx.h"

using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;
using namespace Pylon;

#include "BaslerCamera.h"

ADTF_PLUGIN(LABEL_BASLER_CAMERA_STREAMING_SOURCE, cBaslerCamera);

cBaslerCamera::cBaslerCamera()
{
    //Register Properties
    RegisterPropertyVariable("streamWidth [Pixel]", m_streamWidth);
    RegisterPropertyVariable("streamHeight [Pixel]", m_streamHeight);
    RegisterPropertyVariable("brightness", m_brightness);
    RegisterPropertyVariable("frame_delay [ms]", m_nFrameDelay);
    RegisterPropertyVariable("ROI xOffset [Pixel]", m_xOffset);
    RegisterPropertyVariable("ROI yOffset [Pixel]", m_yOffset);
    RegisterPropertyVariable("ROI width [Pixel]", m_width);
    RegisterPropertyVariable("ROI height [Pixel]", m_height);
}

tResult cBaslerCamera::Construct()
{
    RETURN_IF_FAILED(cSampleStreamingSource::Construct());

    //Define stream output format
    object_ptr<IStreamType> pType = make_object_ptr<cStreamType>(stream_meta_type_image());
    set_property(*pType, stream_meta_type_image::FormatName, ADTF_IMAGE_FORMAT(RGB_24));
    set_property(*pType, stream_meta_type_image::PixelWidth, static_cast<tInt>(m_streamWidth));
    set_property(*pType, stream_meta_type_image::PixelHeight, static_cast<tInt>(m_streamHeight));
    set_property(*pType, stream_meta_type_image::MaxByteSize, m_maxByteSize);

    //Register Output Pin
    RETURN_IF_FAILED(create_pin(*this, m_oOut, "video_rgb", pType));

    RETURN_NOERROR;
}

tResult cBaslerCamera::StartStreaming()
{

    RETURN_IF_FAILED(cSampleStreamingSource::StartStreaming());
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    m_oTimer = kernel_timer(cString(get_named_graph_object_full_name(*this) + "::frame_timer"), m_nFrameDelay, 0,
        &cBaslerCamera::TimerFunc,
        this);
    if (!m_oTimer.Stoppable())
    {
        RETURN_ERROR_DESC(ERR_UNEXPECTED, "Unable to create kernel timer");
    }

    m_maxByteSize = 3 * m_streamHeight * m_streamWidth;

    //Iinitialize Pylon 
    try
    {
        PylonInitialize();
    }
    catch (GenICam::GenericException &e)
    {
        RETURN_ERROR_DESC(ERR_NOT_CONNECTED, cString::Format("Pylon not Initialized: %s", e.GetDescription()));
    }
    //Open connection to camera
    try
    {
        // Only look for cameras supported by Camera_t. 
        CDeviceInfo info;
        info.SetDeviceClass(CBaslerUsbInstantCamera::DeviceClass());

        // Create an instant camera object with the first found camera device that matches the specified device class.
        m_camera.Attach(CTlFactory::GetInstance().CreateFirstDevice(info));

        m_camera.Open();
        //Pixel Format for camera Output
        m_camera.PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_RGB8);
        //Setting Camera options from Properties
        m_camera.Width.SetValue(m_streamWidth);
        m_camera.Height.SetValue(m_streamHeight);
        m_camera.AutoFunctionROIWidth.SetValue(m_width);
        m_camera.AutoFunctionROIHeight.SetValue(m_height);
        m_camera.AutoFunctionROIOffsetX.SetValue(m_xOffset);
        m_camera.AutoFunctionROIOffsetY.SetValue(m_yOffset);
        m_camera.AutoTargetBrightness.SetValue(m_brightness);
        m_camera.BslColorSpaceMode.SetValue(Basler_UsbCameraParams::BslColorSpaceMode_RGB);
        //m_camera.ColorSpace.SetValue(Basler_UsbCameraParams::ColorSpace_RGB);
    }
    catch (GenICam::GenericException &e)
    {
        RETURN_ERROR_DESC(ERR_NOT_CONNECTED, cString::Format("Camera could not be initialized: %s", e.GetDescription()));
    }
    //Start grabbing with camera
    try
    {
        m_camera.StartGrabbing(GrabStrategy_LatestImageOnly);
    }
    catch (GenICam::GenericException &e)
    {
        RETURN_ERROR_DESC(ERR_FAILED, cString::Format("Camera cannot start grabbing: %s", e.GetDescription()));
    }

    RETURN_NOERROR;
}

tResult cBaslerCamera::StopStreaming()
{
    m_oTimer.Stop();

    //Stop Camera
    m_camera.StopGrabbing();
    if (m_camera.IsOpen())
    {
        //close the camera
        m_camera.Close();

        //detach device
        m_camera.DetachDevice();

        //destroy device
        m_camera.DestroyDevice();
    }
    //Terminate pylon sdk
    try
    {
        PylonTerminate();
    }
    catch (GenICam::GenericException &e)
    {
        RETURN_ERROR_DESC(ERR_FAILED, cString::Format("Pylon not deinitialized: %s", e.GetDescription()));
    }

    return cSampleStreamingSource::StopStreaming();
}

tVoid cBaslerCamera::TimerFunc()
{
    //object_ptr<const ISample> pSample = CreateNewFrame();
    
    // check for device
    if (m_camera.IsOpen() && m_camera.IsGrabbing())
    {
        object_ptr<ISample> pSample;
        if (IS_OK(alloc_sample(pSample)))
        {
            object_ptr_locked<ISampleBuffer> pBuffer;
            if (IS_OK(pSample->WriteLock(pBuffer, m_maxByteSize))) //Florian Denk 23.04.2018: RGB mal 3?
            {

                //PTR for the Result
                CGrabResultPtr ptrGrabResult;

                //Waiting up to 5000ms to get Result
                try
                {
                    m_camera.RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);
                    if (ptrGrabResult->GrabSucceeded())
                    {
                        pBuffer->Write(adtf_memory_buffer<void, size_t>(ptrGrabResult->GetBuffer(), ptrGrabResult->GetImageSize()));

                        m_oOut << pSample << trigger;
                    }
                    ptrGrabResult.Release();

                }
                catch (GenICam::GenericException &e)
                {
                    // Error handling.
                    LOG_ERROR(cString::Format("An exception occurred.  %s", e.GetDescription()));
                }

                pBuffer->Unlock();
                pSample->SetTime(m_pClock->GetStreamTime());
            }
        }
    }

    // if (!pSample)
    // {
    //     m_oOut << ERR_BAD_DEVICE;
    // }
}

int i = 0;

object_ptr<ISample> cBaslerCamera::CreateNewFrame()
{
    object_ptr<ISample> pSample;

    

    return pSample;
}