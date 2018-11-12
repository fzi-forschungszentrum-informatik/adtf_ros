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
#include "CustomUndistortion.h"
#include "ADTF3_OpenCV_helper.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CCUSTOMUNDISTORTION_DATA_TRIGGERED_FILTER,
    "Custom Undistortion",
    cCustomUndistortion,
    adtf::filter::pin_trigger({ "input" }));

cCustomUndistortion::cCustomUndistortion()
{
    //Register Properties
    RegisterPropertyVariable("calibration file", m_calibFile);


    //create and set inital input format type
    m_sInputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sInputFormat);

    //Register input pin
    Register(m_oReader, "input", pType);
    //Register output pin
    Register(m_oWriter, "output", pType);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sInputFormat, *pType.Get(), m_oWriter);
    });

    //GPU Processing

    m_bGpuProcessing = tFalse;

    try
    {
        if (cv::cuda::getCudaEnabledDeviceCount() > 0)
        {
            m_bGpuAvailable = tTrue;

            //only set the properties if we have a cuda capable gpu.
            RegisterPropertyVariable("GPU processing", m_bGpuProcessing);
            RegisterPropertyVariable("GPU scaling", m_fScaling);
        }
        else
        {
            m_bGpuAvailable = tFalse;
            m_bGpuProcessing = tFalse;
        }
    }
    catch (cv::Exception& e)
    {
        const char* err_msg = e.what();
        LOG_ERROR(cString("OpenCV exception caught: ") + err_msg);
    }

}


tResult cCustomUndistortion::Configure()
{
    cFilename fileCalibration = m_calibFile;

    adtf::services::ant::adtf_resolve_macros(fileCalibration);
    //check if calibration file with camera paramters exits
    if (fileCalibration.IsEmpty())
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Calibration File %s for camera not found", fileCalibration.GetPtr()));
    }

    if (!(cFileSystem::Exists(fileCalibration)))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Calibration File %s for camera not found", fileCalibration.GetPtr()));
    }
    else
    {
        // read the calibration file with camera paramters exits and save to member variable
        cv::FileStorage camera_data(fileCalibration.GetPtr(), cv::FileStorage::READ);
        camera_data["camera_matrix"] >> m_cameraMatrix;
        camera_data["distortion_coefficients"] >> m_distorsionMatrix;
    }


    //check if we have valid matrices
    if (m_cameraMatrix.empty())
    {
        RETURN_ERROR_DESC(ERR_EMPTY, cString("camera_matrix cannot be loaded from file"));
    }

    if (m_distorsionMatrix.empty())
    {
        RETURN_ERROR_DESC(ERR_EMPTY, cString("camera_matrix cannot be loaded from file"));
    }


    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cCustomUndistortion::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    Mat outputImage;

    if (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat inputImage = Mat(cv::Size(m_sInputFormat.m_ui32Width, m_sInputFormat.m_ui32Height),
                CV_8UC3, (uchar*)pReadBuffer->GetPtr());

            //Do the image processing and copy to destination image buffer
            if (!m_rectifyMapsSet)
            {
                try
                {

                    //estimate the new, undistorted camera matrix
                    cv::Mat newCameraMatrix;

                    cv::initUndistortRectifyMap(
                        m_cameraMatrix,
                        m_distorsionMatrix,
                        cv::Matx33d::eye(),
                        m_cameraMatrix,
                        cv::Size(m_sInputFormat.m_ui32Width, m_sInputFormat.m_ui32Height),
                        CV_16SC2,
                        m_rectifyMap1,
                        m_rectifyMap2);
                                                
                    /*cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
                        m_cameraMatrix,
                        m_distorsionMatrix,
                        cv::Size(m_sInputFormat.m_ui32Width, m_sInputFormat.m_ui32Height),
                        cv::Matx33d::eye(),
                        newCameraMatrix,
                        1.0,
                        cv::Size(m_sInputFormat.m_ui32Width, m_sInputFormat.m_ui32Height),
                        1.0
                    );

                    cv::fisheye::initUndistortRectifyMap(
                        m_cameraMatrix,
                        m_distorsionMatrix,
                        cv::Matx33d::eye(),
                        newCameraMatrix,
                        cv::Size(m_sInputFormat.m_ui32Width, m_sInputFormat.m_ui32Height),
                        CV_16SC2,
                        m_rectifyMap1,
                        m_rectifyMap2
                    );*/

                }
                catch (cv::Exception& e)
                {
                    const char* err_msg = e.what();
                    LOG_ERROR(cString("OpenCV exception caught: ") + err_msg);
                }

                //GPU Processing

                if (m_bGpuAvailable && m_bGpuProcessing)
                {
                    //convert maps to single channel floating point (used for cuda remap)
                    cv::Mat mapX_32FC1, mapY_32FC1;
                    cv::convertMaps(m_rectifyMap1, m_rectifyMap2, mapX_32FC1, mapY_32FC1, CV_32FC1);

                    //upload the maps
                    m_GpuRectifyMap1.upload(mapX_32FC1);
                    m_GpuRectifyMap2.upload(mapY_32FC1);
                }

                m_rectifyMapsSet = true;
            }
            if (m_rectifyMapsSet)
            {
                try {
                    //GPU Processing

                    if (!m_bGpuAvailable || !m_bGpuProcessing)
                    {
                        cv::remap(inputImage,
                            outputImage,
                            m_rectifyMap1, m_rectifyMap2,
                            cv::INTER_LINEAR,
                            cv::BORDER_CONSTANT,
                            cv::Scalar::all(0));


                        if (m_fScaling != 1.0f)
                        {
                            cv::resize(inputImage,
                                outputImage,
                                cv::Size(),
                                m_fScaling,
                                m_fScaling,
                                cv::INTER_LINEAR);
                        }
                    }
                    else
                    {
#ifndef WIN32                        
                        //stream used for queueing the instructions on the gpu
                        cv::cuda::Stream stream;

                        //upload the initial image
                        m_GpuImgUpload.upload(inputImage, stream);

                        //first operation is remaping, note: No in place operations on the gpu!
                        cv::cuda::remap(m_GpuImgUpload,
                            m_GpuImgRemaped,
                            m_GpuRectifyMap1, m_GpuRectifyMap2,
                            cv::INTER_LINEAR,
                            cv::BORDER_CONSTANT, cv::Scalar::all(0),
                            stream);

                        //second is the resize
                        cuda::resize(m_GpuImgRemaped,
                            m_GpuImgDownload,
                            cv::Size(), m_fScaling, m_fScaling,
                            cv::INTER_LINEAR,
                            stream);

                        //download the image to cpu mat
                        m_GpuImgDownload.download(outputImage, stream);

                        //wait for all events to finish.
                        stream.waitForCompletion();
#else
                        LOG_ERROR("GPU processing is not supported in windows so far");
#endif
                    }
                }
                catch (cv::Exception& e)
                {
                    LOG_ERROR(cString::Format("CV exception caught: %s", e.what()));
                }
            }
        }

        //Write processed Image to Output Pin
        if (!outputImage.empty())
        {
            //update output format if matrix size does not fit to
            if (outputImage.total() * outputImage.elemSize() != m_sInputFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_oWriter, outputImage);
            }
            // write to pin
            writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
        }
    }

    RETURN_NOERROR;
}
