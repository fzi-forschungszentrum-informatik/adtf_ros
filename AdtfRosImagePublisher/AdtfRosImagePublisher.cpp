// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include "stdafx.h"
#include "AdtfRosImagePublisher.h"
#include <ADTF3_OpenCV_helper.h>


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ROS_IMAGE_PUBLISHER_TRIGGERED_FILTER,
                                    "ROS Image Publisher",
                                    cAdtfRosImagePublisher,
                                    adtf::filter::pin_trigger({ "input" }));

cAdtfRosImagePublisher::cAdtfRosImagePublisher()
{
    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oReader, "input", pType);
    Register(m_oWriter, "undistorted", pType);

    RegisterPropertyVariable("topic", m_topic);
    RegisterPropertyVariable("Fisheye calibration file (optional)", m_calibFile);

    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        if (m_sImageFormat.m_strFormatName == "R(8)G(8)B(8)") {
            m_encoding = "rgb8";
        } else {
            m_encoding = "bgr8";
        }
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter);
    });
}

tResult cAdtfRosImagePublisher::Configure()
{
    cFilename fileCalibration = m_calibFile;

    // Init fisheye
    if (false && !fileCalibration.IsEmpty())
    {
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

            m_undistActive = true;
        }
    }

    //Init ros
    char *argv[] = {(char*)"cAdtfRosImagePublisher"};
    int argc = sizeof(argv) / sizeof(char*) - 1;
    
    ros::init(argc, argv, "cAdtfRosImagePublisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
	m_pub = it.advertise(cString(m_topic).GetPtr(), 1); // TODO: Kadabra uses 1 here, but does it make sense?

    RETURN_NOERROR;
}

tResult cAdtfRosImagePublisher::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;

    while (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        auto header = std_msgs::Header();
        header.stamp = ros::Time::now();
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;

        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {

            //create a opencv matrix from the media sample buffer
            Mat currentFrame = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height), 
                CV_8UC3, (uchar*) pReadBuffer->GetPtr());

            if (m_undistActive) {
                if (!m_rectifyMapsSet) {
                    BuildRectifyMaps();
                }
                Undistort(currentFrame, currentFrame);
            }

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, m_encoding, currentFrame).toImageMsg();

            m_pub.publish(msg);

            // Also publish undist back to ADTF when undistoring

            if (m_undistActive) {
                if (currentFrame.total() * currentFrame.elemSize() != m_sImageFormat.m_szMaxByteSize)
                {
                    setTypeFromMat(m_oWriter, currentFrame);
                    // write to pin
                    writeMatToPin(m_oWriter, currentFrame, pReadSample->GetTime());
                }
            }
            ros::spinOnce();
            
        }
    }
    
    RETURN_NOERROR;
}

tResult cAdtfRosImagePublisher::Undistort(cv::Mat& input, cv::Mat& output) {

    cv::cuda::Stream stream;

    //upload the initial image
    m_GpuImgUpload.upload(input, stream);

    //first operation is remaping, note: No in place operations on the gpu!
    cv::cuda::remap(m_GpuImgUpload,
        m_GpuImgRemap,
        m_GpuRectifyMap1, m_GpuRectifyMap2,
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT, cv::Scalar::all(0),
        stream);

    //second is the resize
    // if (m_fScaling != 1.0f)
    // {
    //     cuda::resize(m_GpuImgRemaped,
    //         m_GpuImgDownload,
    //         cv::Size(), m_fScaling, m_fScaling,
    //         cv::INTER_LINEAR,
    //         stream);

    //     //download the image to cpu mat
    //     m_GpuImgDownload.download(outputImage, stream);
    // } else {
    //     //download the image to cpu mat
    //     m_GpuImgRemaped.download(outputImage, stream);
    // }
    m_GpuImgRemap.download(output, stream);

    //wait for all events to finish.
    stream.waitForCompletion();

    RETURN_NOERROR;
}

tResult cAdtfRosImagePublisher::BuildRectifyMaps() {
    //estimate the new, undistorted camera matrix
    cv::Mat newCameraMatrix;
    
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        m_cameraMatrix,
        m_distorsionMatrix,
        cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
        cv::Matx33d::eye(),
        newCameraMatrix,
        1.0,
        cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
        1.0
    );

    cv::fisheye::initUndistortRectifyMap(
        m_cameraMatrix,
        m_distorsionMatrix,
        cv::Matx33d::eye(),
        newCameraMatrix,
        cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
        CV_16SC2,
        m_rectifyMap1,
        m_rectifyMap2
    );
    
    //GPU Processing
    //convert maps to single channel floating point (used for cuda remap)
    cv::Mat mapX_32FC1, mapY_32FC1;
    cv::convertMaps(m_rectifyMap1, m_rectifyMap2, mapX_32FC1, mapY_32FC1, CV_32FC1);

    //upload the maps
    m_GpuRectifyMap1.upload(mapX_32FC1);
    m_GpuRectifyMap2.upload(mapY_32FC1);

    m_rectifyMapsSet = true;

    RETURN_NOERROR;
}