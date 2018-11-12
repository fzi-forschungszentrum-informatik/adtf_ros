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
#include "AdtfRosAllInOneImagePublisher.h"
#include <ADTF3_OpenCV_helper.h>


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ROS_ALL_IN_ONE_IMAGE_PUBLISHER_TRIGGERED_FILTER,
                                    "ROS All in one Image Publisher",
                                    cAdtfRosAllInOneImagePublisher,
                                    adtf::filter::pin_trigger({ "basler_rgb" }));

cAdtfRosAllInOneImagePublisher::cAdtfRosAllInOneImagePublisher()
{
    adtf::ucom::object_ptr<adtf::streaming::IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    }
    else
    {
        LOG_WARNING("No mediadescription for tPosition found!");
    }

    Register(m_oPositionReader, "position", pTypePositionData);

    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oImageReader, "basler_rgb", pType);
    Register(m_oWriter, "undistorted", pType);

    RegisterPropertyVariable("topic front cam", m_topicFront);
    RegisterPropertyVariable("topic birdview", m_topicBirdview);
    RegisterPropertyVariable("topic bv with pos", m_topicBirdviewPosition);
    RegisterPropertyVariable("BirdviewCal.yml", m_calibFile);

    m_oImageReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        if (m_sImageFormat.m_strFormatName == "R(8)G(8)B(8)") {
            m_encoding = "rgb8";
        } else {
            m_encoding = "bgr8";
        }
        return ChangeType(m_oImageReader, m_sImageFormat, *pType.Get(), m_oWriter);
    });
}

tResult cAdtfRosAllInOneImagePublisher::Configure()
{
    cFilename fileCalibration = m_calibFile;

    // Init fisheye
    if (fileCalibration.IsEmpty() || !(cFileSystem::Exists(fileCalibration)))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Calibration File %s for camera not found", fileCalibration.GetPtr()));
    } else {
        // read the calibration file with camera paramters exits and save to member variable
        cv::FileStorage camera_data(fileCalibration.GetPtr(), cv::FileStorage::READ);
        camera_data["camera_matrix"] >> m_cameraMatrix;
        camera_data["distortion_coefficients"] >> m_distorsionMatrix;
        camera_data["warpMatrix"] >> m_warpMatrix;

        m_birdviewConverter.loadConfig(fileCalibration.GetPtr());
    }

    //Init ros
    char *argv[] = {(char*)"cAdtfRosAllInOneImagePublisher"};
    int argc = sizeof(argv) / sizeof(char*) - 1;
    
    ros::init(argc, argv, "cAdtfRosAllInOneImagePublisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
	m_frontPublisher = it.advertise(cString(m_topicFront).GetPtr(), 1);
	m_birdviewPublisher = it.advertise(cString(m_topicBirdview).GetPtr(), 1);

    m_birdviewPositionPublisher = nh.advertise<ros_oadrive::ImagePosition>(cString(m_topicBirdviewPosition).GetPtr(), 1);

    RETURN_NOERROR;
}

#include <chrono> // TODO: REMOVE ME
#define START_TIMER(t1) auto t1 = std::chrono::system_clock::now();
#define END_TIMER(t1, name) std::cout << "[TIME] " << name << ": " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - t1).count() << "ms" << std::endl;

tResult cAdtfRosAllInOneImagePublisher::Process(tTimeStamp tmTimeOfTrigger)
{
    auto stamp = ros::Time::now();
    cv::Mat undistorted;
    cv::Mat undistortedLow;
    cv::Mat birdview;

    object_ptr<const ISample> pPositionSample;
    // Get the last position 
    while (IS_OK(m_oPositionReader.GetNextSample(pPositionSample)))
    {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(pPositionSample);

        m_lastPose.x = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.x);
        m_lastPose.y = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.y);
        m_lastPose.theta = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.heading);
    }

    object_ptr<const ISample> pReadSample;
    // Get the last image
    while (IS_OK(m_oImageReader.GetNextSample(pReadSample)))
    {
        // START_TIMER(t1)
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;

        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat currentFrame = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height), 
                CV_8UC3, (uchar*) pReadBuffer->GetPtr());

            if (!m_rectifyMapsSet) {
                BuildRectifyMaps();
            }

            cv::cuda::Stream stream;
            // Step 0: upload image to gpu
            m_GpuImgUpload.upload(currentFrame, stream);

            // Step 1: Undistort Image
            cv::cuda::remap(m_GpuImgUpload,
                m_GpuImgRemapHigh,
                m_GpuRectifyMap1, m_GpuRectifyMap2,
                cv::INTER_LINEAR,
                cv::BORDER_CONSTANT, cv::Scalar::all(0),
                stream);

            // Step 2: Resize Image (Not used at the moment)
            cv::cuda::resize(m_GpuImgRemapHigh, m_GpuImgRemapLow, cv::Size(640, 480), 0, 0, INTER_LINEAR, stream);
            
            // Step 3: Create Birdview
            cv::cuda::warpPerspective(m_GpuImgRemapHigh, m_GpuImgBirdviewHigh, m_warpMatrix, currentFrame.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(), stream);
            // Somewhere seems to be a bug that causes empty images here .. 
            // so i copied the one-liner here - m_birdviewConverter.transform(m_GpuImgRemapHigh, m_GpuImgBirdviewHigh, currentFrame.size(), stream);

            // Step 4: Resize Birdview
            cv::cuda::resize(m_GpuImgBirdviewHigh, m_GpuImgBirdviewLow, cv::Size(640, 480), 0, 0, INTER_LINEAR, stream);

            // Step 5: Download Front
            m_GpuImgRemapHigh.download(undistorted, stream);
            m_GpuImgRemapLow.download(undistortedLow, stream);

            // Step 6: Download Birdview
            m_GpuImgBirdviewLow.download(birdview, stream);

            // Step 7: wait for all events to finish.
            stream.waitForCompletion();

            auto header = std_msgs::Header();
            header.stamp = stamp;

            // publish birdview in low res with position to ros
            sensor_msgs::ImagePtr birdviewMsg = cv_bridge::CvImage(header, m_encoding, birdview).toImageMsg();

            ros_oadrive::ImagePosition msg;
            msg.pose = m_lastPose;
            msg.image = *birdviewMsg;
            m_birdviewPositionPublisher.publish(msg);

            // publish front cam in high res to adtf (for marker detection)
            if (!undistorted.empty())
            {
                if (undistorted.total() * undistorted.elemSize() != m_sImageFormat.m_szMaxByteSize)
                {
                    setTypeFromMat(m_oWriter, undistorted);
                }
                // write to pin
                writeMatToPin(m_oWriter, undistorted, pReadSample->GetTime());
            }

            // publish front cam in low res to ros
            sensor_msgs::ImagePtr frontMsg = cv_bridge::CvImage(header, m_encoding, undistortedLow).toImageMsg();
            m_frontPublisher.publish(frontMsg);

            // publish birdview in low res to ros
            m_birdviewPublisher.publish(birdviewMsg);

            ros::spinOnce();
            // END_TIMER(t1, "image pipeline")
        }
    }
    
    RETURN_NOERROR;
}

tResult cAdtfRosAllInOneImagePublisher::BuildRectifyMaps() {
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