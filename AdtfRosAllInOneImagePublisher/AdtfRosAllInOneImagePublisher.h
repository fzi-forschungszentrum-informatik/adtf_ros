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

#pragma once
#define CID_ROS_ALL_IN_ONE_IMAGE_PUBLISHER_TRIGGERED_FILTER "ros__all_in_one_image_publisher.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

class cAdtfRosAllInOneImagePublisher : public cTriggerFunction
{
private:
    //Pins
    cPinReader m_oImageReader;
    cPinWriter m_oWriter;
    cPinReader m_oPositionReader;

    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    }  m_ddlPositionIndex;

    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;
    
    //Stream Formats
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    // Properties
    property_variable<cString> m_topicFront = cString("aadc/front_cam");
    property_variable<cString> m_topicBirdview = cString("aadc/birdview");
    property_variable<cString> m_topicBirdviewPosition = cString("aadc/front/birdview");
    adtf::base::property_variable<cFilename> m_calibFile = cFilename(cString("/home/aadc/robot_folders/checkout/aadc18/ic_workspace/packages/oadrive/config/Alpacapone/BirdviewCal.yml"));


    // For fisheye undistortion
    tBool m_undistActive = tFalse;
    tBool m_rectifyMapsSet = tFalse;
    cv::Mat m_cameraMatrix;
    cv::Mat m_warpMatrix;
    cv::Mat m_distorsionMatrix;
    cv::Mat m_rectifyMap1;
    cv::Mat m_rectifyMap2;
    cv::cuda::GpuMat m_GpuRectifyMap1;
    cv::cuda::GpuMat m_GpuRectifyMap2;
    cv::cuda::GpuMat m_GpuImgUpload;
    cv::cuda::GpuMat m_GpuImgRemapHigh;
    cv::cuda::GpuMat m_GpuImgRemapLow;
    cv::cuda::GpuMat m_GpuImgBirdviewHigh;
    cv::cuda::GpuMat m_GpuImgBirdviewLow;

    // ROS:
    geometry_msgs::Pose2D m_lastPose;

    image_transport::Publisher m_frontPublisher;
    image_transport::Publisher m_birdviewPublisher;
    ros::Publisher m_birdviewPositionPublisher;
    std::string m_encoding;

    oadrive::util::BirdViewConverter m_birdviewConverter;
public:
    cAdtfRosAllInOneImagePublisher();
    virtual ~cAdtfRosAllInOneImagePublisher() = default;

    tResult Configure() override;

    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    tResult BuildRectifyMaps();
    tResult Undistort(cv::Mat& input, cv::Mat& output);

};