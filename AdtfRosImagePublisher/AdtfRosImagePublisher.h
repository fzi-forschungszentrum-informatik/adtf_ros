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
#define CID_ROS_IMAGE_PUBLISHER_TRIGGERED_FILTER "ros_image_publisher.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

class cAdtfRosImagePublisher : public cTriggerFunction
{
private:
    //Pins
    cPinReader m_oReader;
    cPinWriter m_oWriter;
    
    //Stream Formats
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    // Properties
    property_variable<cString> m_topic = cString("aadc/image_raw");
    adtf::base::property_variable<cFilename> m_calibFile = cFilename(cString(""));


    // For fisheye undistortion
    tBool m_undistActive = tFalse;
    tBool m_rectifyMapsSet = tFalse;
    cv::Mat m_cameraMatrix;
    cv::Mat m_distorsionMatrix;
    cv::Mat m_rectifyMap1;
    cv::Mat m_rectifyMap2;
    cv::cuda::GpuMat m_GpuRectifyMap1;
    cv::cuda::GpuMat m_GpuRectifyMap2;
    cv::cuda::GpuMat m_GpuImgUpload;
    cv::cuda::GpuMat m_GpuImgRemap;

    image_transport::Publisher m_pub;
    std::string m_encoding;
public:
    cAdtfRosImagePublisher();
    virtual ~cAdtfRosImagePublisher() = default;

    tResult Configure() override;

    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    tResult BuildRectifyMaps();
    tResult Undistort(cv::Mat& input, cv::Mat& output);

};