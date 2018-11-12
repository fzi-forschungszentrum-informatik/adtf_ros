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
#define CID_ROS_TRAFFIC_SIGN_PUBLISHER_TRIGGERED_FILTER "ros_traffic_sign_publisher.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cAdtfRosTrafficSignPublisher : public cTriggerFunction
{
private:
    //Pins
    cPinReader m_oReader;

    struct
    {
        tSize id;
        tSize size;
        tSize tvec;
        tSize rvec;
    } m_ddlRoadSignIndex;

    adtf::mediadescription::cSampleCodecFactory m_RoadSignSampleFactory;

    // Properties
    property_variable<cString> m_topic = cString("aadc/objects/traffic_sign");

    ros::Publisher m_pub;

    /*! translation vector */
    cv::Mat m_Tvec;
    /*! rotation vector */
    cv::Mat m_Rvec;
public:
    cAdtfRosTrafficSignPublisher();
    virtual ~cAdtfRosTrafficSignPublisher() = default;

    tResult Configure() override;

    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    tResult ProcessInputRoadSign(const adtf::streaming::ISample &pSample);

};