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
#define CID_ROS_OPENDRIVE_PUBLISHER_TRIGGERED_FILTER "ros_opendrive_publisher.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cAdtfRosOpendrivePublisher : public cTriggerFunction
{
private:
    //Pins
    cPinReader m_oReader;

    struct tSignalValueId
    {
        tSize ui32ArduinoTimestamp;
        tSize f32Value;
    };

    struct
    {
        tSignalValueId tSideLeft;
        tSignalValueId tSideRight;
        tSignalValueId tRearLeft;
        tSignalValueId tRearCenter;
        tSignalValueId tRearRight;
    } m_ddlUltrasonicStructIndex;

    adtf::mediadescription::cSampleCodecFactory m_USDataSampleFactory;

    // Properties
    property_variable<cString> m_topic = cString("aadc/opendrive_map");

    ros::Publisher m_pub;
public:
    cAdtfRosOpendrivePublisher();
    virtual ~cAdtfRosOpendrivePublisher() = default;

    tResult Configure() override;

    tResult Process(tTimeStamp tmTimeOfTrigger) override;

};