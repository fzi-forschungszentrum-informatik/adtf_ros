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
#define CID_ROS_POSITION_PUBLISHER_TRIGGERED_FILTER "ros_position_publisher.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cAdtfRosPositionPublisher : public cTriggerFunction
{
private:
    //Pins
    cPinReader m_oReaderLocal;
    cPinReader m_oReaderGlobal;

    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    }  m_ddlPositionIndex;

    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;

    // Properties
    property_variable<cString> m_topic = cString("aadc/pose_2d");
    property_variable<cString> m_topicExtended = cString("aadc/marker_position");
    property_variable<cString> m_topicExtendedWorld = cString("aadc/marker_position_world");
    property_variable<cString> m_tfWorld = cString("world");
    property_variable<cString> m_tfLocal = cString("local");
    property_variable<cString> m_tfCar = cString("car");
    property_variable<cString> m_tfOrientedCar = cString("oriented_car");

    ros::Publisher m_pubWorld;
    ros::Publisher m_pub;
    ros::Publisher m_pubExtended;
    tf::TransformBroadcaster* m_broadcaster;

    ros_oadrive::Pose2DStamped m_lastPose;
public:
    cAdtfRosPositionPublisher();
    virtual ~cAdtfRosPositionPublisher() = default;

    tResult Configure() override;

    tResult Process(tTimeStamp tmTimeOfTrigger) override;

};