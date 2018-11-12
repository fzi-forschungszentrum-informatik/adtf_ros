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
#define CID_ROS_JURY_COMMUNICATION_TRIGGERED_FILTER "ros_jury_communication.filter.user.aadc.cid"

#include <aadc_structs.h>
#include <aadc_jury.h>

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

using namespace aadc::jury;

class cRosJuryCommunication : public cTriggerFunction
{
private:
    cSampleCodecFactory m_juryStructSampleFactory;

    struct tJuryStructId
    {
        tSize actionId;
        tSize maneuverEntry;
    } m_ddlJuryStructId;

    struct tDriverStructId
    {
        tSize stateId;
        tSize maneuverEntry;
    } m_ddlDriverStructId;

    cSampleCodecFactory m_driverStructSampleFactory;
    cString     m_strManeuverFileString;
    aadc::jury::maneuverList m_sectorList;
    
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    // Properties
    cPinWriter     m_oOutputDriverStruct;
    cPinReader     m_oInputJuryStruct;
    cPinReader     m_oInputManeuverList;
    property_variable<cString> m_topic = cString("aadc/ultrasonic");

    bool m_running = false;
    int m_sectorIdx = -1;
    int m_maneuverIdx = -1;
    tInt16 m_currentManeuverEntry = 0;

    ros::Publisher m_pubManeuver;
    ros::Publisher m_eventPub;
    ros::Publisher m_pubJury;
    
    ros::Subscriber m_eventSub;

    // The callback queue for our controller
    // We need an own queue so other ADTF filters running ros::spin wont invoke this one
    // This might have been an issue resulting in segfaults
    ros::CallbackQueue* m_callbackQueue;
public:
    cRosJuryCommunication();
    virtual ~cRosJuryCommunication() = default;

    tResult Configure() override;

    tResult Process(tTimeStamp tmTimeOfTrigger) override;
    tResult ProcessJuryStruct(aadc::jury::juryAction action, int entry);

    tResult LoadManeuverList();
    tResult TransmitDriverStruct(tDriverStruct& driverStruct);

    // publish methods
    void publishStartUp();
    void publishReady();
    void publishRunning();
    void publishComplete();
    void publishGetReady();
    void publishStart();
    void publishStop();
    void setManeuver(int entry);
    bool incrementManeuver();
    tManeuver getCurrentManeuver();
    void publishCurrentManeuver();

    void eventLogic(const std_msgs::String::ConstPtr& event);

};