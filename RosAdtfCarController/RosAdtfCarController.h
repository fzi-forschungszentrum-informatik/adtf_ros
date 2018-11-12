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
#define CID_ROS_ADTF_CAR_CONTROLLER_TRIGGERED_FILTER "ros_adtf_car_controller.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cRosAdtfCarController : public cTriggerFunction
{
private:
    //Pins
    cPinReader m_oReaderWheelSpeed;
    cPinReader m_oReaderIMU;

    cPinWriter     m_oOutputSpeedController;
    cPinWriter     m_oOutputSteeringController;

    cPinWriter     m_oOutputTurnRight;
    cPinWriter     m_oOutputTurnLeft;
    cPinWriter     m_oOutputHazard;
    cPinWriter     m_oOutputHeadLight;
    cPinWriter     m_oOutputReverseLight;
    cPinWriter     m_oOutputBrakeLight;


    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    struct tBoolSignalValueId
    {
        tSize ui32ArduinoTimestamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;

    struct
    {
        tSize timeStamp;
        tSize A_x;
        tSize A_y;
        tSize A_z;
        tSize G_x;
        tSize G_y;
        tSize G_z;
        tSize M_x;
        tSize M_y;
        tSize M_z;
        tSize roll;
        tSize pitch;
        tSize yaw;
    } m_ddlInerMeasUnitDataIndex;
    adtf::mediadescription::cSampleCodecFactory m_IMUDataSampleFactory;


    std::mutex m_oMutex;
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    bool m_firstRun = true;
    /**
     * Timeout in seconds
     */
    const int m_timeoutTime = 2;
    ros::Subscriber m_controlSub;
    ros::Subscriber m_lightStatusSub;
    ros_oadrive::LightStatus m_lastLight;
    std::time_t m_lastTimestamp;
    bool m_active = false;

    float m_steeringSlope = 0.0;
    
    oadrive::control::SpeedController m_speedController;
    float m_targetSpeed = 0.0;

    float m_lastSpeed = 0.0;
    float m_lastSteering = 0.0;

    // The callback queue for our controller
    // We need an own queue so other ADTF filters running ros::spin wont invoke this one
    // This might have been an issue resulting in segfaults
    ros::CallbackQueue* m_callbackQueue;
public:
    cRosAdtfCarController();
    virtual ~cRosAdtfCarController() = default;

    tResult Configure() override;

    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    tResult ProcessInerMeasUnitSample(tTimeStamp tmTimeOfTrigger, const adtf::streaming::ISample &oSample);


    void LightStatusUpdateCallback(const ros_oadrive::LightStatus msg);
    void CarControlCallback(const ackermann_msgs::AckermannDrive::ConstPtr& ackermann_msg);

    void controlSpeed(float currentSpeed);
    
    void TransmitBool(cPinWriter& outputPin, tBool boolValue);
    tResult TransmitSteering(tFloat32 value);
    tResult TransmitThrottle(tFloat32 value);

};