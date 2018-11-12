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
#include "RosAdtfCarController.h"
#include <ADTF3_helper.h>
#include <oadrive_util/Config.h>


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ROS_ADTF_CAR_CONTROLLER_TRIGGERED_FILTER,
                                    "ROS Adtf Car Controller",
                                    cRosAdtfCarController,
                                    adtf::filter::timer_trigger(1));

cRosAdtfCarController::cRosAdtfCarController()
{
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    Register(m_oOutputSteeringController, "steering", pTypeSignalValue);
    Register(m_oOutputSpeedController, "speed", pTypeSignalValue);

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue")              , m_ddlBoolSignalValueId.bValue));
    }
    else
    {
        LOG_INFO("No mediadescription for tBoolSignalValue found!");
    }
    Register(m_oOutputHeadLight, "head_light", pTypeBoolSignalValue);
    Register(m_oOutputTurnLeft, "turn_signal_left", pTypeBoolSignalValue);
    Register(m_oOutputTurnRight, "turn_signal_right", pTypeBoolSignalValue);
    Register(m_oOutputBrakeLight, "brake_light", pTypeBoolSignalValue);
    Register(m_oOutputHazard, "hazard_light", pTypeBoolSignalValue);
    Register(m_oOutputReverseLight, "reverse_light", pTypeBoolSignalValue);

    Register(m_oReaderWheelSpeed, "wheel_speed", pTypeSignalValue);


    //the imu struct
    object_ptr<IStreamType> pTypeIMUData;

    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tInerMeasUnitData", pTypeIMUData, m_IMUDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "ui32ArduinoTimestamp", m_ddlInerMeasUnitDataIndex.timeStamp);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_x", m_ddlInerMeasUnitDataIndex.A_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_y", m_ddlInerMeasUnitDataIndex.A_y);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_z", m_ddlInerMeasUnitDataIndex.A_z);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_x", m_ddlInerMeasUnitDataIndex.G_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_y", m_ddlInerMeasUnitDataIndex.G_y);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_z", m_ddlInerMeasUnitDataIndex.G_z);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_x", m_ddlInerMeasUnitDataIndex.M_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_y", m_ddlInerMeasUnitDataIndex.M_y);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_z", m_ddlInerMeasUnitDataIndex.M_z);

        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32roll", m_ddlInerMeasUnitDataIndex.roll);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32pitch", m_ddlInerMeasUnitDataIndex.pitch);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32yaw", m_ddlInerMeasUnitDataIndex.yaw);
    }
    else
    {
        LOG_WARNING("No mediadescription for tInerMeasUnitData found!");
    }
    
    object_ptr<const IStreamType> pConstTypeIMUData = pTypeIMUData;

    Register(m_oReaderIMU, "imu", pConstTypeIMUData);

    // oadrive::util::Config::setConfigPath("/home/aadc/robot_folders/checkout/aadc18/ic_workspace/packages/oadrive/config", "Alpacalypse");
}

tResult cRosAdtfCarController::Configure()
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    // publish default config

    char *argv[] = {(char*)"cRosAdtfCarController"};
    int argc = sizeof(argv) / sizeof(char*) - 1;
    
    ros::init(argc, argv, "cRosAdtfCarController");
    ros::NodeHandle nh;
    m_callbackQueue = new ros::CallbackQueue();
    nh.setCallbackQueue(m_callbackQueue);

    m_controlSub = nh.subscribe("/aadc/control/car", 1, &cRosAdtfCarController::CarControlCallback, this);
    m_lightStatusSub = nh.subscribe("/aadc/control/light_status", 1, &cRosAdtfCarController::LightStatusUpdateCallback, this);

    // Get steering slope
    // Estimated steering curve: steering = m*servo_steering
    const char * carName = ::getenv("AADC_CONFIG_CAR_NAME");
    if ( carName == 0 ) 
      carName = "";
    if( strcmp(carName, "Simsala") == 0)
      m_steeringSlope = 0.286636636174348; 
    else if( strcmp(carName, "Abra") == 0)
      m_steeringSlope = 0.303686711355509; 
    else
    {
      LOG_WARNING("No car name found in env 'AADC_CONFIG_CAR_NAME'! Using default value");
      m_steeringSlope = 0.3;
    }

    RETURN_NOERROR;
}

tResult cRosAdtfCarController::Process(tTimeStamp tmTimeOfTrigger)
{  

    if (m_firstRun) {
        // Send initial status on first run to get the controller synced with our internal state
        m_firstRun = false;
        m_lastLight.headLightsOn = true;

        TransmitBool(m_oOutputHeadLight, m_lastLight.headLightsOn);
	    TransmitBool(m_oOutputBrakeLight, m_lastLight.breakingLightsOn);
	    TransmitBool(m_oOutputReverseLight, m_lastLight.reverseLightsOn);
	    TransmitBool(m_oOutputTurnLeft, m_lastLight.turnLeftLightsOn);
	    TransmitBool(m_oOutputTurnRight, m_lastLight.turnRightLightsOn);
	    TransmitBool(m_oOutputHazard, m_lastLight.hazardLightsOn);
    }

    m_callbackQueue->callAvailable();

    // get current speed
    adtf::ucom::object_ptr<const adtf::streaming::ISample> pReadSample;
    while (IS_OK(m_oReaderWheelSpeed.GetNextSample(pReadSample)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);
        m_lastSpeed = adtf_ddl::access_element::get_value(oDecoder, m_ddlSignalValueId.value);
    }

    // Get imu
    object_ptr<const ISample> pIMUReadSample;

    while (IS_OK(m_oReaderIMU.GetNextSample(pIMUReadSample))) {
        ProcessInerMeasUnitSample(tmTimeOfTrigger, *pIMUReadSample);
    }

    controlSpeed(m_lastSpeed);

    if (m_active && std::time(NULL) - m_lastTimestamp > m_timeoutTime) {
        // Stop the car!
        m_lastSpeed = m_targetSpeed = 0.0;
        TransmitThrottle(0.0);
        LOG_INFO("TimeOut: car stopped");
    }

    RETURN_NOERROR;
}

tResult cRosAdtfCarController::ProcessInerMeasUnitSample(tTimeStamp tmTimeOfTrigger, const adtf::streaming::ISample &oSample)
{
    // parse the data 
    auto oDecoder = m_IMUDataSampleFactory.MakeDecoderFor(oSample);
    // const float currentPitch = access_element::get_value(oDecoder, m_ddlInerMeasUnitDataIndex.A_x);

    RETURN_NOERROR;
}

void cRosAdtfCarController::controlSpeed(float currentSpeed) {
    m_speedController.addSpeedSample(currentSpeed);
    float servoSpeed = m_speedController.update(m_targetSpeed, m_lastSteering, false);

    // LOG_INFO(cString::Format("Target: %f, Current: %f, Servo: %f", m_targetSpeed, currentSpeed, servoSpeed));
    
    float speedLimit = 60.f;// + fabs(m_lastSteering) / 5.0;

    // backward
    if(servoSpeed > speedLimit) servoSpeed = speedLimit;
    // forward
    if(servoSpeed < -speedLimit) servoSpeed = -speedLimit;

    // Boost mode
    // if (m_targetSpeed >= 4.0) servoSpeed = 40.f;

    // Enter a safe state, when the controlled speed gets nan
    if (std::isnan(servoSpeed)) {
        LOG_ERROR("Servo Speed was NaN!");
        servoSpeed = 0.0;
    }


    // float servoSpeed = 0;
    // if (fabs(m_targetSpeed) < 0.2) {
    //     servoSpeed = 0.0;
    // } else if (fabs(m_targetSpeed) < 0.9) {
    //     servoSpeed = 23.0;
    // } else if (fabs(m_targetSpeed) < 1.2) {
    //     servoSpeed = 25.0;
    // } else {
    //     servoSpeed = 35.0;
    // }

    // if (m_targetSpeed < 0) {
    //     servoSpeed *= -1;
    // }

    //LOG_INFO(cString::Format("Speed: %f of max %f", servoSpeed, speedLimit));

    TransmitThrottle(servoSpeed);
}


void cRosAdtfCarController::LightStatusUpdateCallback(const ros_oadrive::LightStatus msg)
{
    // Only send the bool values if the values change
    if(m_lastLight.headLightsOn != msg.headLightsOn) {
	    m_lastLight.headLightsOn = msg.headLightsOn;
	    TransmitBool(m_oOutputHeadLight, m_lastLight.headLightsOn);
    }
    if(m_lastLight.breakingLightsOn != msg.breakingLightsOn) {
	    m_lastLight.breakingLightsOn = msg.breakingLightsOn;
	    TransmitBool(m_oOutputBrakeLight, m_lastLight.breakingLightsOn);
    }
    if(m_lastLight.reverseLightsOn != msg.reverseLightsOn) {
	    m_lastLight.reverseLightsOn = msg.reverseLightsOn;
	    TransmitBool(m_oOutputReverseLight, m_lastLight.reverseLightsOn);
    }
    if(m_lastLight.turnLeftLightsOn != msg.turnLeftLightsOn) {
	    m_lastLight.turnLeftLightsOn = msg.turnLeftLightsOn;
	    TransmitBool(m_oOutputTurnLeft, m_lastLight.turnLeftLightsOn);
    }
    if (m_lastLight.turnRightLightsOn != msg.turnRightLightsOn) {
	    m_lastLight.turnRightLightsOn = msg.turnRightLightsOn;
	    TransmitBool(m_oOutputTurnRight, m_lastLight.turnRightLightsOn);
    }
    if (m_lastLight.hazardLightsOn != msg.hazardLightsOn) {
	    m_lastLight.hazardLightsOn = msg.hazardLightsOn;
	    TransmitBool(m_oOutputHazard, m_lastLight.hazardLightsOn);
    }
}

void cRosAdtfCarController::TransmitBool(cPinWriter& outputPin, tBool boolValue) {
    transmitBoolSignalValue(outputPin, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, boolValue);
}


tResult cRosAdtfCarController::TransmitSteering(tFloat32 value)
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    m_lastSteering = value;

    transmitSignalValue(m_oOutputSteeringController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, value);

    RETURN_NOERROR;
}

tResult cRosAdtfCarController::TransmitThrottle(tFloat32 value)
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    m_active = value > 0;

    transmitSignalValue(m_oOutputSpeedController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, value);

    RETURN_NOERROR;
}

void cRosAdtfCarController::CarControlCallback(const ackermann_msgs::AckermannDrive::ConstPtr& ackermann_msg)
{
    // store the timestamp of the last received control msg
    m_lastTimestamp = std::time(NULL);
    m_targetSpeed = static_cast<float> (ackermann_msg->speed);

    float steering = static_cast<float>(ackermann_msg->steering_angle);
    float servo_steering = 0;

    // convert from ackermann_msgs steering (in radians) to output value (between -100 and 100)
    // Calculatue inverse of steering curve: servo_steering = steering/m (steering is converted to degree first) 
    // servo_steering = -(steering * 180.0f / 3.1415f) / m_steeringSlope; 
    servo_steering = (steering * 180.0f / 3.1415f) / m_steeringSlope; 

    // security checks
    if(servo_steering > 95.0f) servo_steering = 95.0f;
    if(servo_steering < -95.0f) servo_steering = -95.0f;  

    TransmitSteering(servo_steering);
}
