/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#include "stdafx.h"
#include "aadc_structs.h"
#include "ConverterWheels.h"
#include "ADTF3_helper.h"

#define CW_SLOT_COUNT 60.f
#define CW_ERROR_DIFFERENCE_SIDES 0.30f
#define CW_MIN_LIMIT_IGNORE 0.01f

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CONVERTERWHEELS_DATA_TRIGGERED_FILTER,
    "Better ConverterWheels",
    cConverterWheels,
    adtf::filter::pin_trigger({ "wheel_left" }));

cConverterWheels::cConverterWheels()
{
    //Register Properties
    RegisterPropertyVariable("wheel circumference [m]", m_f32wheelCircumference);
    RegisterPropertyVariable("filter constant of first order", m_f32FilterConstantfirstOrder);
    RegisterPropertyVariable("enable filtering", m_bEnableFiltering);
    RegisterPropertyVariable("plausibilization via direction indicator enable", m_bEnableDirectionPlausibilization);
    RegisterPropertyVariable("direction derived from speed controller actuator value", m_bEnableSpeedControllerDirection);
    RegisterPropertyVariable("deadband for speed controll direction", m_f32SpeedControllerDeadband);

    //the wheel data
    object_ptr<IStreamType> pTypeWheelData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tWheelData", pTypeWheelData, m_WheelDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32ArduinoTimestamp", m_ddlWheelDataIndex.ArduinoTimestamp);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32WheelTach", m_ddlWheelDataIndex.WheelTach);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "i8WheelDir", m_ddlWheelDataIndex.WheelDir);
    }
    else
    {
        LOG_INFO("No mediadescription for tWheelData found!");
    }
    Register(m_oInputWheelLeft, "wheel_left", pTypeWheelData);
    Register(m_oInputWheelRight, "wheel_right", pTypeWheelData);

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
    Register(m_oInputSpeedController, "speed_control", pTypeSignalValue);
    Register(m_oOutputCarSpeed, "vehicle_speed", pTypeSignalValue);
}


//implement the Configure function to read ALL Properties
tResult cConverterWheels::Configure()
{
    RETURN_IF_FAILED(cTriggerFunction::Configure());
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cConverterWheels::Process(tTimeStamp tmTimeOfTrigger)
{

    // Speed Controller
    object_ptr<const ISample> pSampleFromSpeedController;

    while (IS_OK(m_oInputSpeedController.GetNextSample(pSampleFromSpeedController)))
    {
        tUInt32 ui32Timestamp = 0;
        tFloat32 f32Value = 0.0;

        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleFromSpeedController);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &ui32Timestamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

        //update the struct
        m_tLastSpeedControllerValue.f32Value = f32Value;
        m_tLastSpeedControllerValue.ui32ArduinoTimestamp = ui32Timestamp;

    }

    //Wheel Left 
    object_ptr<const ISample> pSampleFromWheelLeft;

    while (IS_OK(m_oInputWheelLeft.GetNextSample(pSampleFromWheelLeft)))
    {
        // save the last struct to the struct beforeLast if it is not the first one
        if (m_bfirstSampleReceivedLeftWheel == tTrue)
        {
            m_tBeforeLastStructLeft = m_tLastStructLeft;
        }

        tUInt32 ui32Tach = 0;
        tInt8   i8Direction = 0;
        tUInt32 ui32Timestamp = 0;

        auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelLeft);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &ui32Timestamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &ui32Tach));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &i8Direction));

        // if it is the first sample stop here and set to true
        if (m_bfirstSampleReceivedLeftWheel == tFalse)
        {
            m_bfirstSampleReceivedLeftWheel = tTrue;

            m_tLastStructLeft.i8WheelDir = i8Direction;
            m_tLastStructLeft.ui32ArduinoTimestamp = ui32Timestamp;
            m_tLastStructLeft.ui32WheelTach = ui32Tach;
        }
        // doing the calculation and the transmit
        else
        {
            // doing an minimal smoothing of the signal
            if (m_bEnableFiltering)
                m_f32LastCalculatedSpeedLeft = m_f32LastCalculatedSpeedLeft +
                m_f32FilterConstantfirstOrder * (calculateSpeed(ui32Timestamp, m_tLastStructLeft.ui32ArduinoTimestamp, ui32Tach - m_tLastStructLeft.ui32WheelTach)
                    - m_f32LastCalculatedSpeedLeft);
            else
                m_f32LastCalculatedSpeedLeft = calculateSpeed(ui32Timestamp, m_tLastStructLeft.ui32ArduinoTimestamp, ui32Tach - m_tLastStructLeft.ui32WheelTach);

            m_tLastStructLeft.i8WheelDir = i8Direction;
            m_tLastStructLeft.ui32ArduinoTimestamp = ui32Timestamp;
            m_tLastStructLeft.ui32WheelTach = ui32Tach;
        }
    }



    //Wheel Right
    object_ptr<const ISample> pSampleFromWheelRight;

    while (IS_OK(m_oInputWheelRight.GetNextSample(pSampleFromWheelRight)))
    {
        // save the last struct to the struct beforeLast if it is not the first one
        if (m_bfirstSampleReceivedRightWheel == tTrue)
        {
            m_tBeforeLastStructRight = m_tLastStructRight;
        }

        tUInt32 ui32Tach = 0;
        tInt8 i8Direction = 0;
        tUInt32 ui32Timestamp = 0;

        {

            auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelRight);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &ui32Timestamp));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &ui32Tach));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &i8Direction));

        }

        // if it is the first sample stop here and set to true
        if (m_bfirstSampleReceivedRightWheel == tFalse)
        {
            m_bfirstSampleReceivedRightWheel = tTrue;

            m_tLastStructRight.i8WheelDir = i8Direction;
            m_tLastStructRight.ui32ArduinoTimestamp = ui32Timestamp;
            m_tLastStructRight.ui32WheelTach = ui32Tach;
        }
        // doing the calculation and the transmit
        else
        {
            // doing an minimal smoothing of the signal
            if (m_bEnableFiltering)
                m_f32LastCalculatedSpeedRight = m_f32LastCalculatedSpeedRight + m_f32FilterConstantfirstOrder * (calculateSpeed(ui32Timestamp, m_tLastStructRight.ui32ArduinoTimestamp, ui32Tach - m_tLastStructRight.ui32WheelTach) - m_f32LastCalculatedSpeedRight);
            else
                m_f32LastCalculatedSpeedRight = calculateSpeed(ui32Timestamp, m_tLastStructRight.ui32ArduinoTimestamp, ui32Tach - m_tLastStructRight.ui32WheelTach);

            m_tLastStructRight.i8WheelDir = i8Direction;
            m_tLastStructRight.ui32ArduinoTimestamp = ui32Timestamp;
            m_tLastStructRight.ui32WheelTach = ui32Tach;

            unsigned int temp = 0;
            if (m_bEnableSpeedControllerDirection)// ADTF2:  && m_bIDsSpeedControllerSet)
            {
                //update the direction only when we have a controll value!
                if (m_tLastSpeedControllerValue.f32Value > m_f32SpeedControllerDeadband)
                {
                    m_i8olddirection = 0;
                }
                else if (m_tLastSpeedControllerValue.f32Value < -m_f32SpeedControllerDeadband)
                {
                    m_i8olddirection = 1;
                }

                m_tLastStructRight.i8WheelDir = m_i8olddirection;
                m_tLastStructLeft.i8WheelDir = m_i8olddirection;

            }
            else if (m_bEnableDirectionPlausibilization)
            {
                if (m_tLastStructRight.i8WheelDir == m_tLastStructLeft.i8WheelDir)
                {
                    m_i8olddirection = m_tLastStructRight.i8WheelDir;
                }
                m_vi8olddirection.insert(m_vi8olddirection.begin(), m_i8olddirection);
                if (m_vi8olddirection.size() > 15)
                {
                    m_vi8olddirection.pop_back();
                }
                for (std::vector<tInt8>::iterator it = m_vi8olddirection.begin(); it < m_vi8olddirection.end(); it++)
                {
                    temp = temp + *it;
                }
                if (temp > m_vi8olddirection.size() / 2)
                {
                    m_tLastStructRight.i8WheelDir = 1;
                    m_tLastStructLeft.i8WheelDir = 1;
                }
                else
                {
                    m_tLastStructRight.i8WheelDir = 0;
                    m_tLastStructRight.i8WheelDir = 0;
                }
            }
            TransmitSamples();
        }
    }
    RETURN_NOERROR;
}

tFloat32 cConverterWheels::calculateSpeed(const tUInt32 &ui32CurrentTimeStamp, const tUInt32 &ui32LastTimeStamp, const tUInt32 &ui32Ticks)
{
    // return if time difference is 0, if time difference is smaller than 0, if ticks are 0 or smaller 0
    if ((ui32CurrentTimeStamp - ui32LastTimeStamp == 0) || (ui32Ticks == 0)) return 0;
    //          circumference      SlotsInTimeDiff
    // speed =  -------------- *  -------------
    //           TotalSlots*          TimeDiff
    return (m_f32wheelCircumference / CW_SLOT_COUNT * static_cast<tFloat32>(ui32Ticks)) /
        (static_cast<tFloat32>(ui32CurrentTimeStamp - ui32LastTimeStamp) / static_cast<tFloat32>(1e6));
}

tResult cConverterWheels::TransmitSamples()
{
    // static variable for warning outputs to console
    static tInt32 i32WarningCounter = 0;

    // calculate the average of both wheel speeds
    tFloat32 f32speed = (m_f32LastCalculatedSpeedRight + m_f32LastCalculatedSpeedLeft) / 2;

    if (false && fabs((m_f32LastCalculatedSpeedRight - m_f32LastCalculatedSpeedLeft)) > CW_ERROR_DIFFERENCE_SIDES)
    {
        if (m_f32LastCalculatedSpeedRight < CW_MIN_LIMIT_IGNORE)
        {
            f32speed = m_f32LastCalculatedSpeedLeft;
            if (m_tLastStructLeft.i8WheelDir == 1)
                 f32speed = f32speed * -1;
        }
        else if (m_f32LastCalculatedSpeedLeft < CW_MIN_LIMIT_IGNORE)
        {
            f32speed = m_f32LastCalculatedSpeedRight;
            if (m_tLastStructRight.i8WheelDir == 1)
                f32speed = f32speed * -1;
        }
        i32WarningCounter++;
        if (i32WarningCounter % 200 == 0)
            LOG_WARNING(cString::Format("Wheel speed from left and right side are very different. Please check cables and connections! Right: %f, Left: %f, Result: %f",
                m_f32LastCalculatedSpeedRight, m_f32LastCalculatedSpeedLeft, f32speed));
    }
    else
    {
        // if direction is backwards speed should be negative
        if (m_tLastStructLeft.i8WheelDir == 1 && m_tLastStructRight.i8WheelDir == 1)
            f32speed = f32speed * -1;
    }

    //calculate the average of the arduino timestamp
    tUInt32 ui32arduinoTimestamp = (m_tLastStructLeft.ui32ArduinoTimestamp + m_tLastStructRight.ui32ArduinoTimestamp) / 2;


    // LOG_INFO(cString::Format("Measured Speed: %f", f32speed));
    //Transmit Values 
    RETURN_IF_FAILED(transmitSignalValue(m_oOutputCarSpeed, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, ui32arduinoTimestamp, m_ddlSignalValueId.value, f32speed));

    RETURN_NOERROR;
}
