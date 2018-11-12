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

#pragma once

//*************************************************************************************************
#define CID_CONVERTERWHEELS_DATA_TRIGGERED_FILTER "better_converter_wheels.filter.user.aadc.cid"


using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

/*! The main class of the converter wheels module */
class cConverterWheels : public cTriggerFunction
{
private:

    //Properties
    /*! the wheel circumference in meter */
    property_variable<tFloat32> m_f32wheelCircumference = 0.34f;

    /*! the filter constant of first order */
    property_variable<tFloat32> m_f32FilterConstantfirstOrder = 0.3f;

    /*! enables or filtering enabled */
    property_variable<tBool> m_bEnableFiltering = tFalse;

    /*! plausibilization via direction indicator enable */
    property_variable<tBool> m_bEnableDirectionPlausibilization = tTrue;

    /*! direction derived from speed controller actuator value */
    property_variable<tBool> m_bEnableSpeedControllerDirection = tFalse;

    /*! deadband for speed controll direction. Values within the deadband (around 0) will not trigger an update of the direction*/
    property_variable<tFloat32> m_f32SpeedControllerDeadband = 0.1f;

    //Pins
    /*! Input Pin for wheel struct*/
    cPinReader      m_oInputWheelLeft;
    /*! Input Pin for wheel struct*/
    cPinReader      m_oInputWheelRight;
    /*! Input Pin for speed controller struct*/
    cPinReader      m_oInputSpeedController;


    /*! output pin writer for the the speed of the wheels */
    cPinWriter m_oOutputCarSpeed;

    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;
public:

    /*! Default constructor. */
    cConverterWheels();

    /*! Destructor. */
    virtual ~cConverterWheels() = default;

    /*!
     * Overwrites the Configure.
     *
     * \return  Standard Result Code.
     */
    tResult Configure() override;

    /*!
     * Overwrites the Process.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     *
     * \return  Standard Result Code.
     */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;
private:
    

    /*! calculate the speed from two timestamps
    * \param ui32CurrentTimeStamp the new and current timestamp
    * \param ui32LastTimeStamp the former timestamp
    * \param ui32Ticks the ticks since last sample
    * \return calculated speed in meter /sec
    */
    tFloat32 calculateSpeed(const tUInt32 &ui32CurrentTimeStamp, const tUInt32 &ui32LastTimeStamp, const tUInt32 &ui32Ticks = 1);

    /*! with this function all the samples (distance, overall distance, speed) on the output pins are transmitted
    *    \return   Returns a standard result code.
    */
    tResult TransmitSamples();

    /*! DDL identifier for signal value */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;


    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    /*! DDL identifier for wheel data value */
    struct
    {
        tSize ArduinoTimestamp;
        tSize WheelTach;
        tSize WheelDir;
    } m_ddlWheelDataIndex;

    /*! The wheel data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_WheelDataSampleFactory;

    /*! the timestamp of the last left wheel struct */
    tWheelData m_tLastStructLeft;

    /*! the timestamp of the last right wheel struct */
    tWheelData m_tLastStructRight;

    /*! the timestamp of the last left wheel struct */
    tWheelData m_tBeforeLastStructLeft;

    /*! the timestamp of the last right wheel struct */
    tWheelData m_tBeforeLastStructRight;

    /*! the last received speed controller value */
    tSignalValue m_tLastSpeedControllerValue;

    /*! the last calculated speed of right wheel */
    tFloat32 m_f32LastCalculatedSpeedRight = 0;

    /*! the last calculated speed of left wheel */
    tFloat32 m_f32LastCalculatedSpeedLeft = 0;


    /*! holds the overall distance since starting the adtf config*/
    tFloat32 m_f32OverallDistance = 0.0f;
    /*! first sample was received from left wheel */
    tBool m_bfirstSampleReceivedLeftWheel = tFalse;
    /*! first sample was received from right wheel */
    tBool m_bfirstSampleReceivedRightWheel = tFalse;
    /*! last direction */
    tInt8 m_i8olddirection;

    /*! vector with last directions for filtering */
    std::vector<tInt8> m_vi8olddirection;

};


//*************************************************************************************************
