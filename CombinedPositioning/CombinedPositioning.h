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

#define CID_CCOMBINEDPOS_DATA_TRIGGERED_FILTER "combined_positioning.filter.user.aadc.cid"
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

/*! the main class of the marker positioning. */
class cCombinedPositioning : public cTriggerFunction
{
private:
    /*! speed scale */
    adtf::base::property_variable<tFloat32> m_f32SpeedScale = 1.0f;


    /*! Reader of an InPin speed. */
    cPinReader m_oReaderSpeed;
    /*! Reader of an InPin IMU. */
    cPinReader m_oReaderIMU;

    /*! Writer to an OutPin. */
    cPinWriter m_oWriterLocal;

    /*! The codec factory */
    cSampleCodecFactory m_oCodecFactory;

    /*! The ddl indices for a tInerMeasUnitData */
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

    /*! The imu data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_IMUDataSampleFactory;

    /*! The ddl indices for a tSignalValue */
    struct
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalDataIndex;

    /*! The signal data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalDataSampleFactory;

    /*! The ddl indices for a tPosition */
    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionIndex;

    /*! The position sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;

    /*! speed estimate */
    tFloat32 m_f32Speed;
    /*! The 32 yaw rate */
    tFloat32 m_f32YawRate;

    /*! The 32 arduino timestamp */
    tUInt32 m_ui32ArduinoTimestamp;

    /*! The ticks */
    tTimeStamp m_ticks;

    /*! EKF variables */
    // For Local Positioning (without signs)
    Mat m_stateLocal; /*! filter state {X} */
    Mat m_errorCovLocal; /*! error covariance matrix {P} */
    Mat m_processCovLocal; /*! process covariance matrix {Q} */
    Mat m_transitionMatrixLocal; /*! state transition matrix {F} */

    /*!
     * helper functions.
     *
     * \param   x   A tFloat32 to process.
     * \param   y   A tFloat32 to process.
     *
     * \return  A tFloat32.
     */
    tFloat32 mod(tFloat32 x, tFloat32 y);

    /*!
     * Normalize angle.
     *
     * \param   alpha   The alpha.
     * \param   center  The center.
     *
     * \return  A tFloat32.
     */
    tFloat32 normalizeAngle(tFloat32 alpha, tFloat32 center);

    /*!
     * Gets the time.
     *
     * \return  The time.
     */
    tTimeStamp GetTime();

    /*!
     * Process the iner meas unit sample.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     * \param   oSample         The sample.
     *
     * \return  Standard Result Code.
     */
    tResult ProcessInerMeasUnitSample(tTimeStamp tmTimeOfTrigger, const adtf::streaming::ISample &oSample);

    /*!
     * Sends a position structure.
     *
     * \param   timeOfFix   The time of fix.
     * \param   f32X        The 32 x coordinate.
     * \param   f32Y        The 32 y coordinate.
     * \param   f32Radius   The 32 radius.
     * \param   f32Heading  The 32 heading.
     * \param   f32Speed    The 32 speed.
     *
     * \return  Standard Result Code.
     */
    tResult sendPositionStruct(cPinWriter& writer, const tTimeStamp &timeOfFix, const tFloat32 &f32X, const tFloat32 &f32Y, const tFloat32 &f32Radius,
        const tFloat32 &f32Heading, const tFloat32 &f32Speed, bool triggerPipe);

    /**
     * Run a kalman step
     */
    void predictStep(float dt, Mat& state, Mat& errorCov, Mat& processCov, Mat& transitionMatrix);

public:

    /*! Default constructor. */
    cCombinedPositioning();

    /*! Destructor. */
    virtual ~cCombinedPositioning() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

};
