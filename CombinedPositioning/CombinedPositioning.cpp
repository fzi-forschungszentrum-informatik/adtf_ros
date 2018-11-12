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


/*********************************************************************
 * This code was provided by HERE
 *
 * *******************************************************************/

#define A_UTILS_NO_DEPRECATED_WARNING

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <adtf3.h>
#include <stdlib.h>

#include "CombinedPositioning.h"

 /*! configuration parameters */

// process covariances
#define MP_PROCESS_X                    1e-3
#define MP_PROCESS_Y                    1e-3
#define MP_PROCESS_HEADING              3e-4
#define MP_PROCESS_HEADING_DRIFT        5e-8
#define MP_PROCESS_SPEED                2e-3
#define MP_PROCESS_SPEED_SCALE          1e-6

// initial covariance values
#define MP_PROCESS_INIT_X               10.0
#define MP_PROCESS_INIT_Y               10.0
#define MP_PROCESS_INIT_HEADING         0.55
#define MP_PROCESS_INIT_HEADING_DRIFT   0.25
#define MP_PROCESS_INIT_SPEED           1.0
#define MP_PROCESS_INIT_SPEED_SCALE     0.5

/*! defines a data triggered filter and exposes it via a plugin class factory */
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CCOMBINEDPOS_DATA_TRIGGERED_FILTER,
    "Combined Positioning",
    cCombinedPositioning,
    adtf::filter::pin_trigger({ "imu" }));

/*! initialize the trigger function */
cCombinedPositioning::cCombinedPositioning()
{
    SetName("MarkerPos");

    RegisterPropertyVariable("Speed Scale", m_f32SpeedScale);

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

    //the signal struct
    object_ptr<IStreamType> pTypeSignalData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalData, m_SignalDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_SignalDataSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalDataIndex.timeStamp);
        adtf_ddl::access_element::find_index(m_SignalDataSampleFactory, "f32Value", m_ddlSignalDataIndex.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }

    //the position struct
    object_ptr<IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    }
    else
    {
        LOG_WARNING("No mediadescription for tPosition found!");
    }

    // cast to const type for the method calls below
    object_ptr<const IStreamType> pConstTypeSignalData = pTypeSignalData;
    object_ptr<const IStreamType> pConstTypeIMUData = pTypeIMUData;
    object_ptr<const IStreamType> pConstTypePositionData = pTypePositionData;

    //register input pin(s)
    Register(m_oReaderSpeed, "speed", pConstTypeSignalData);
    Register(m_oReaderIMU, "imu", pConstTypeIMUData);

    //register output pin
    Register(m_oWriterLocal, "position_local", pConstTypePositionData);

    // initialize EKF variables
    m_stateLocal = Mat(6, 1, CV_64F, Scalar::all(0));
    m_errorCovLocal = Mat(6, 6, CV_64F, Scalar::all(0));

    tFloat64 T = 0.1;
    m_errorCovLocal.at<double>(0, 0) = MP_PROCESS_INIT_X;
    m_errorCovLocal.at<double>(1, 1) = MP_PROCESS_INIT_Y;
    m_errorCovLocal.at<double>(2, 2) = MP_PROCESS_INIT_HEADING;
    m_errorCovLocal.at<double>(2, 3) = MP_PROCESS_INIT_HEADING / T;
    m_errorCovLocal.at<double>(3, 3) = MP_PROCESS_INIT_HEADING_DRIFT;
    m_errorCovLocal.at<double>(4, 4) = MP_PROCESS_INIT_SPEED;
    m_errorCovLocal.at<double>(4, 5) = MP_PROCESS_INIT_SPEED / T;
    m_errorCovLocal.at<double>(5, 5) = MP_PROCESS_INIT_SPEED_SCALE;

    m_transitionMatrixLocal = Mat(6, 6, CV_64F, Scalar::all(0));
    setIdentity(m_transitionMatrixLocal);

    m_processCovLocal = Mat(6, 6, CV_64F, Scalar::all(0));
    m_processCovLocal.at<double>(0, 0) = MP_PROCESS_X;
    m_processCovLocal.at<double>(1, 1) = MP_PROCESS_Y;
    m_processCovLocal.at<double>(2, 2) = MP_PROCESS_HEADING;
    m_processCovLocal.at<double>(3, 3) = MP_PROCESS_HEADING_DRIFT;
    m_processCovLocal.at<double>(4, 4) = MP_PROCESS_SPEED;
    m_processCovLocal.at<double>(5, 5) = MP_PROCESS_SPEED_SCALE;

    // initialize other variables
    m_f32Speed = 0;
    m_f32YawRate = 0;

    m_ui32ArduinoTimestamp = 0;

}



/*! implements the configure function to read ALL Properties */
tResult cCombinedPositioning::Configure()
{
    RETURN_NOERROR;
}

/*! funtion will be executed each time a trigger occured */
tResult cCombinedPositioning::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;

    //LOG_INFO(cString::Format("process: %lu", tmTimeOfTrigger).GetPtr());

    while (IS_OK(m_oReaderSpeed.GetNextSample(pReadSample)))
    {
        // store speed
        auto oDecoder = m_SignalDataSampleFactory.MakeDecoderFor(*pReadSample);
        m_f32Speed = adtf_ddl::access_element::get_value(oDecoder, m_ddlSignalDataIndex.value);
    }

    while (IS_OK(m_oReaderIMU.GetNextSample(pReadSample)))
    {
        // predict
        ProcessInerMeasUnitSample(tmTimeOfTrigger, *pReadSample);
    }

    RETURN_NOERROR;
}

/*! calculates normalized angle */
tFloat32 cCombinedPositioning::normalizeAngle(tFloat32 alpha, tFloat32 center)
{
    return mod(alpha - center + static_cast<tFloat32>(M_PI), 2.0f*static_cast<tFloat32>(M_PI)) + center - static_cast<tFloat32>(M_PI);
}

/*! calculates modulus after division */
tFloat32 cCombinedPositioning::mod(tFloat32 x, tFloat32 y)
{
    tFloat32 r;
    tFloat32 b_x;
    if (y == floor(y))
    {
        return x - floor(x / y) * y;
    }
    else
    {
        r = x / y;
        if (r < 0.0)
        {
            b_x = ceil(r - 0.5f);
        }
        else
        {
            b_x = floor(r + 0.5f);
        }
        if (fabs(r - b_x) <= 2.2204460492503131E-16 * fabs(r))
        {
            return 0.0;
        }
        else
        {
            return (r - floor(r)) * y;
        }
    }
}

/*! support function for getting time */
tTimeStamp cCombinedPositioning::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}

/*! sends position data out */
tResult cCombinedPositioning::sendPositionStruct(cPinWriter& writer, const tTimeStamp &timeOfFix,  const tFloat32 &f32X, const tFloat32 &f32Y, const tFloat32 &f32Radius,
    const tFloat32 &f32Heading, const tFloat32 &f32Speed, bool triggerPipe)
{
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, timeOfFix));

    auto oCodec = m_PositionSampleFactory.MakeCodecFor(pSample);

    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.x, f32X));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.y, f32Y));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.radius, f32Radius));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.speed, f32Speed));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.heading, f32Heading));

    //LOG_INFO(cString::Format("sendPositionStruct: %.3f %.3f %.3f %.3f %.3f", f32X, f32Y,
        //f32Radius, f32Heading, f32Speed).GetPtr());

    // the sample buffer lock is released in the destructor of oCodec
    if (triggerPipe) {
        writer << pSample << flush << trigger;
    } else {
        writer << pSample << flush;
    }

    RETURN_NOERROR;
}

/*! processes inertial measurement data sample, and runs EKF prediction
 *  based on heading rate and speed measurements */
tResult cCombinedPositioning::ProcessInerMeasUnitSample(tTimeStamp tmTimeOfTrigger, const adtf::streaming::ISample &oSample)
{
    tFloat64 dt = 0;
    tUInt32 ui32ArduinoTimestamp = 0;

    // parse the data 
    auto oDecoder = m_IMUDataSampleFactory.MakeDecoderFor(oSample);

    // yaw-rate gyro measurment 
    m_f32YawRate = access_element::get_value(oDecoder, m_ddlInerMeasUnitDataIndex.G_z);

    // fetch timestamp and calculate dt
    ui32ArduinoTimestamp = access_element::get_value(oDecoder, m_ddlInerMeasUnitDataIndex.timeStamp);

    if (m_ui32ArduinoTimestamp == 0) {
        // Abort if this is the first message - we cannot calculate a dt, yet
        m_ui32ArduinoTimestamp = ui32ArduinoTimestamp;
        
        RETURN_NOERROR;
    }

    dt = (tFloat64)(ui32ArduinoTimestamp - m_ui32ArduinoTimestamp)*1e-6;

    m_ui32ArduinoTimestamp = ui32ArduinoTimestamp;

    predictStep(dt, m_stateLocal, m_errorCovLocal, m_processCovLocal, m_transitionMatrixLocal);

    sendPositionStruct(m_oWriterLocal, tmTimeOfTrigger, static_cast<tFloat32>(m_stateLocal.at<double>(0)), static_cast<tFloat32>(m_stateLocal.at<double>(1)),
        static_cast<tFloat32>(sqrt(m_errorCovLocal.at<double>(0, 0) + m_errorCovLocal.at<double>(1, 1))),
        static_cast<tFloat32>(m_stateLocal.at<double>(2)), static_cast<tFloat32>(m_stateLocal.at<double>(4)), false);

    RETURN_NOERROR;
}

void cCombinedPositioning::predictStep(float dt, Mat& state, Mat& errorCov, Mat& processCov, Mat& transitionMatrix) {
    // update heading
    tFloat32 hk = static_cast<tFloat32>(state.at<double>(2) + (m_f32YawRate*static_cast<tFloat32>(DEG2RAD) + state.at<double>(3))*dt);

    // normalize heading -pi:pi
    hk = normalizeAngle(hk, 0);

    tFloat32 sc = m_f32SpeedScale;

    // update speed and scale
    tFloat32 ak = static_cast<tFloat32>(state.at<double>(5));
    tFloat32 vk = m_f32Speed*(sc - ak);

    // update transition matrix; F = I + Fc*dt
    transitionMatrix.at<double>(0, 2) = -vk*sin(hk)*dt;
    transitionMatrix.at<double>(0, 3) = -vk*sin(hk)*dt;
    transitionMatrix.at<double>(0, 4) = cos(hk)*dt;
    transitionMatrix.at<double>(0, 5) = -vk / (sc - ak)*cos(hk)*dt;

    transitionMatrix.at<double>(1, 2) = vk*cos(hk)*dt;
    transitionMatrix.at<double>(1, 3) = vk*cos(hk)*dt;
    transitionMatrix.at<double>(1, 4) = sin(hk)*dt;
    transitionMatrix.at<double>(1, 5) = -vk / (sc - ak)*sin(hk)*dt;

    transitionMatrix.at<double>(2, 3) = dt;

    // propagate state and covariance
    state.at<double>(0) += vk*cos(hk)*dt;
    state.at<double>(1) += vk*sin(hk)*dt;
    state.at<double>(2) = hk;
    state.at<double>(4) = vk;

    errorCov = transitionMatrix*errorCov*transitionMatrix.t() + processCov;
}