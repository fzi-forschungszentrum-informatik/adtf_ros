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
#include "AdtfRosUltrasonicPublisher.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ROS_ULTRASONIC_PUBLISHER_TRIGGERED_FILTER,
                                    "ROS Ultrasonic Publisher",
                                    cAdtfRosUltrasonicPublisher,
                                    adtf::filter::pin_trigger({ "ultrasonic_struct" }));

cAdtfRosUltrasonicPublisher::cAdtfRosUltrasonicPublisher()
{
    object_ptr<IStreamType> pTypeUSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tUltrasonicStruct", pTypeUSData, m_USDataSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.tSideLeft.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.tSideLeft.f32Value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideRight") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.tSideRight.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.tSideRight.f32Value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.tRearLeft.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.tRearLeft.f32Value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.tRearCenter.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") + cString(".f32Value"), m_ddlUltrasonicStructIndex.tRearCenter.f32Value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearRight") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.tRearRight.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.tRearRight.f32Value));
    }
    else
    {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }

    Register(m_oReader, "ultrasonic_struct", pTypeUSData);

    RegisterPropertyVariable("topic", m_topic);
}

tResult cAdtfRosUltrasonicPublisher::Configure()
{
    char *argv[] = {(char*)"cAdtfRosUltrasonicPublisher"};
    int argc = sizeof(argv) / sizeof(char*) - 1;
    
    ros::init(argc, argv, "cAdtfRosUltrasonicPublisher");
    ros::NodeHandle nh;

    m_pub = nh.advertise<ros_oadrive::Ultrasonic>(cString(m_topic).GetPtr(), 1);

    RETURN_NOERROR;
}

tResult cAdtfRosUltrasonicPublisher::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSampleFromUS;

    if (IS_OK(m_oReader.GetLastSample(pSampleFromUS)))
    {
        auto oDecoderUS = m_USDataSampleFactory.MakeDecoderFor(*pSampleFromUS);

        RETURN_IF_FAILED(oDecoderUS.IsValid());

        tUltrasonicStruct US_data;
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.tSideLeft.f32Value, &US_data.tSideLeft.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.tSideRight.f32Value, &US_data.tSideRight.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.tRearLeft.f32Value, &US_data.tRearLeft.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.tRearCenter.f32Value, &US_data.tRearCenter.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.tRearRight.f32Value, &US_data.tRearRight.f32Value));

       
        ros_oadrive::Ultrasonic msg;
        msg.header = std_msgs::Header();
        msg.header.stamp = ros::Time::now();
        msg.sideLeft = US_data.tSideLeft.f32Value;
        msg.sideRight = US_data.tSideRight.f32Value;
        msg.rearLeft = US_data.tRearLeft.f32Value;
        msg.rearCenter = US_data.tRearCenter.f32Value;
        msg.rearRight = US_data.tRearRight.f32Value;

        // fix -1:
        if (msg.sideLeft < 0) msg.sideLeft = 400;
        if (msg.sideRight < 0) msg.sideRight = 400;
        if (msg.rearLeft < 0) msg.rearLeft = 400;
        if (msg.rearCenter < 0) msg.rearCenter = 400;
        if (msg.rearRight < 0) msg.rearRight = 400;

        // Store result
        m_lastMsgs[m_currentIdx] = msg;

        m_currentIdx++;

        if (m_currentIdx >= NUM_LAST_MSGS) {
            m_currentIdx = 0;
        }

        // Hard coded for exactly 3 US right now..
        const int id1 = m_currentIdx;
        const int id2 = (m_currentIdx + 1) % NUM_LAST_MSGS;
        // Smoothing step to filter flickering
        if (msg.sideLeft > 0 && (m_lastMsgs[id1].sideLeft >= 400 || m_lastMsgs[id2].sideLeft >= 400)) {
            msg.sideLeft = 400;
        }
        if (msg.sideRight > 0 && (m_lastMsgs[id1].sideRight >= 400 || m_lastMsgs[id2].sideRight >= 400)) {
            msg.sideRight = 400;
        }
        if (msg.rearLeft > 0 && (m_lastMsgs[id1].rearLeft >= 400 || m_lastMsgs[id2].rearLeft >= 400)) {
            msg.rearLeft = 400;
        }
        if (msg.rearCenter > 0 && (m_lastMsgs[id1].rearCenter >= 400 || m_lastMsgs[id2].rearCenter >= 400)) {
            msg.rearCenter = 400;
        }
        if (msg.rearRight > 0 && (m_lastMsgs[id1].rearRight >= 400 || m_lastMsgs[id2].rearRight >= 400)) {
            msg.rearRight = 400;
        }

        m_pub.publish(msg);
        ros::spinOnce();
    }
    
    RETURN_NOERROR;
}