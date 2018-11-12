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
#include "AdtfRosOpendrivePublisher.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ROS_OPENDRIVE_PUBLISHER_TRIGGERED_FILTER,
                                    "ROS Opendrive Publisher",
                                    cAdtfRosOpendrivePublisher,
                                    adtf::filter::pin_trigger({ "open_drive" }));

cAdtfRosOpendrivePublisher::cAdtfRosOpendrivePublisher()
{
    adtf::ucom::object_ptr<adtf::streaming::ant::IStreamType> pTypeDefault = adtf::ucom::make_object_ptr<adtf::streaming::ant::cStreamType>(adtf::streaming::ant::stream_meta_type_anonymous());
    // create_pin(*this, m_oInputOpenDrive, "open_drive", pTypeDefault);
    Register(m_oReader, "open_drive", pTypeDefault);

    RegisterPropertyVariable("topic", m_topic);
}

tResult cAdtfRosOpendrivePublisher::Configure()
{
    char *argv[] = {(char*)"cAdtfRosOpendrivePublisher"};
    int argc = sizeof(argv) / sizeof(char*) - 1;
    
    ros::init(argc, argv, "cAdtfRosOpendrivePublisher");
    ros::NodeHandle nh;

    m_pub = nh.advertise<std_msgs::String>(cString(m_topic).GetPtr(), 1);

    RETURN_NOERROR;
}

tResult cAdtfRosOpendrivePublisher::Process(tTimeStamp tmTimeOfTrigger)
{
    adtf::ucom::object_ptr<const adtf::streaming::ISample> pSampleAnonymous;
    while (IS_OK(m_oReader.GetNextSample(pSampleAnonymous)))
    {

        adtf::ucom::ant::object_ptr_shared_locked<const adtf::streaming::ant::ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pSampleAnonymous->Lock(pSampleBuffer));

        adtf_util::cString openDriveMapFileString;
        openDriveMapFileString.SetBuffer(pSampleBuffer->GetSize());
        memcpy(openDriveMapFileString.GetBuffer(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());

        if (openDriveMapFileString.GetBufferSize() > 0)
        {
            std_msgs::String msg;
            msg.data = openDriveMapFileString.GetBuffer();

            m_pub.publish(msg);
            ros::spinOnce();
        }
    }
    
    RETURN_NOERROR;
}