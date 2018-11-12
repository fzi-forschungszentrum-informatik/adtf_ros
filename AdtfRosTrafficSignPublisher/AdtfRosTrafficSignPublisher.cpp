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
#include "AdtfRosTrafficSignPublisher.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ROS_TRAFFIC_SIGN_PUBLISHER_TRIGGERED_FILTER,
                                    "ROS Traffic Sign Publisher",
                                    cAdtfRosTrafficSignPublisher,
                                    adtf::filter::pin_trigger({ "road_sign_ext" }));

cAdtfRosTrafficSignPublisher::cAdtfRosTrafficSignPublisher()
{
    object_ptr<IStreamType> pTypeRoadSignData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tRoadSignExt", pTypeRoadSignData, m_RoadSignSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "i16Identifier", m_ddlRoadSignIndex.id);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "f32Imagesize", m_ddlRoadSignIndex.size);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "af32TVec", m_ddlRoadSignIndex.tvec);
        adtf_ddl::access_element::find_index(m_RoadSignSampleFactory, "af32RVec", m_ddlRoadSignIndex.rvec);
    }
    else
    {
        LOG_WARNING("No mediadescription for tRoadSignExt found!");
    }
    Register(m_oReader, "road_sign_ext", pTypeRoadSignData);

    RegisterPropertyVariable("topic", m_topic);

    // initialize translation and rotation vectors
    m_Tvec = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));
    m_Rvec = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));
}

tResult cAdtfRosTrafficSignPublisher::Configure()
{
    char *argv[] = {(char*)"cAdtfRosTrafficSignPublisher"};
    int argc = sizeof(argv) / sizeof(char*) - 1;
    
    ros::init(argc, argv, "cAdtfRosTrafficSignPublisher");
    ros::NodeHandle nh;

    m_pub = nh.advertise<ros_oadrive::TrafficSign>(cString(m_topic).GetPtr(), 1);

    RETURN_NOERROR;
}

tResult cAdtfRosTrafficSignPublisher::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const adtf::streaming::ISample> pReadSample;
    while (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        ProcessInputRoadSign(*pReadSample);
    }

    RETURN_NOERROR;
}

tResult cAdtfRosTrafficSignPublisher::ProcessInputRoadSign(const adtf::streaming::ISample &pSample) {
    tInt16 i16Identifier = 0;

    // Decode message
    auto oDecoder = m_RoadSignSampleFactory.MakeDecoderFor(pSample);

    i16Identifier = access_element::get_value(oDecoder, m_ddlRoadSignIndex.id);

    const tVoid* pArray;
    tSize size;

    // fetch marker translation and rotation arrays
    access_element::get_array(oDecoder, "af32TVec", pArray, size);
    m_Tvec.data = const_cast<uchar*>(static_cast<const uchar*>(pArray));

    access_element::get_array(oDecoder, "af32RVec", pArray, size);
    m_Rvec.data = const_cast<uchar*>(static_cast<const uchar*>(pArray));

    ros_oadrive::TrafficSign trafficSignMsg;
    ros_oadrive::EnvironmentObject environmentObjectMsg;
    geometry_msgs::Pose2D poseMsg;

    // TODO: Kadabra reads in those values, I dont know if those are correct or not
    poseMsg.x = m_Tvec.at<float>(2) + 0.295; // add camera offset (0.295)
    poseMsg.y = m_Tvec.at<float>(0);
    //poseMsg.x = m_Tvec.at<float>(0);
    //poseMsg.y = m_Tvec.at<float>(1);
    poseMsg.theta = m_Rvec.at<float>(0);

    environmentObjectMsg.pose = poseMsg;
    environmentObjectMsg.width = 0.1f;
    environmentObjectMsg.length = 0.01f;

    trafficSignMsg.header = std_msgs::Header();
    trafficSignMsg.header.stamp = ros::Time::now();
    trafficSignMsg.header.frame_id = "oriented_car";
    trafficSignMsg.object = environmentObjectMsg;
    
    if (i16Identifier == 0) {
        trafficSignMsg.type = "UNMARKED_INTERSECTION";
    } else if (i16Identifier == 1) {
        trafficSignMsg.type = "STOP_AND_GIVE_WAY";
    } else if (i16Identifier == 2) {
        trafficSignMsg.type = "PARKING_AREA";
    } else if (i16Identifier == 3) {
        trafficSignMsg.type = "HAVE_WAY";
    } else if (i16Identifier == 4) {
        trafficSignMsg.type = "AHEAD_ONLY";
    } else if (i16Identifier == 5) {
        trafficSignMsg.type = "GIVE_WAY";
    } else if (i16Identifier == 6) {
        trafficSignMsg.type = "PEDESTRIAN_CROSSING";
    } else if (i16Identifier == 7) {
        trafficSignMsg.type = "ROUNDABOUT";
    } else if (i16Identifier == 8) {
        trafficSignMsg.type = "NO_OVERTAKING";
    } else if (i16Identifier == 9) {
        trafficSignMsg.type = "NO_ENTRY_VEHICULAR_TRAFFIC";
    } else if (i16Identifier == 10) {
        trafficSignMsg.type = "TEST_COURSE_A9";
    } else if (i16Identifier == 11) {
        trafficSignMsg.type = "ONE_WAY_STREET";
    } else if (i16Identifier == 12) {
        trafficSignMsg.type = "ROAD_WORKS";
    } else if (i16Identifier == 13) {
        trafficSignMsg.type = "KMH_50";
    } else if (i16Identifier == 14) {
        trafficSignMsg.type = "KMH_100";
    } else if (i16Identifier == 99) {
        trafficSignMsg.type = "NO_MATCH";
    }

    m_pub.publish(trafficSignMsg);

    ros::spinOnce();

    RETURN_NOERROR;
}