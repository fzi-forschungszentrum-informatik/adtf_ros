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
#include "AdtfRosPositionPublisher.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ROS_POSITION_PUBLISHER_TRIGGERED_FILTER,
                                    "ROS Position Publisher",
                                    cAdtfRosPositionPublisher,
                                    adtf::filter::pin_trigger({ "position_local" }));

cAdtfRosPositionPublisher::cAdtfRosPositionPublisher()
{
    adtf::ucom::object_ptr<adtf::streaming::IStreamType> pTypePositionData;
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

    Register(m_oReaderLocal, "position_local", pTypePositionData);
    Register(m_oReaderGlobal, "position_global", pTypePositionData);

    RegisterPropertyVariable("topic", m_topic);
    RegisterPropertyVariable("topic_extended", m_topicExtended);
    // RegisterPropertyVariable("topic_extended_world", m_topicExtendedWorld);
    
    RegisterPropertyVariable("tf_world", m_tfWorld);
    RegisterPropertyVariable("tf_local", m_tfLocal);
    RegisterPropertyVariable("tf_car", m_tfCar);
    RegisterPropertyVariable("tf_oriented_car", m_tfOrientedCar);
}

tResult cAdtfRosPositionPublisher::Configure()
{
    char *argv[] = {(char*)"cAdtfRosPositionPublisher"};
    int argc = sizeof(argv) / sizeof(char*) - 1;
    
    ros::init(argc, argv, "cAdtfRosPositionPublisher");
    ros::NodeHandle nh;

    // Debug code, not used atm
    m_pubWorld = nh.advertise<geometry_msgs::PoseStamped>("/aadc/pose_world", 1);
    m_pub = nh.advertise<ros_oadrive::Pose2DStamped>(cString(m_topic).GetPtr(), 1);
    m_pubExtended = nh.advertise<ros_oadrive::MarkerPosition>(cString(m_topicExtended).GetPtr(), 1);
    // m_pubExtendedWorld = nh.advertise<ros_oadrive::MarkerPosition>(cString(m_topicExtendedWorld).GetPtr(), 1);

    m_broadcaster = new tf::TransformBroadcaster();
    RETURN_NOERROR;
}

tResult cAdtfRosPositionPublisher::Process(tTimeStamp tmTimeOfTrigger)
{
    auto stamp = ros::Time::now();
    adtf::ucom::object_ptr<const adtf::streaming::ISample> pReadSample;
    while (IS_OK(m_oReaderLocal.GetNextSample(pReadSample)))
    {
        ros_oadrive::MarkerPosition markerPos;

        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(pReadSample);

        markerPos.header = m_lastPose.header = std_msgs::Header();
        markerPos.header.frame_id = m_lastPose.header.frame_id = cString(m_tfLocal).GetPtr();
        markerPos.header.stamp = m_lastPose.header.stamp = stamp;

        markerPos.pose.x = m_lastPose.x = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.x);
        markerPos.pose.y = m_lastPose.y = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.y);
        markerPos.pose.theta = m_lastPose.theta = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.heading);
        
        markerPos.radius = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.radius);
        markerPos.speed = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.speed);

        m_pub.publish(m_lastPose);
        m_pubExtended.publish(markerPos);

        tf::Quaternion car_orientation = tf::Quaternion(0, 0, 0, 1);
        m_broadcaster->sendTransform(
            tf::StampedTransform(tf::Transform(car_orientation, tf::Vector3(m_lastPose.x, m_lastPose.y, 0.0f)),
            stamp,
            cString(m_tfLocal).GetPtr(),
            cString(m_tfCar).GetPtr()));

        car_orientation.setRotation(tf::Vector3(0.0f, 0.0f, 1.0f), m_lastPose.theta);
        m_broadcaster->sendTransform(
            tf::StampedTransform(tf::Transform(car_orientation, tf::Vector3(0.0f, 0.0f, 0.0f)),
            stamp,
            cString(m_tfCar).GetPtr(),
            cString(m_tfOrientedCar).GetPtr()));
    }

    // Publish transformation from local to world frame
    while (IS_OK(m_oReaderGlobal.GetNextSample(pReadSample)))
    {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(pReadSample);

        float x = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.x);
        float y = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.y);
        float theta = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.heading);

        // Debug code, not used atm
        geometry_msgs::PoseStamped msg;

        msg.header = std_msgs::Header();
        msg.header.frame_id = cString(m_tfWorld).GetPtr();
        msg.header.stamp = stamp;

        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        //msg.pose.theta = theta;

        m_pubWorld.publish(msg);

        // publish as own topic
        // markerPos.header = m_lastPose.header = std_msgs::Header();
        // markerPos.header.frame_id = m_lastPose.header.frame_id = cString(m_tfWorld).GetPtr();
        // markerPos.header.stamp = m_lastPose.header.stamp = stamp;

        // markerPos.pose.x = x;
        // markerPos.pose.y = y;
        // markerPos.pose.theta = theta;
        
        // markerPos.radius = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.radius);
        // markerPos.speed = adtf_ddl::access_element::get_value(oDecoder, m_ddlPositionIndex.speed);

        tf::Quaternion orientation = tf::Quaternion(0, 0, 0, 1);
        orientation.setRotation(tf::Vector3(0.0f, 0.0f, 1.0f), theta - m_lastPose.theta);

        auto transform = tf::Transform(orientation, tf::Vector3(0.0f, 0.0f, 0.0f));
        auto rotated = transform(tf::Vector3(m_lastPose.x, m_lastPose.y, 0.0f));

        transform.setOrigin(tf::Vector3(x - rotated.getX(), y- rotated.getY(), 0.0f));
        m_broadcaster->sendTransform(
            tf::StampedTransform(transform,
            stamp,
            // cString("world_tmp").GetPtr(),
            cString(m_tfWorld).GetPtr(),
            cString(m_tfLocal).GetPtr()));
    }
    
    RETURN_NOERROR;
}