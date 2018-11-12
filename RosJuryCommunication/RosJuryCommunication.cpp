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
#include "RosJuryCommunication.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ROS_JURY_COMMUNICATION_TRIGGERED_FILTER,
                                    "ROS Jury Communication",
                                    cRosJuryCommunication,
                                    adtf::filter::timer_trigger(1));

cRosJuryCommunication::cRosJuryCommunication()
{
    // Jury_struct
    object_ptr<IStreamType> pTypeJuryStruct;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tJuryStruct", pTypeJuryStruct, m_juryStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ActionID"), m_ddlJuryStructId.actionId));
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ManeuverEntry"), m_ddlJuryStructId.maneuverEntry));
    }
    else
    {
        LOG_INFO("No mediadescription for tJuryStruct found!");
    }
    Register(m_oInputJuryStruct, "jury_struct", pTypeJuryStruct);

    // Driver_struct
    object_ptr<IStreamType> pTypeDriverStruct;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDriverStruct", pTypeDriverStruct, m_driverStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16StateID"), m_ddlDriverStructId.stateId));
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16ManeuverEntry"), m_ddlDriverStructId.maneuverEntry));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", "tDriverStruct"));
    }
    Register(m_oOutputDriverStruct, "driver_struct", pTypeDriverStruct);

    object_ptr<IStreamType> pTypeDefault = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_anonymous());
    Register(m_oInputManeuverList, "maneuver_list", pTypeDefault);
}

tResult cRosJuryCommunication::Configure()
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    char *argv[] = {(char*)"cRosJuryCommunication"};
    int argc = sizeof(argv) / sizeof(char*) - 1;
    
    ros::init(argc, argv, "cRosJuryCommunication");
    ros::NodeHandle nh;
    m_callbackQueue = new ros::CallbackQueue();
    nh.setCallbackQueue(m_callbackQueue);

    // here we will publish the current maneuver we have to drive
    m_pubManeuver = nh.advertise<ros_oadrive::JuryManeuver>("/aadc/jury/current_maneuver", 25);
    
    m_eventPub = nh.advertise<std_msgs::String>("/aadc/jury/event", 25);
    
    // Keep this for a few more days for backwards compatibility..
    m_pubJury = nh.advertise<std_msgs::String>("/aadc/jury/status", 25);

    // Check for READY_EVENT to send ready
    m_eventSub = nh.subscribe("/aadc/jury/event", 25, &cRosJuryCommunication::eventLogic, this);

    // m_pub = nh.advertise<ros_oadrive::Ultrasonic>(cString(m_topic).GetPtr(), 1);

    RETURN_NOERROR;
}

// Commands:
/**
 * Publishes to jury
 */
void cRosJuryCommunication::publishStartUp() {
    tDriverStruct driverStruct;
    driverStruct.i16StateID = statecar_startup;
    driverStruct.i16ManeuverEntry = m_currentManeuverEntry;

    TransmitDriverStruct(driverStruct);
}

void cRosJuryCommunication::publishReady() {
    tDriverStruct driverStruct;
    driverStruct.i16StateID = statecar_ready;
    driverStruct.i16ManeuverEntry = m_currentManeuverEntry;

    TransmitDriverStruct(driverStruct);
}

void cRosJuryCommunication::publishRunning() {
    tDriverStruct driverStruct;
    driverStruct.i16StateID = statecar_running;
    driverStruct.i16ManeuverEntry = m_currentManeuverEntry;

    TransmitDriverStruct(driverStruct);
}

void cRosJuryCommunication::publishComplete() {
    tDriverStruct driverStruct;
    driverStruct.i16StateID = statecar_complete;
    driverStruct.i16ManeuverEntry = m_currentManeuverEntry;

    TransmitDriverStruct(driverStruct);
}

/**
 * publishes to ros
 */
void cRosJuryCommunication::publishGetReady() {
    std_msgs::String event;
    event.data = "GET_READY_EVENT";
    m_eventPub.publish(event);

    m_running = true;
}

void cRosJuryCommunication::publishStart() {
    std_msgs::String event;
    event.data = "START_EVENT";
    m_eventPub.publish(event);

    // TODO: Remove as soon as rewritten in ros_oadrive
    // Temporarily also publish to jury status
    std_msgs::String juryMsg;
    juryMsg.data = "Start";
    m_pubJury.publish(juryMsg);

    // Also tell the jury that we are now running
    publishRunning();
}

void cRosJuryCommunication::publishStop() {
    std_msgs::String event;
    event.data = "STOP_EVENT";
    m_eventPub.publish(event);

    // TODO: Remove as soon as rewritten in ros_oadrive
    // Temporarily also publish to jury status
    std_msgs::String juryMsg;
    juryMsg.data = "Stop";
    m_pubJury.publish(juryMsg);

    m_running = false;
}

/**
 * sets a new maneuver entry
 */
void cRosJuryCommunication::setManeuver(int entry) {
    m_currentManeuverEntry = entry;

    // Find the correct sector and maneuver idx in the arrays:
    m_sectorIdx = -1;
    m_maneuverIdx = -1;
    for (unsigned int i = 0; i < m_sectorList.size(); i++) {
        for (unsigned int j = 0; j < m_sectorList[i].sector.size(); j++) {
            if (m_sectorList[i].sector[j].id == entry) {
                m_sectorIdx = i;
                m_maneuverIdx = j;
                break;
            }
        }

        if (m_sectorIdx >= 0) {
            // Sector was found!
            break;
        }
    }

    if (m_sectorIdx < 0) {
        LOG_ERROR(cString::Format("JURY: Entry %d in no sector found! This is an error!e", entry));
    }
}

/**
 * sets us to the next maneuver, returns false if we were driving the last maneuver
 */
bool cRosJuryCommunication::incrementManeuver() {
    if (m_sectorIdx < 0) {
        LOG_ERROR("JURY: Increment called before setManeuver! This is an error! Please run GetReady before doing anything else");
        return false;
    }

    // Check if there are still maneuvers in this sector
    if ((m_maneuverIdx + 1) < m_sectorList[m_sectorIdx].sector.size()) {
        m_maneuverIdx++;

        LOG_INFO("JURY: Incremented maneuver!");

        return true;
    } else {
        // Jump to the next sector if there is one left
        if ((m_sectorIdx + 1) < m_sectorList.size()) {
            m_maneuverIdx = 0;
            m_sectorIdx++;

            LOG_INFO("JURY: Incremented sector!");

            return true;
        } else {
            // There is no maneuver and no sector left, we are done 🎉🎉🎉🎉🎉
            LOG_INFO(cString::Format("JURY: We are done! #Sectors: %d, SectorIdx: %d, ManeuverIdx: %d", m_sectorList.size(), m_sectorIdx, m_maneuverIdx));
            return false;
        }
    }
}

tManeuver cRosJuryCommunication::getCurrentManeuver() {
    if (m_sectorIdx < 0) {
        LOG_ERROR("getCurrentManeuver called without a maneuver! This is an error! Please run GetReady before doing anything else!");
        return tManeuver();
    }

    return m_sectorList[m_sectorIdx].sector[m_maneuverIdx];
}

/**
 * publishes the current maneuver to ros
 */
void cRosJuryCommunication::publishCurrentManeuver() {
    auto current = getCurrentManeuver();

    ros_oadrive::JuryManeuver msg;
    msg.id = current.id;
    msg.action = maneuverToString(current.action);
    msg.extra = current.extra;


    // Also store the current maneuver for later usage
    m_currentManeuverEntry = current.id;

    m_pubManeuver.publish(msg);
}

void cRosJuryCommunication::eventLogic(const std_msgs::String::ConstPtr& event) {
    std::string type = event->data;

    if (type == "READY_EVENT" && m_running) {
        publishReady();
        publishCurrentManeuver();
    } else if (type == "MANEUVER_DONE_EVENT") {
        if (m_running) {
            // Increment Maneuver list and send new maneuver

            publishComplete();
            if (incrementManeuver()) {
                publishCurrentManeuver();
                // Also tell the jury that we are now running the next maneuver
                publishRunning();
            } else {
                // We are done!
                publishStop();
            }
        } else {
            LOG_WARNING("RosJuryCommunication: Ignored MANEUVER_DONE_EVENT because we are not running");
        }
    }
}

tResult cRosJuryCommunication::ProcessJuryStruct(aadc::jury::juryAction action, int entry) {
    switch (action) {
        case action_getready:
            LOG_INFO(cString::Format("RosJuryCommunication: Received Request Ready with maneuver ID %d", entry));

            // Publish get ready to ros_oadrive
            publishGetReady();

            // Set the new maneuver we are expected to drive
            setManeuver(entry);

            // Publish the current maneuver 
            publishCurrentManeuver();
            break;
        case action_start:
            LOG_INFO(cString::Format("RosJuryCommunication: Received Run with maneuver ID %d", entry));
            // Publish start to ros_oadrive
            publishStart();

            break;
        case action_stop:
            LOG_INFO(cString::Format("RosJuryCommunication: Received Stop with maneuver ID %d", entry));
            // Publish stop to ros_oadrive
            publishStop();

            break;
    }

    RETURN_NOERROR;
}

tResult cRosJuryCommunication::Process(tTimeStamp tmTimeOfTrigger)
{
    // Read jury struct
    object_ptr<const ISample> pSample;
    while (IS_OK(m_oInputJuryStruct.GetNextSample(pSample)))
    {
        auto oDecoder = m_juryStructSampleFactory.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());      
        tJuryStruct juryInput;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlJuryStructId.maneuverEntry, &juryInput.i16ManeuverEntry));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlJuryStructId.actionId, &juryInput.i16ActionID));
        
        tInt8 i8ActionID = juryInput.i16ActionID;
        tInt16 i16entry = juryInput.i16ManeuverEntry;

        auto action = aadc::jury::juryAction(i8ActionID);

        RETURN_IF_FAILED(ProcessJuryStruct(action, i16entry));
    }

    // Read maneuver list
    object_ptr<const ISample> pSampleAnonymous;
    while (IS_OK(m_oInputManeuverList.GetNextSample(pSampleAnonymous)))
    {
        std::vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pSampleAnonymous->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (data.size() > 0)
        {//maneuverlist
            m_strManeuverFileString.Set(data.data(), data.size());
            LoadManeuverList();
        }

        publishStartUp();
    }

    m_callbackQueue->callAvailable();
    
    RETURN_NOERROR;
}

tResult cRosJuryCommunication::TransmitDriverStruct(tDriverStruct& driverStruct)
{
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
    {
        auto oCodec = m_driverStructSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.stateId, driverStruct.i16StateID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.maneuverEntry, driverStruct.i16ManeuverEntry));
    }

    m_oOutputDriverStruct << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cRosJuryCommunication::LoadManeuverList()
{
    m_sectorList.clear();
    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;

    //read first Sector Elem
    if (IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");

            if (IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for (cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tManeuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action = maneuverFromString((*itManeuverElem)->GetAttribute("action").GetPtr());
                    man.extra = (*itManeuverElem)->GetAttributeInt("extra", -1);
                    sector.sector.push_back(man);
                }
            }

            m_sectorList.push_back(sector);
        }
    }
    if (oSectorElems.size() > 0)
    {
        LOG_INFO("RosJuryCommunication: Loaded Maneuver file successfully.");
    }
    else
    {
        LOG_ERROR("RosJuryCommunication: no valid Maneuver Data found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}