# Sequence
* Jury sends Maneuvers and RoadSigns
    * We send StartUp
* Jury sends GetReady
    * GET_READY_EVENT 
    * ros_oadrive has to reset everything now and prepare to start
    * We will also publish the next maneuver to ros_oadrive here!
* ROS sends READY_EVENT
    * We tell the jury that we are ready for the current maneuver
* Jury sends Start
    * START_EVENT
    * The car must now start its run
* ros_oadrive may send MANEUVER_DONE_EVENT to indicate the completion of a maneuver
    * The next maneuver will be published to ros_oadrive
    * We tell the jury that we are now running the next maneuver
* The jury may send Stop
    * STOP_EVENT
    * ros_oadrive has to stop driving (do not do anything else!)

# Published events
## GET_READY_EVENT
Published when ros_oadrive has to get ready, ros_oadrive has to reset itself answer with READY_EVENT

## START_EVENT
Tells ros_oadrive that it has to start its run. The current maneuver will be published simultaneously with the event

## STOP_EVENT
Tells the stack to stop (e.g. if we are done or if the jury sent stop to reset us)


# Processed events
## READY_EVENT
Tells the Jury that we are ready

## MANEUVER_DONE_EVENT
Tells the jury that we completed the current event and publishes the next event to ros_oadrive. Stops the stack if we are done
