cmake_minimum_required(VERSION 3.10.0)

project(adtf_ros_ultrasonic_publisher)


find_package(catkin REQUIRED
    COMPONENTS
        ros_oadrive
        roscpp
        std_msgs
)

include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set (SOURCEFILES    	stdafx.h 
						AdtfRosUltrasonicPublisher.h
						AdtfRosUltrasonicPublisher.cpp)

adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 ) 

target_link_libraries(${PROJECT_NAME}  LINK_PUBLIC ${catkin_LIBRARIES})

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
