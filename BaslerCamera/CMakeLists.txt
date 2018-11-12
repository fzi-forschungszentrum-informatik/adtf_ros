
cmake_minimum_required(VERSION 3.10.0)
project(better_basler_camera)

if (NOT TARGET adtf::filtersdk)
	find_package(ADTF COMPONENTS filtersdk)
endif()
	

set (SOURCEFILES        
    stdafx.h 
	BaslerCamera.h
	BaslerCamera.cpp)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ../../include
    SYSTEM ${PYLON_INCLUDE_DIRS}
    )
	
link_directories(
    ${PYLON_LIBRARY_DIRS}
)
	
if(WIN32) 
    #install Release/RelWithDebInfo
    install(FILES "${PYLON_BINARY_DIRS}/PylonBase_MD_VC120_v5_0.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    install(FILES "${PYLON_BINARY_DIRS}/PylonUsb_MD_VC120_V5_0_TL.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    install(FILES "${PYLON_BINARY_DIRS}/GenApi_MD_VC120_v3_0_Basler_pylon_v5_0.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    install(FILES "${PYLON_BINARY_DIRS}/GCBase_MD_VC120_v3_0_Basler_pylon_v5_0.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    install(FILES "${PYLON_BINARY_DIRS}/PylonC_MD_VC120_v5_0.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)

    install(FILES "${PYLON_BINARY_DIRS}/Log_MD_VC120_v3_0_Basler_pylon_v5_0.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    install(FILES "${PYLON_BINARY_DIRS}/MATHPARSER_MD_VC120_V3_0_BASLER_PYLON_V5_0.DLL" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    install(FILES "${PYLON_BINARY_DIRS}/XMLPARSER_MD_VC120_V3_0_BASLER_PYLON_V5_0.DLL" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    install(FILES "${PYLON_BINARY_DIRS}/NODEMAPDATA_MD_VC120_V3_0_BASLER_PYLON_V5_0.DLL" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
	
    #install Debug
    install(FILES "${PYLON_BINARY_DIRS}/PylonBase_MD_VC120_v5_0.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${PYLON_BINARY_DIRS}/PylonUsb_MD_VC120_V5_0_TL.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug )
    install(FILES "${PYLON_BINARY_DIRS}/GenApi_MD_VC120_v3_0_Basler_pylon_v5_0.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug )
    install(FILES "${PYLON_BINARY_DIRS}/GCBase_MD_VC120_v3_0_Basler_pylon_v5_0.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug )
    install(FILES "${PYLON_BINARY_DIRS}/PylonC_MD_VC120_v5_0.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Release Debug)

	install(FILES "${PYLON_BINARY_DIRS}/Log_MD_VC120_v3_0_Basler_pylon_v5_0.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${PYLON_BINARY_DIRS}/MATHPARSER_MD_VC120_V3_0_BASLER_PYLON_V5_0.DLL" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${PYLON_BINARY_DIRS}/XMLPARSER_MD_VC120_V3_0_BASLER_PYLON_V5_0.DLL" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${PYLON_BINARY_DIRS}/NODEMAPDATA_MD_VC120_V3_0_BASLER_PYLON_V5_0.DLL" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
endif(WIN32)
	
#we use the special ADTF CMake command to add an shared object/DLL which has an extension called .adtfplugin
adtf_add_streaming_service(${PROJECT_NAME} ${SOURCEFILES})

#install the program you need 
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
	TARGET
		${PROJECT_NAME}
	PLUGIN_SUBDIR
		bin
	DEPENDENT_PLUGINS
)
	
target_link_libraries(${PROJECT_NAME}
	LINK_PUBLIC
    ${PYLON_LIBS}
)

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER sensor)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER sensor)
