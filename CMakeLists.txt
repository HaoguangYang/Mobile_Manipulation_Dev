# Require 3.6 to support pkg_check_modules IMPORTED_TARGET
cmake_minimum_required(VERSION 3.6)

project(Mobile_Manip VERSION 1.0)

# Set CMake flags
set(CMAKE_CXX_STANDARD 14)

set(MOBILE_MANIP_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(MOBILE_MANIP_EXT_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(MOBILE_MANIP_BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)

# Find dependencies
function(download_package PACKAGE_NAME)
    configure_file(${MOBILE_MANIP_CMAKE_DIR}/${PACKAGE_NAME}-CMakeLists.txt.in
        ${CMAKE_BINARY_DIR}/${PACKAGE_NAME}/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
endfunction()

find_package(ctrl_utils 1.0.0 QUIET)
if(NOT ctrl_utils_FOUND)
    download_package(ctrl_utils)
    find_package(ctrl_utils 1.0.0 REQUIRED)
endif()

find_package(realsense2 HINTS /usr/local/include/librealsense2)
set (LIB_REALSENSE_LIBRARIES 
		${realsense2_LIBRARY})

#option to use OTG
option(USE_OTG "use the Reflexxes Library for trajectory generation" ON)
if(USE_OTG)
	set (OTG_SOURCE_DIR ${PROJECT_SOURCE_DIR}/lib/ReflexxesTypeII)
	set (OTG_LIBRARY ${OTG_SOURCE_DIR}/Linux/x64/release/lib/shared/libReflexxesTypeII.so)

	set (PROJECT_DEFINITIONS -DUSING_OTG)
	set (OTG_INCLUDE_DIRS ${OTG_SOURCE_DIR}/include)
	set (OTG_SOURCE
			${PROJECT_SOURCE_DIR}/src/OTG.cc)
	include_directories(${OTG_INCLUDE_DIRS})
	add_definitions(-DUSING_OTG)

endif()

include_directories( ${PROJECT_SOURCE_DIR}/include
					 ${PROJECT_SOURCE_DIR}/lib)

link_libraries("-lrt -pthread")
link_libraries(ctrl_utils::ctrl_utils)

add_subdirectory(src)


