# 

cmake_minimum_required(VERSION 2.8.7)

PROJECT(Plugin_LinkWorldPose)

find_package(gazebo REQUIRED)
find_package(YARP REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(HEADERS_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/include/gazebo")
set(SRC_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/src")

set(HEADERS    	${HEADERS_FOLDER}/GetLinkWorldPose.hh)

set(SOURCES     ${SRC_FOLDER}/GetLinkWorldPose.cc)

include_directories(${GAZEBO_INCLUDE_DIRS}
					${YARP_INCLUDE_DIRS})
					
include_directories(${HEADERS_FOLDER})
					
link_directories(${GAZEBO_LIBRARY_DIRS})


add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${YARP_LIBRARIES})
