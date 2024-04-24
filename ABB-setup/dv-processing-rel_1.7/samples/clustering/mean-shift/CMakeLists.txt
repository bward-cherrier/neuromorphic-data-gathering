CMAKE_MINIMUM_REQUIRED(VERSION 3.22)

PROJECT(
	mean-shift-clustering-sample
	VERSION 0.1.0
	LANGUAGES C CXX)

SET(CMAKE_CXX_STANDARD 20)

IF(NOT TARGET dv::processing)
	FIND_PACKAGE(dv-processing 1.7.0 REQUIRED)
ENDIF()

ADD_EXECUTABLE(mean-shift mean-shift.cpp)

TARGET_LINK_LIBRARIES(mean-shift PRIVATE dv::processing)
