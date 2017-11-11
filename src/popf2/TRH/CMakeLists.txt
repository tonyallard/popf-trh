cmake_minimum_required(VERSION 2.4)

include_directories(${val_SOURCE_DIR} ${val_SOURCE_DIR}/parsing ${PDDL_SOURCE_DIR})

project(TRH)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(libTRH_SRCS
	PlannerExecutionReader.cpp
	TRH.cpp
)

add_library(TRH ${libTRH_SRCS})
target_link_libraries(TRH UTIL PDDL HRELAX)
SET_TARGET_PROPERTIES(TRH PROPERTIES COMPILE_FLAGS "-DPOPF3ANALYSIS")

