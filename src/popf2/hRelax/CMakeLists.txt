project(HRELAX)
include_directories(${val_SOURCE_DIR} ${val_SOURCE_DIR}/parsing ${UTIL_SOURCE_DIR} ${PDDL_SOURCE_DIR} {$KK_SOURCE_DIR} ${STN_SOURCE_DIR} ${ITC_SOURCE_DIR} ${EXECUTOR_SOURCE_DIR})

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(hrelax_build_srcs
	TemporalConflictRelaxation.cpp
    HRelax.cpp)
    
add_library(HRELAX ${hrelax_build_srcs})

target_link_libraries(HRELAX EXECUTOR ITC STN KK PDDL UTIL)
