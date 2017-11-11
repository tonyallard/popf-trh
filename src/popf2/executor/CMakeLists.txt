project(EXECUTOR)
include_directories(${val_SOURCE_DIR} ${val_SOURCE_DIR}/parsing ${UTIL_SOURCE_DIR} ${STN_SOURCE_DIR})

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(executor_build_srcs
	PlanExecutor.cpp
)
    
add_library(EXECUTOR ${executor_build_srcs})

target_link_libraries(EXECUTOR STN UTIL)
