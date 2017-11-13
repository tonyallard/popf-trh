include_directories(${val_SOURCE_DIR} ${val_SOURCE_DIR}/parsing ${UTIL_SOURCE_DIR} ${PDDL_SOURCE_DIR})

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

add_library(KK KK.cpp)

target_link_libraries(KK PDDL UTIL)
