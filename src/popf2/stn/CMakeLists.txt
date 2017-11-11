project(STN)

include_directories(${val_SOURCE_DIR} ${val_SOURCE_DIR}/parsing ${UTIL_SOURCE_DIR} ${PDDL_SOURCE_DIR})

add_library(STN ColinSTNImpl.cpp)

target_link_libraries(STN PDDL UTIL)