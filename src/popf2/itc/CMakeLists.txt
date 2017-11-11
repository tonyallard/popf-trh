project(ITC)

include_directories(${val_SOURCE_DIR} ${val_SOURCE_DIR}/parsing ${UTIL_SOURCE_DIR} ${STN_SOURCE_DIR})

add_library(ITC ITC.cpp)

target_link_libraries(ITC STN UTIL)