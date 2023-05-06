cmake_minimum_required(VERSION 3.15)
PROJECT(stereo-camera)

set(CMAKE_CXX_STANDARD 17)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

MACRO(add_example name)
    ADD_EXECUTABLE( ${name} ${name}.cpp )
    TARGET_LINK_LIBRARIES( ${name} ${OpenCV_LIBS} )
ENDMACRO()

add_example(capture_images)
add_example(calibrate)
add_example(estimateZ)
add_example(calibrateZ)
add_example(record_stereo_video)

# debug, release mode
if (CMAKE_BUILD_TYPE STREQUAL "")
    # Build type is not set eg. command was "cmake .."
    message(STATUS "  Diag: Build type was unspecified, set to Release")
    set(CMAKE_BUILD_TYPE Release)
else ()
    message(STATUS "  Diag: Build type specified as '${CMAKE_BUILD_TYPE}'")
endif ()

if (${CMAKE_BUILD_TYPE} STREQUAL Debug)
    set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "_DEBUG")
else ()
    set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "NDEBUG")
endif ()