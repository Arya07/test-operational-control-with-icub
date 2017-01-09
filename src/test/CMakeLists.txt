cmake_minimum_required(VERSION 2.8.9)
project(Operational-control-Test)

# find a version of RTF higher then 1.0.1
# and with yarp support enabled
find_package(RTF 1.0.1 COMPONENTS YARP REQUIRED)

# set the output plugin directory to collect all the shared libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

# options
option(ICUB_TESTS_USES_ICUB_MAIN "Turn on to compile the tests that depend on the icub-main repository" OFF)
option(ICUB_TESTS_USES_CODYCO    "Turn on to compile the test that depend on the codyco-superbuil repository" OFF)

# Build gaze controller tests
add_subdirectory(src)


