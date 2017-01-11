cmake_minimum_required(VERSION 2.8.9)
set(PROJECTNAME makeitrollTest-elisa)
project(${PROJECTNAME})

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(RTF)
find_package(RTF COMPONENTS DLL)


include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
		    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

add_library(${PROJECTNAME} MODULE src/MakeItRollTest.cpp src/MakeItRollTest.h)
#add_library(mytest2 MODULE src/MyTest2.cpp src/MyTest2.h)

target_link_libraries(${PROJECTNAME} ${RTF_LIBRARIES} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})

#add_executable(simple_run run.cpp)
#target_link_libraries(simple_run ${RTF_LIBRARIES})


#install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES suits/mySuit.xml suits/fixtures/my-fixture.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR}/${PROJECTNAME})
icubcontrib_add_uninstall_target()
