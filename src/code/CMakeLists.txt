cmake_minimum_required(VERSION 2.8.9)
set(PROJECTNAME make-it-roll-elisa)
project(${PROJECTNAME})
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
INCLUDE_DIRECTORIES(
${YARP_INCLUDE_DIRS}
${ICUB_INCLUDE_DIRS}
)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()
add_executable(${PROJECTNAME} main.cpp)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ctrlLib)

install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES app/scripts/system.xml app/scripts/app.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR}/${PROJECTNAME})
icubcontrib_add_uninstall_target()
