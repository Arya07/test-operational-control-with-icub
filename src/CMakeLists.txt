cmake_minimum_required(VERSION 2.8.9)

set(PROJECTNAME make-it-roll)
project(${PROJECTNAME})

FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(
	common
	${YARP_INCLUDE_DIRS}
	include
)

add_executable(${PROJECTNAME} main.cpp)
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)
#yarp_install(FILES ../app/scripts/system.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
