cmake_minimum_required(VERSION 3.5)
project(utils)
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(rostest REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  roslib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# set(MSG_DIRS "mushr")
set(MSG_FILES "")
file(GLOB_RECURSE MSG_DIRS "msg/*.msg")
foreach(M_DIR ${MSG_DIRS})
  GET_FILENAME_COMPONENT(M_NAME ${M_DIR} NAME_WE )
  GET_FILENAME_COMPONENT(D_NAME ${M_DIR} DIRECTORY )
  add_message_files(DIRECTORY "${D_NAME}" FILES "${M_NAME}.msg")
endforeach()

## Generate services in the 'srv' folder
set(SRV_FILES "")
file(GLOB_RECURSE SRV_DIRS "srv/*.srv")
foreach(S_DIR ${SRV_DIRS})
  GET_FILENAME_COMPONENT(S_NAME ${S_DIR} NAME_WE )
  GET_FILENAME_COMPONENT(D_NAME ${S_DIR} DIRECTORY )
  add_service_files(DIRECTORY "${D_NAME}" FILES "${S_NAME}.srv")
endforeach()

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES mujoco
 CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)

### Nodelets as a library
set(NODELET_LIB ${PROJECT_NAME}_nodelets)
file(GLOB NODELET_FILES ${PROJECT_SOURCE_DIR}/src/nodelets/*.cpp)
list(LENGTH NODELET_FILES TOTAL_NODELET_FILES)
if (${TOTAL_NODELET_FILES} GREATER "0")
  add_library(${NODELET_LIB} ${NODELET_FILES})
  target_include_directories(${NODELET_LIB} PUBLIC include  ${catkin_INCLUDE_DIRS})
  add_dependencies(${NODELET_LIB} ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${NODELET_LIB} ${catkin_LIBRARIES})
endif()

### Executable files
file(GLOB EXECUTABLE_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
foreach(f ${EXECUTABLE_FILES})
  get_filename_component(fname ${f} NAME_WE )
  add_executable(${fname} ${f})
  ## Add cmake target dependencies of the executable
  ## same as for the library above
  add_dependencies(${fname}
                   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                   ${catkin_EXPORTED_TARGETS}
                   )
  ## Specify libraries to link a library or executable target against
  target_link_libraries(${fname}
                        ${catkin_LIBRARIES}
                        ${OpenCV_LIBS} 
                      )
endforeach()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/mushr_open_loop_publisher.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mujoco.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
file(GLOB TESTS_FILES ${PROJECT_SOURCE_DIR}/tests/*.cpp)
set(gtest_disable_pthreads OFF)
foreach(f_test ${TESTS_FILES})
  get_filename_component(fname ${f_test} NAME_WE )
  set(test_name "test_${fname}")
  MESSAGE(STATUS "Test: ${test_name}")
  catkin_add_gtest(${test_name} ${f_test})
  target_link_libraries(${test_name} ${catkin_LIBRARIES}  )
endforeach()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
