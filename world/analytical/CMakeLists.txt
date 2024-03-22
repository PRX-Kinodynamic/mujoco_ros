cmake_minimum_required(VERSION 3.0.2)
project(analytical)
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  roslib
  interface
  utils
)


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
set(MSG_FILES "")
file(GLOB_RECURSE MSG_DIRS "msg/*.msg")
foreach(M_DIR ${MSG_DIRS})
  GET_FILENAME_COMPONENT(M_NAME ${M_DIR} NAME_WE )
  GET_FILENAME_COMPONENT(D_NAME ${M_DIR} DIRECTORY )
  add_message_files(DIRECTORY "${D_NAME}" FILES "${M_NAME}.msg")
endforeach()

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


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
#  INCLUDE_DIRS include
#  LIBRARIES mujoco
 CATKIN_DEPENDS roscpp std_msgs message_runtime
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
)

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
                      )
endforeach()

#############
## Install ##
#############

file(GLOB SCRIPT_FILES ${PROJECT_SOURCE_DIR}/scripts/*.py)
catkin_install_python(PROGRAMS
  ${SCRIPT_FILES}
  # scripts/mushr_open_loop_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
