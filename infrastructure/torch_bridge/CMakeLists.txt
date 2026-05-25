cmake_minimum_required(VERSION 3.5)
project(torch_bridge)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
cmake_policy(SET CMP0177 NEW)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  roslib
  tf2_ros
  tf2
  utils
  ml4kp_bridge
)

catkin_python_setup()

find_package(Eigen3 CONFIG REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

MESSAGE(STATUS "Torch_DIR: ${Torch_DIR}")
if(DEFINED ENV{Torch_DIR})
  MESSAGE(STATUS "Torch_DIR defined!")
  find_package(Torch REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
else()
  message("Please download libtorch C++, place in external folder and setup the enviornmental variable Torch_DIR accordingly.")
  add_definitions(-DTORCH_NOT_BUILT)
endif()

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
set(MSG_FILES "")
file(GLOB_RECURSE MSG_DIRS "msg/*.msg")
foreach(M_DIR ${MSG_DIRS})
  GET_FILENAME_COMPONENT(M_NAME ${M_DIR} NAME_WE )
  GET_FILENAME_COMPONENT(D_NAME ${M_DIR} DIRECTORY )
  add_message_files(DIRECTORY "${D_NAME}" FILES "${M_NAME}.msg")
endforeach()

set(SRV_FILES "")
file(GLOB_RECURSE SRV_DIRS "srv/*.srv")
foreach(S_DIR ${SRV_DIRS})
  GET_FILENAME_COMPONENT(S_NAME ${S_DIR} NAME_WE )
  GET_FILENAME_COMPONENT(D_NAME ${S_DIR} DIRECTORY )
  add_service_files(DIRECTORY "${D_NAME}" FILES "${S_NAME}.srv")
endforeach()

set(ACTION_FILES "")
file(GLOB_RECURSE ACTION_FILES "action/*.action")
foreach(A_DIR ${ACTION_FILES})
  GET_FILENAME_COMPONENT(A_NAME ${A_DIR} NAME_WE )
  GET_FILENAME_COMPONENT(D_NAME ${A_DIR} DIRECTORY )
  add_action_files(DIRECTORY "${D_NAME}" FILES "${A_NAME}.msg")
endforeach()


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
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
 INCLUDE_DIRS include ${TORCH_INCLUDE_DIRS}
 # LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp std_msgs message_runtime 
 # DEPENDS ${TORCH_LIBRARIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} INTERFACE)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
	# ${TORCH_LIBRARIES}
	)

target_link_libraries(${PROJECT_NAME} INTERFACE ${TORCH_LIBRARIES})
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/mujoco_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

### Nodelets as a library
set(NODELET_LIB ${PROJECT_NAME}_nodelets)
file(GLOB NODELET_FILES ${PROJECT_SOURCE_DIR}/src/nodelets/*.cpp)
list(LENGTH NODELET_FILES TOTAL_NODELET_FILES)
if (${TOTAL_NODELET_FILES} GREATER "0")
	add_library(${NODELET_LIB} ${NODELET_FILES})
	target_include_directories(${NODELET_LIB} PUBLIC include ${catkin_INCLUDE_DIRS})
	add_dependencies(${NODELET_LIB} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(${NODELET_LIB} ${catkin_LIBRARIES} ${PROJECT_NAME} ${TORCH_LIBRARIES})
endif()

file(GLOB EXECUTABLE_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
foreach(f ${EXECUTABLE_FILES})
    MESSAGE("file: ${f}")
    GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
    add_executable(${fname} ${f})
    add_dependencies(${fname}
                     ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                     ${catkin_EXPORTED_TARGETS}
                     )
    target_link_libraries(${fname}
                          ${catkin_LIBRARIES}
                          ${TORCH_LIBRARIES}
                        )
endforeach()

#############
## Install ##
#############

file(GLOB SCRIPT_FILES ${PROJECT_SOURCE_DIR}/scripts/*.py)
catkin_install_python(PROGRAMS
  ${SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install bash wrapper scripts
install(PROGRAMS
  scripts/torch_service_server_wrapper.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  # PATTERN ".svn" EXCLUDE
)

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
file(GLOB TESTS_FILES ${PROJECT_SOURCE_DIR}/tests/*.cpp)

foreach(f_test ${TESTS_FILES})
  catkin_add_gtest(${PROJECT_NAME}-test ${f_test})
  MESSAGE(STATUS "${PROJECT_NAME}-test ${f_test}")
  if(TARGET ${PROJECT_NAME}-test)
    MESSAGE(STATUS "TARGET  ${PROJECT_NAME}-test ")
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${TORCH_LIBRARIES} )
  endif()
endforeach()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

