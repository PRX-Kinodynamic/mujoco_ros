cmake_minimum_required(VERSION 3.15)
project(ml4kp_bridge)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

cmake_policy(SET CMP0177 NEW)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# Mujoco-relates packages
find_package(Eigen3 REQUIRED)
find_package(ML4KP REQUIRED HINTS $ENV{DIRTMP_PATH}/install/lib/cmake/ML4KP)
# Explicitly find Torch to ensure targets are available to downstream catkin packages
if(DEFINED ENV{Torch_DIR})
  find_package(Torch REQUIRED)
endif()
find_package(rostest REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  nodelet
  roslib
)

MESSAGE(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}.")
if (CMAKE_BUILD_TYPE MATCHES Release)
  # -03 <- Turn on all optims
  # -march=native <- Turn on eigen vectorization optimizations
  set(BUILD_FLAGS "-O3")
  # MESSAGE(STATUS "MARCH NATIVE: ${BUILD_WITH_SIMD}")
  # if (${GTSAM_BUILD_WITH_MARCH_NATIVE})
    list(APPEND BUILD_FLAGS "-march=native")
    MESSAGE(WARNING "USING MARCH NATIVE")
    # endif()
  add_definitions(-DRELEASE_BUILD)
elseif(CMAKE_BUILD_TYPE MATCHES DEBUG)
  set(BUILD_FLAGS "-O0 -fno-inline -g")
  # list(APPEND BUILD_FLAGS "-fsanitize=address")
  add_definitions(-DDEBUG_BUILD)
endif()

MESSAGE(STATUS "Build Flags: ${BUILD_FLAGS}")
add_definitions(${BUILD_FLAGS})


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
# add_message_files(
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
add_library(${PROJECT_NAME} INTERFACE)
catkin_package(
 INCLUDE_DIRS include ${ML4KP_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp std_msgs message_runtime 
 # DEPENDS gtsam ML4KP::External
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
get_target_property(ML4KP_External_DIRS ML4KP::External INTERFACE_INCLUDE_DIRECTORIES)

message(STATUS "ML4KP_External_DIRS ${ML4KP_External_DIRS}")
include_directories(
  include
  ${ML4KP_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${ML4KP_External_DIRS}
)

include(CMakePrintHelpers)
# cmake_print_variables(ML4KP::ML4KP)

MESSAGE(WARNING "GTSAM_INCLUDE_DIRECTORIES ${GTSAM_INCLUDE_DIRECTORIES}")
MESSAGE(WARNING "ML4KP_INCLUDE_DIR ${ML4KP_INCLUDE_DIR}")
MESSAGE(WARNING "INTERFACE_LINK_LIBRARIES ${INTERFACE_LINK_LIBRARIES}")
MESSAGE(WARNING "${${PROJECT_NAME}_EXPORTED_TARGETS}")
# set_target_properties(ML4KP::External PROPERTIES
#   INTERFACE_INCLUDE_DIRECTORIES "/Users/Gary/pracsys/gtsam/install/include/gtsam/3rdparty/SuiteSparse_config;/Users/Gary/pracsys/gtsam/install/include/gtsam/3rdparty/CCOLAMD;/Users/Gary/pracsys/gtsam/install/include/;${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include"
#   INTERFACE_LINK_LIBRARIES "yaml-cpp::yaml-cpp;gtsam;/Users/Gary/pracsys/ML4KP-devel/build/_deps/foxglove-src/lib/libfoxglove.a;torch;torch_library;/Users/Gary/3rd_party_sftw/libtorch/lib/libc10.dylib;/Users/Gary/3rd_party_sftw/libtorch/lib/libkineto.a;Boost::filesystem;Boost::system;Boost::serialization;Threads::Threads"
# )


cmake_print_properties(
  TARGETS ML4KP::External
  PROPERTIES INTERFACE_LINK_LIBRARIES  INTERFACE_INCLUDE_DIRECTORIES 
)
cmake_print_properties(
  TARGETS ML4KP::ML4KP
  PROPERTIES INTERFACE_LINK_LIBRARIES  INTERFACE_INCLUDE_DIRECTORIES 
)


find_package(GTest REQUIRED)

## Declare a C++ library
# Consider all src to add to the library (no executables in this package)
# file(GLOB_RECURSE ML4KP_BRIDGE_SRC  src/*.cpp)
# file(GLOB_RECURSE SRC_PRX_LIB_PLANNING  ./*.cpp)
add_dependencies(${PROJECT_NAME} ML4KP::ML4KP ML4KP::External)
target_link_libraries(${PROJECT_NAME} INTERFACE ML4KP::ML4KP ML4KP::External)


cmake_print_properties(
  TARGETS ${PROJECT_NAME}
  PROPERTIES INTERFACE_LINK_LIBRARIES INTERFACE_INCLUDE_DIRECTORIES 
)


# set_property(
#   TARGET ${PROJECT_NAME} PROPERTY INTERFACE_LINK_DIRECTORIES "")

# message(STATUS "EIGEN3_INCLUDE_DIR " ${EIGEN3_INCLUDE_DIR})

### Nodelets as a library
set(NODELET_LIB ${PROJECT_NAME}_nodelets)
file(GLOB NODELET_FILES ${PROJECT_SOURCE_DIR}/src/nodelets/*.cpp)
add_library(${NODELET_LIB} ${NODELET_FILES})
target_include_directories(${NODELET_LIB} PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(${NODELET_LIB} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODELET_LIB} ${catkin_LIBRARIES} ${PROJECT_NAME})

get_target_property(_INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_NAME} INTERFACE_INCLUDE_DIRECTORIES)

# message(STATUS "_INTERFACE_INCLUDE_DIRECTORIES: ${_INTERFACE_INCLUDE_DIRECTORIES}")
# message(STATUS "ML4KP_INCLUDE_DIR: ${ML4KP_INCLUDE_DIR}")
# message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
# message(STATUS "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")
### Executable files
file(GLOB EXECUTABLE_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
    # ${PROJECT_SOURCE_DIR}/src/prx/experiments/*.cpp)

# message(STATUS "${PROJECT_NAME}_EXPORTED_TARGETS  ${${PROJECT_NAME}_EXPORTED_TARGETS} ")
# message(STATUS "catkin_EXPORTED_TARGETS  ${catkin_EXPORTED_TARGETS} ")
# message(STATUS "catkin_LIBRARIES  ${catkin_LIBRARIES} ")

# include(CMakePrintHelpers)

# cmake_print_properties(
#   TARGETS ${PROJECT_NAME}
#   PROPERTIES LOCATION  INTERFACE_INCLUDE_DIRECTORIES 
# )

foreach(f ${EXECUTABLE_FILES})
    MESSAGE("file: ${f}")
    GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
    add_executable(${fname} ${f})
    add_dependencies(${fname}
                     ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                     ${catkin_EXPORTED_TARGETS}
                     ${PROJECT_NAME}
                     )
    target_link_libraries(${fname}
                          ${PROJECT_NAME}
                          ${catkin_LIBRARIES}
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

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# install(TARGETS ML4KP::ML4KP
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
file(GLOB TESTS_FILES ${PROJECT_SOURCE_DIR}/tests/*.cpp)
set(gtest_disable_pthreads OFF)
foreach(f_test ${TESTS_FILES})
  get_filename_component(fname ${f_test} NAME_WE )
  set(test_name "test_${fname}")
  MESSAGE(STATUS "Test: ${test_name}")
  catkin_add_gtest(${test_name} ${f_test})
  target_link_libraries(${test_name} ${catkin_LIBRARIES} ${PROJECT_NAME}  )
endforeach()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
