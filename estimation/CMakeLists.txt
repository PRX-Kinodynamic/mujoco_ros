cmake_minimum_required(VERSION 3.5)
project(estimation)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Mujoco-relates packages
find_package(OpenCV REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  interface
  ml4kp_bridge
  roscpp
  std_msgs
  roslib
  prx_models
  sensor_msgs
  nodelet
  tf
  tf_conversions
  utils
)

find_package(GTest REQUIRED)

if(DEFINED ENV{Torch_DIR})
  find_package(Torch REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
endif()
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
 CATKIN_DEPENDS roscpp std_msgs message_runtime prx_models ml4kp_bridge
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


## Nodeletes
set(NODELET_LIB ${PROJECT_NAME}_nodelets)
file(GLOB NODELET_FILES ${PROJECT_SOURCE_DIR}/src/nodelets/*.cpp)
add_library(${NODELET_LIB} ${NODELET_FILES})
target_include_directories(${NODELET_LIB} PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(${NODELET_LIB} ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODELET_LIB} ${catkin_LIBRARIES} ${OpenCV_LIBS})
file(GLOB EXECUTABLE_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

## Executables
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
                        ${MJ_LINK}
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
file(GLOB SCRIPT_FILES ${PROJECT_SOURCE_DIR}/scripts/*.py)
catkin_install_python(PROGRAMS
  ${SCRIPT_FILES}
  # scripts/mushr_open_loop_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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

file(GLOB TESTS_FILES ${PROJECT_SOURCE_DIR}/tests/*.cpp)
set(gtest_disable_pthreads OFF)
foreach(f_test ${TESTS_FILES})
  get_filename_component(fname ${f_test} NAME_WE )
  set(test_name "test_${fname}")
  MESSAGE(STATUS "Test: ${test_name}")
  catkin_add_gtest(${test_name} ${f_test})
  target_link_libraries(${test_name} ${catkin_LIBRARIES}  )
endforeach()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mujoco.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
