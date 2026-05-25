cmake_minimum_required(VERSION 3.5)
project(perception)
set(CMAKE_CXX_STANDARD 17)

find_package(ML4KP REQUIRED HINTS $ENV{DIRTMP_PATH}/install/lib/cmake/ML4KP)
find_package(catkin REQUIRED
  roscpp 
  roslib
  std_msgs 
  geometry_msgs
  cv_bridge
  message_generation
  interface
  nodelet
  prx_models
  utils
  )
find_package(OpenCV 4 REQUIRED)

MESSAGE(STATUS "Torch_DIR: ${Torch_DIR}")
if(DEFINED ENV{Torch_DIR})
  MESSAGE(STATUS "Torch_DIR defined!")
  find_package(Torch REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
else()
  message("Torch not found - some executables may not build")
  add_definitions(-DTORCH_NOT_BUILT)
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
set(NODELET_LIB ${PROJECT_NAME}_nodelets)
catkin_package(
#  INCLUDE_DIRS include
  # LIBRARIES ${NODELET_LIB}
  CATKIN_DEPENDS nodelet interface message_runtime roscpp roslib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include 
${catkin_INCLUDE_DIRS}		
	)

file(GLOB NODELET_FILES ${PROJECT_SOURCE_DIR}/src/nodelets/*.cpp)
add_library(${NODELET_LIB} ${NODELET_FILES})
target_include_directories(${NODELET_LIB} PUBLIC include 
  ${catkin_INCLUDE_DIRS})

add_dependencies(${NODELET_LIB} ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODELET_LIB} 
                      ${catkin_LIBRARIES} 
                      ${OpenCV_LIBS} 
                      ML4KP::ML4KP
                      )
# if(catkin_EXPORTED_LIBRARIES)
# endif()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(perception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

file(GLOB EXECUTABLE_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
    # ${PROJECT_SOURCE_DIR}/src/prx/experiments/*.cpp)

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
                        ${OpenCV_LIBS}
                        ${TORCH_LIBRARIES}
                        ML4KP::ML4KP
                      )
endforeach()
# add_executable(video_gatherer_cpp src/vid_gather_cpp.cpp)
# add_executable(display_video_cpp src/display_video.cpp)
# add_executable(offline_rosbag_analysis_cpp src/offline_rosbag_analysis.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rosbag_record ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(video_gatherer_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(display_video_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(offline_rosbag_analysis_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Specify libraries to link a library or executable target against
# target_link_libraries(rosbag_record
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
# )

# target_link_libraries(video_gatherer_cpp
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
# )

# target_link_libraries(display_video_cpp
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
# )

# target_link_libraries(offline_rosbag_analysis_cpp
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
# )

#############
## Install ##
#############

file(GLOB SCRIPT_FILES ${PROJECT_SOURCE_DIR}/scripts/*.py)
catkin_install_python(PROGRAMS
  ${SCRIPT_FILES}
  # scripts/mushr_open_loop_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/test_video_resource.py scripts/vid_gather.py
#   scripts/test_video_resource.py scripts/rosbag_to_imgs.py
#   scripts/test_video_resource.py scripts/show_imgs.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS perception perception_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_perception.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
