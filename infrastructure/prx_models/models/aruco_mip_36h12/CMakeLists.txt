

execute_process(
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/aruco_mj_tags_generator.bash ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE outVar
  )
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "outVar: ${outVar}")
