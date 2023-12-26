

execute_process(
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/aruco_mj_tags_generator.bash ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE out_msg
  )
message(STATUS "${out_msg}")
