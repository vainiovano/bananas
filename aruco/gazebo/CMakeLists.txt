function(add_world)
  cmake_parse_arguments(PARSE_ARGV 0 arg "" WORLD OUT_BOARDS)

  set(OUT_BOARD_GLBS ${arg_OUT_BOARDS})
  list(TRANSFORM OUT_BOARD_GLBS APPEND ".glb")
  set(OUT_BOARD_SDFS ${arg_OUT_BOARDS})
  list(TRANSFORM OUT_BOARD_SDFS APPEND ".sdf")

  set(OUT_BOARD_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/boards/)
  set(OUT_BOARD_FILES ${OUT_BOARD_GLBS} ${OUT_BOARD_SDFS})
  list(TRANSFORM OUT_BOARD_FILES PREPEND ${OUT_BOARD_PREFIX})
  message(STATUS "OUT_BOARD_FILES=${OUT_BOARD_FILES}")
  message(STATUS "WORLD=${arg_WORLD}")

  set(BOARD_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/boards.json)
  add_custom_command(
    OUTPUT ${OUT_BOARD_FILES}
    COMMAND gltf_exporter -o=boards --sdf ${BOARD_SOURCE}
    DEPENDS ${BOARD_SOURCE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  add_custom_target(process-${arg_WORLD}-boards ALL
                    DEPENDS ${OUT_BOARD_FILES})

  set(COPIED_FILES ${arg_WORLD} boards.json camera.json
                   static_environment.json)
  list(TRANSFORM COPIED_FILES PREPEND ${CMAKE_CURRENT_SOURCE_DIR}/
                                      OUTPUT_VARIABLE COPY_SOURCE_FILES)
  list(TRANSFORM COPIED_FILES PREPEND ${CMAKE_CURRENT_BINARY_DIR}/
                                      OUTPUT_VARIABLE COPY_DESTINATION_FILES)

  add_custom_command(
    OUTPUT ${COPY_DESTINATION_FILES}
    COMMAND ${CMAKE_COMMAND} -E copy ${COPY_SOURCE_FILES}
            ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${COPY_SOURCE_FILES})
  add_custom_target(copy-${arg_WORLD}-config ALL
                    DEPENDS ${COPY_DESTINATION_FILES})
endfunction()

add_subdirectory(box_world)
add_subdirectory(demo_world)
add_subdirectory(drone_world)
