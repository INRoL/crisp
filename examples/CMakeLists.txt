file(GLOB example_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

foreach(example_source IN LISTS example_sources)
  get_filename_component(example_name "${example_source}" NAME_WE)
  add_executable("${example_name}" "${example_source}")
  target_link_libraries("${example_name}" PRIVATE crisp::crisp)

  if(WIN32)
    add_custom_command(
      TARGET "${example_name}" POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "$<TARGET_FILE:crisp::crisp>"
      "$<TARGET_FILE_DIR:${example_name}>"
    )
  endif()
endforeach()
