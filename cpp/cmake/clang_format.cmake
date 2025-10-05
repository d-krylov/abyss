set(CLANG_FORMAT_NAMES
  clang-format 
  clang-format-15 
  clang-format-11 
)

function(add_clang_format_target TARGET_NAME EXCLUDE_FROM_FORMAT)
  find_program(CLANG_FORMAT_EXECUTABLE NAMES ${CLANG_FORMAT_NAMES} REQUIRED)

  if(NOT CLANG_FORMAT_EXECUTABLE)
    message(FATAL_ERROR "clang-format not found")
  endif()

  file(GLOB_RECURSE SOURCES_FOR_FORMAT 
    RELATIVE "${PROJECT_SOURCE_DIR}"  
    "*.cpp" "*.h" "*.ipp"
  )

  foreach(EXCLUDE_DIR ${EXCLUDE_FROM_FORMAT})
    list(FILTER SOURCES_FOR_FORMAT EXCLUDE REGEX "${EXCLUDE_DIR}/")
  endforeach()

  add_custom_target(${TARGET_NAME}
    COMMAND ${CLANG_FORMAT_EXECUTABLE} -i ${SOURCES_FOR_FORMAT}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
endfunction()