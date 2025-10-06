cmake_minimum_required(VERSION 3.25)

project(Abyss LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

include(cmake/libraries.cmake)
include(cmake/clang_format.cmake)

add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)

FetchContent_MakeAvailable(
  glm
  tinyobjloader
  polyscope
)

add_clang_format_target(format "build")

add_subdirectory(bvh)
add_subdirectory(kdtree)
add_subdirectory(ADS)





