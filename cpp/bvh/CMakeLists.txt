set(BVH_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/bvh.cpp
)

add_executable(bvh)

target_sources(bvh
  PRIVATE ${BVH_SOURCES}
)

target_link_libraries(bvh PUBLIC
  glm::glm
  tinyobjloader
  polyscope
)
