set(KDTREE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/kdtree.cpp
)

add_executable(kdtree)

target_sources(kdtree
  PRIVATE ${KDTREE_SOURCES}
)

target_link_libraries(kdtree PUBLIC
  glm::glm
  polyscope
  tinyobjloader
)
