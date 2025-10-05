include(FetchContent)

FetchContent_Declare(
	tinyobjloader
	GIT_REPOSITORY https://github.com/tinyobjloader/tinyobjloader
	GIT_TAG release
)

FetchContent_Declare(
	fastgltf
	GIT_REPOSITORY https://github.com/spnda/fastgltf
	GIT_TAG v0.9.0
)

FetchContent_Declare(
	polyscope
	GIT_REPOSITORY https://github.com/nmwsharp/polyscope
	GIT_TAG v2.3.0
)

FetchContent_Declare(
	glm
	GIT_REPOSITORY https://github.com/g-truc/glm.git
	GIT_TAG 1.0.1
)