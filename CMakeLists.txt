cmake_minimum_required(VERSION 3.16)

project(form
	VERSION 0.1.0
	LANGUAGES CXX
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS Off)

option(FORM_BUILD_TESTS "Build FORM Tests" OFF)
option(FORM_BUILD_PYTHON "Build python evalio wrapper" OFF)

# ------------------------- Dependencies ------------------------- #
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(TBB REQUIRED)

include(FetchContent)
FetchContent_Declare(
    tsl
    GIT_REPOSITORY https://github.com/Tessil/robin-map.git
    GIT_TAG v1.4.0
	FIND_PACKAGE_ARGS CONFIG
)
FetchContent_MakeAvailable(tsl)

# ------------------------- form ------------------------- #
add_library(FORM
	form/form.cpp
	form/feature/factor.cpp
	form/optimization/constraints.cpp
	form/mapping/keyscanner.cpp
)
target_sources(FORM
	PUBLIC
	FILE_SET form_headers
	TYPE HEADERS
	BASE_DIRS .
	FILES 
		# feature
		form/feature/factor.hpp
		form/feature/features.hpp
		form/feature/extraction.hpp
		# optimization
		form/optimization/constraints.hpp
		form/optimization/gtsam.hpp
		# mapping
		form/mapping/map.hpp
		form/mapping/keyscanner.hpp
		# core
		form/form.hpp
		form/utils.hpp
)
target_link_libraries(FORM
	PUBLIC
	gtsam
	Eigen3::Eigen
	TBB::tbb
	tsl::robin_map
)

# Tests
if(${FORM_BUILD_TESTS})
	add_subdirectory(tests)
endif()

# python bindings
if(${FORM_BUILD_PYTHON})
	add_subdirectory(python)
endif()