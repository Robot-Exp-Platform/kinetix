cmake_minimum_required(VERSION 3.18)

project(kinetix_pinocchio_benchmarks LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(KINETIX_PINOCCHIO_SOURCE_DIR "" CACHE PATH "Optional local Pinocchio source tree")
option(
  KINETIX_PINOCCHIO_HEADER_ONLY
  "Use local Pinocchio headers directly with generated benchmark-only config headers"
  ON
)

if(KINETIX_PINOCCHIO_SOURCE_DIR)
  if(KINETIX_PINOCCHIO_HEADER_ONLY)
    find_package(Eigen3 CONFIG REQUIRED)
    find_package(Boost REQUIRED COMPONENTS filesystem serialization)

    set(PINOCCHIO_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
    file(MAKE_DIRECTORY "${PINOCCHIO_GENERATED_INCLUDE_DIR}/pinocchio")

    file(
      WRITE "${PINOCCHIO_GENERATED_INCLUDE_DIR}/pinocchio/config.hpp"
      "#pragma once\n"
      "#define PINOCCHIO_VERSION \"4.0.0-local\"\n"
      "#define PINOCCHIO_VERSION_MAJOR 4\n"
      "#define PINOCCHIO_VERSION_MINOR 0\n"
      "#define PINOCCHIO_VERSION_PATCH 0\n"
      "#define PINOCCHIO_VERSION_AT_LEAST(major, minor, patch) "
      "((PINOCCHIO_VERSION_MAJOR > (major)) || "
      "(PINOCCHIO_VERSION_MAJOR == (major) && PINOCCHIO_VERSION_MINOR > (minor)) || "
      "(PINOCCHIO_VERSION_MAJOR == (major) && PINOCCHIO_VERSION_MINOR == (minor) && PINOCCHIO_VERSION_PATCH >= (patch)))\n"
      "#define PINOCCHIO_DLLAPI\n"
      "#define PINOCCHIO_PARSERS_DLLAPI\n"
      "#define PINOCCHIO_EXTRA_DLLAPI\n"
      "#define PINOCCHIO_VISUALIZERS_DLLAPI\n"
      "#define PINOCCHIO_PYTHON_PARSER_DLLAPI\n"
      "#define PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI\n"
      "#define PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI\n"
      "#define PINOCCHIO_COLLISION_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI\n"
    )
    file(
      WRITE "${PINOCCHIO_GENERATED_INCLUDE_DIR}/pinocchio/deprecated.hpp"
      "#pragma once\n"
      "#define PINOCCHIO_DEPRECATED [[deprecated]]\n"
      "#define PINOCCHIO_DEPRECATED_MESSAGE(message) [[deprecated(message)]]\n"
      "#define PINOCCHIO_DEPRECATED_HEADER(message)\n"
    )
    file(
      WRITE "${PINOCCHIO_GENERATED_INCLUDE_DIR}/pinocchio/warning.hpp"
      "#pragma once\n"
      "#define PINOCCHIO_PRAGMA(message)\n"
    )

    add_library(pinocchio_local_headers INTERFACE)
    add_library(pinocchio::pinocchio ALIAS pinocchio_local_headers)
    target_include_directories(
      pinocchio_local_headers
      INTERFACE
        "${PINOCCHIO_GENERATED_INCLUDE_DIR}"
        "${KINETIX_PINOCCHIO_SOURCE_DIR}/include"
        "${KINETIX_PINOCCHIO_SOURCE_DIR}/include/pinocchio/deprecated"
    )
    target_compile_definitions(
      pinocchio_local_headers
      INTERFACE
        BOOST_MPL_LIMIT_LIST_SIZE=30
        BOOST_MPL_LIMIT_VECTOR_SIZE=30
        BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
        BOOST_FUSION_INVOKE_MAX_ARITY=12
        PINOCCHIO_DISABLE_UNSUPPORTED_WARNINGS
    )
    target_link_libraries(
      pinocchio_local_headers
      INTERFACE Eigen3::Eigen Boost::boost Boost::filesystem Boost::serialization
    )
  else()
    set(BUILD_BENCHMARK OFF CACHE BOOL "" FORCE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(BUILD_UTILS OFF CACHE BOOL "" FORCE)
    set(BUILD_PYTHON_INTERFACE OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_URDF_SUPPORT OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_SDF_SUPPORT OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_COLLISION_SUPPORT OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_AUTODIFF_SUPPORT OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_CASADI_SUPPORT OFF CACHE BOOL "" FORCE)
    set(BUILD_WITH_OPENMP_SUPPORT OFF CACHE BOOL "" FORCE)
    set(INSTALL_DOCUMENTATION OFF CACHE BOOL "" FORCE)
    add_subdirectory("${KINETIX_PINOCCHIO_SOURCE_DIR}" pinocchio-from-source)
  endif()
else()
  find_package(pinocchio REQUIRED)
endif()

add_executable(pinocchio_bench bench_pinocchio.cpp)
target_link_libraries(pinocchio_bench PRIVATE pinocchio::pinocchio)

if(MSVC)
  target_compile_options(pinocchio_bench PRIVATE /O2 /DNDEBUG /bigobj /utf-8)
  target_compile_definitions(pinocchio_bench PRIVATE _WIN32_WINNT=0x0A00 NOMINMAX)
else()
  target_compile_options(pinocchio_bench PRIVATE -O3 -DNDEBUG -march=native)
endif()
