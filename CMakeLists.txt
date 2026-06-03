cmake_minimum_required(VERSION 3.20)
project(crisp LANGUAGES CXX)

set(crisp_prefix "${PROJECT_SOURCE_DIR}/crisp")

option(crisp_download_latest_release "Download the latest crisp release" ON)
option(crisp_verify_release "Verify downloaded crisp release checksum" ON)
option(crisp_use_bundled_eigen "Use bundled Eigen instead of find_package" OFF)

if(crisp_use_bundled_eigen)
  include(cmake/fetch_eigen.cmake)
  crisp_fetch_eigen(3.4.0)
else()
  find_package(Eigen3 3.4.0 CONFIG REQUIRED)
endif()

if(crisp_download_latest_release)
  include(cmake/download_crisp_release.cmake)
  crisp_download_release("${crisp_prefix}" latest)
  find_package(crisp CONFIG REQUIRED PATHS "${crisp_prefix}" NO_DEFAULT_PATH)
else()
  find_package(crisp CONFIG REQUIRED)
endif()

add_subdirectory(examples)
