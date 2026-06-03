function(crisp_fetch_eigen eigen_version)
  if(TARGET Eigen3::Eigen)
    message(FATAL_ERROR "Eigen3::Eigen target already exists")
  endif()

  set(eigen_source_dir "${CMAKE_BINARY_DIR}/_deps/eigen-src")
  if(NOT EXISTS "${eigen_source_dir}/Eigen")
    include(FetchContent)
    message(STATUS "Fetching Eigen ${eigen_version}")
    FetchContent_Populate(
      Eigen3
      GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
      GIT_TAG "${eigen_version}"
      SOURCE_DIR "${eigen_source_dir}"
    )
  endif()

  if(NOT EXISTS "${eigen_source_dir}/Eigen")
    message(FATAL_ERROR "Eigen headers not found: ${eigen_source_dir}")
  endif()

  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  target_include_directories(Eigen3::Eigen INTERFACE "${eigen_source_dir}")
endfunction()
