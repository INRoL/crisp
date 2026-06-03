function(crisp_download_required_file url output error_prefix)
  file(DOWNLOAD "${url}" "${output}" TLS_VERIFY ON STATUS download_status)

  list(GET download_status 0 download_code)
  if(NOT download_code EQUAL 0)
    list(GET download_status 1 download_message)
    message(FATAL_ERROR "${error_prefix}: ${download_message}")
  endif()
endfunction()

function(crisp_download_release install_prefix release_tag)
  if(EXISTS "${install_prefix}/lib/cmake/crisp/crisp-config.cmake")
    return()
  endif()

  set(repo "INRoL/crisp")

  set(cache_dir "${CMAKE_BINARY_DIR}/_deps/crisp")
  file(MAKE_DIRECTORY "${cache_dir}")

  if(release_tag STREQUAL "latest")
    set(latest_json "${cache_dir}/latest.json")
    set(api_url "https://api.github.com/repos/${repo}/releases/latest")
    crisp_download_required_file(
      "${api_url}" "${latest_json}" "Failed to query latest release"
    )

    file(READ "${latest_json}" release_body)
    string(JSON tag ERROR_VARIABLE tag_error GET "${release_body}" tag_name)
    if(tag_error)
      message(FATAL_ERROR "Could not find latest release tag: ${tag_error}")
    endif()
  else()
    set(tag "${release_tag}")
  endif()

  string(REGEX MATCH "^v([0-9]+\\.[0-9]+\\.[0-9]+)$" version_match "${tag}")
  if(NOT version_match)
    message(FATAL_ERROR "Unsupported crisp release tag: ${tag}")
  endif()
  set(version "${CMAKE_MATCH_1}")
  message(STATUS "crisp release: ${tag}")

  if(WIN32)
    set(platform "windows-x86_64")
    set(ext "zip")
  elseif(APPLE)
    set(platform "macos")
    set(ext "tar.gz")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(platform "linux-aarch64")
    set(ext "tar.gz")
  else()
    set(platform "linux-x86_64")
    set(ext "tar.gz")
  endif()

  set(release_url "https://github.com/${repo}/releases/download/${tag}")
  set(asset "crisp-${version}-${platform}.${ext}")
  set(asset_url "${release_url}/${asset}")
  set(asset_file "${cache_dir}/${asset}")
  set(checksum "${asset}.sha256")
  set(checksum_url "${release_url}/${checksum}")
  set(checksum_file "${cache_dir}/${checksum}")

  message(STATUS "Release asset: ${asset_url}")
  if(NOT EXISTS "${asset_file}")
    crisp_download_required_file(
      "${asset_url}" "${asset_file}" "Failed to download release asset"
    )
  endif()

  if(crisp_verify_release)
    message(STATUS "Release checksum: ${checksum_url}")
    if(NOT EXISTS "${checksum_file}")
      crisp_download_required_file(
        "${checksum_url}" "${checksum_file}"
        "Failed to download release checksum"
      )
    endif()

    file(READ "${checksum_file}" checksum_body)
    string(REGEX MATCH "^[0-9A-Fa-f]+" expected_sha256 "${checksum_body}")
    string(LENGTH "${expected_sha256}" expected_sha256_length)
    if(NOT expected_sha256_length EQUAL 64)
      message(FATAL_ERROR "Invalid SHA256 checksum file: ${checksum_file}")
    endif()

    file(SHA256 "${asset_file}" actual_sha256)
    string(TOLOWER "${expected_sha256}" expected_sha256)
    string(TOLOWER "${actual_sha256}" actual_sha256)
    if(NOT expected_sha256 STREQUAL actual_sha256)
      message(FATAL_ERROR "SHA256 mismatch for release asset: ${asset}")
    endif()

    message(STATUS "Verified release SHA256: ${asset}")
  endif()

  file(MAKE_DIRECTORY "${install_prefix}")
  file(ARCHIVE_EXTRACT INPUT "${asset_file}" DESTINATION "${install_prefix}")

  if(NOT EXISTS "${install_prefix}/lib/cmake/crisp/crisp-config.cmake")
    message(FATAL_ERROR "crisp config not found: ${install_prefix}")
  endif()
endfunction()
