cmake_minimum_required(VERSION 3.20)
project(rkmpp_harness CXX)
set(CMAKE_CXX_STANDARD 17)
enable_testing()

# Whether to build the host-runnable unit tests (FormatCaps, BuildNoopJob).
# Defaults ON when target arch == host arch; OFF when cross-compiling, since
# we cannot execute the test binary on the host in that case.
if(CMAKE_VS_PLATFORM_NAME STREQUAL CMAKE_HOST_SYSTEM_PROCESSOR
   OR (CMAKE_VS_PLATFORM_NAME STREQUAL "x64" AND CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "AMD64")
   OR (CMAKE_VS_PLATFORM_NAME STREQUAL "ARM64" AND CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "ARM64"))
    set(_default_tests ON)
else()
    set(_default_tests OFF)
endif()
option(RKMPP_HARNESS_TESTS "Build the host-runnable unit tests" ${_default_tests})

message(STATUS "rkmpp_harness: target=${CMAKE_VS_PLATFORM_NAME} host=${CMAKE_HOST_SYSTEM_PROCESSOR} tests=${RKMPP_HARNESS_TESTS}")

# Pull in the user-mode helper library (parser_glue, regbuilder, …).
add_subdirectory(${CMAKE_SOURCE_DIR}/../../mft ${CMAKE_BINARY_DIR}/mft)

# decode_engine static lib (parser → dpb → packed_tables → regbuilder
# wrapped around the rkmpp.sys IOCTL surface).  Must be added before
# mft/dll so the DLL can link it.
add_subdirectory(rkmpp_decode)

# IMFTransform DLL (Phase 2B — wired through decode_engine).
add_subdirectory(${CMAKE_SOURCE_DIR}/../../mft/dll ${CMAKE_BINARY_DIR}/mft_dll)

add_subdirectory(rkmpp_caps_dump)
add_subdirectory(rkmpp_smoke)
add_subdirectory(rkmpp_iommu_fault)
add_subdirectory(parser_test)
add_subdirectory(avcc_test)
add_subdirectory(parser_test_h265)
add_subdirectory(regbuilder_test)
add_subdirectory(packed_tables_test)
add_subdirectory(dpb_test)
add_subdirectory(mp4_extract)
add_subdirectory(mft_smoke)
add_subdirectory(mft_decode)
add_subdirectory(mft_d3d_smoke)
add_subdirectory(mft_play)
