# third_party/dav1d.cmake — expose dav1d to CMake consumers on both
# Windows (cross-built submodule) and Linux (system pkg-config).
#
# Usage from a CMakeLists.txt:
#     include(${CMAKE_SOURCE_DIR}/../../third_party/dav1d.cmake)
#     target_link_libraries(<my_target> PRIVATE dav1d)
#
# On Windows: caller must run third_party/build_dav1d.bat once before
# configuring CMake — meson handles the actual build, we just import
# the resulting static archive.
#
# On Linux (e.g. when cross-checking on the rk3588 board): we use the
# system libdav1d-dev package via pkg-config, since dav1d is already
# present from the BSP's ffmpeg dependency.

if(WIN32)
    set(_dav1d_root  "${CMAKE_CURRENT_LIST_DIR}/dav1d")

    # Pick the right pre-built dav1d archive for the active CMake target.
    # ARM64 build comes from build_dav1d.bat (cross-compiled from x64 host).
    # x64 build comes from build_dav1d_x64.bat (native host build, used for
    # dev-machine validation of the AV1 regbuilder + parser).
    if(CMAKE_VS_PLATFORM_NAME STREQUAL "ARM64"
       OR CMAKE_SYSTEM_PROCESSOR MATCHES "^(ARM64|aarch64)$")
        set(_dav1d_build_dir "build")
        set(_dav1d_helper    "build_dav1d.bat")
    else()
        set(_dav1d_build_dir "build-x64")
        set(_dav1d_helper    "build_dav1d_x64.bat")
    endif()

    set(_dav1d_lib   "${_dav1d_root}/${_dav1d_build_dir}/src/libdav1d.a")
    set(_dav1d_inc   "${_dav1d_root}/include"
                     "${_dav1d_root}/${_dav1d_build_dir}/include")

    if(NOT EXISTS "${_dav1d_lib}")
        message(FATAL_ERROR
            "dav1d static lib not found at ${_dav1d_lib}.\n"
            "Run third_party/${_dav1d_helper} from a developer shell first.")
    endif()

    add_library(dav1d STATIC IMPORTED GLOBAL)
    set_target_properties(dav1d PROPERTIES
        IMPORTED_LOCATION             "${_dav1d_lib}"
        INTERFACE_INCLUDE_DIRECTORIES "${_dav1d_inc}"
    )
    # dav1d on Windows pulls Bcrypt for thread-name + entropy
    target_link_libraries(dav1d INTERFACE Bcrypt)
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(DAV1D REQUIRED dav1d)
    add_library(dav1d INTERFACE)
    target_include_directories(dav1d INTERFACE ${DAV1D_INCLUDE_DIRS})
    target_link_libraries(dav1d INTERFACE ${DAV1D_LIBRARIES})
    target_link_directories(dav1d INTERFACE ${DAV1D_LIBRARY_DIRS})
endif()
