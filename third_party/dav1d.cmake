# third_party/dav1d.cmake — expose dav1d headers to CMake consumers.
#
# We use dav1d as a header-only dependency.  mft/av1_parser.{cpp,h}
# produces Dav1dSequenceHeader / Dav1dFrameHeader / Dav1d* enum types
# verbatim so the engine code can stay drop-in with anything that
# expects dav1d structs, but the parser is a clean-room implementation
# that never calls into the dav1d runtime.  No libdav1d.a / libdav1d.so
# link is required.
#
# Usage from a CMakeLists.txt:
#     include(${CMAKE_SOURCE_DIR}/../../third_party/dav1d.cmake)
#     target_link_libraries(<my_target> PRIVATE dav1d)
#
# Windows: headers come from the submodule's src tree (the static
# headers checked into the repo + the generated `version.h` /
# `vcs_version.h` that meson emits during build).  If the submodule
# is initialised but build/ is empty, we synthesise the generated
# headers ourselves with placeholder values — they aren't load-bearing
# for our parser, only for #ifdef gates inside Dav1d* type definitions.
#
# Linux: use system libdav1d-dev headers via pkg-config (BSP ffmpeg
# already pulls it in).

if(WIN32)
    set(_dav1d_root "${CMAKE_CURRENT_LIST_DIR}/dav1d")

    if(NOT EXISTS "${_dav1d_root}/include/dav1d/dav1d.h")
        message(FATAL_ERROR
            "dav1d submodule not initialised at ${_dav1d_root}.\n"
            "Run: git submodule update --init third_party/dav1d")
    endif()

    # Pick the per-target build dir for the static lib (only used by
    # `dav1d_lib` below — `dav1d` itself is header-only).
    if(CMAKE_VS_PLATFORM_NAME STREQUAL "ARM64"
       OR CMAKE_SYSTEM_PROCESSOR MATCHES "^(ARM64|aarch64)$")
        set(_dav1d_build_dir "build")
        set(_dav1d_helper    "build_dav1d.bat")
    else()
        set(_dav1d_build_dir "build-x64")
        set(_dav1d_helper    "build_dav1d_x64.bat")
    endif()

    # Pick a per-target generated-headers dir so x64 and ARM64 can be
    # configured side-by-side without sharing state.
    if(CMAKE_VS_PLATFORM_NAME STREQUAL "ARM64"
       OR CMAKE_SYSTEM_PROCESSOR MATCHES "^(ARM64|aarch64)$")
        set(_dav1d_gen_dir "${CMAKE_BINARY_DIR}/dav1d_gen_arm64")
    else()
        set(_dav1d_gen_dir "${CMAKE_BINARY_DIR}/dav1d_gen_x64")
    endif()

    # version.h + vcs_version.h are generated at meson configure time.
    # For our header-only use they only need to exist — the macros they
    # define (DAV1D_API_VERSION_MAJOR/MINOR/PATCH, DAV1D_VERSION) gate
    # nothing we touch.  Synthesise minimal stand-ins.
    file(MAKE_DIRECTORY "${_dav1d_gen_dir}/dav1d")
    file(MAKE_DIRECTORY "${_dav1d_gen_dir}/vcs_version")
    if(NOT EXISTS "${_dav1d_gen_dir}/dav1d/version.h")
        file(WRITE "${_dav1d_gen_dir}/dav1d/version.h"
"#ifndef DAV1D_VERSION_H\n#define DAV1D_VERSION_H\n"
"#define DAV1D_API_VERSION_MAJOR 7\n"
"#define DAV1D_API_VERSION_MINOR 0\n"
"#define DAV1D_API_VERSION_PATCH 0\n"
"#define DAV1D_VERSION \"header-only-stub\"\n"
"#endif\n")
    endif()
    if(NOT EXISTS "${_dav1d_gen_dir}/vcs_version/vcs_version.h")
        file(WRITE "${_dav1d_gen_dir}/vcs_version/vcs_version.h"
"#define DAV1D_VERSION \"header-only-stub\"\n")
    endif()

    add_library(dav1d INTERFACE)
    target_include_directories(dav1d INTERFACE
        "${_dav1d_root}/include"
        "${_dav1d_gen_dir}")

    # Optional: dav1d_lib for consumers that DO call real dav1d
    # functions (e.g. av1_parse_smoke, which feeds a stream to real
    # dav1d and prints its parse for cross-check vs our clean-room
    # parser).  Skipped unless build_dav1d.bat has produced the .a.
    set(_dav1d_lib "${_dav1d_root}/${_dav1d_build_dir}/src/libdav1d.a")
    if(EXISTS "${_dav1d_lib}")
        add_library(dav1d_lib STATIC IMPORTED GLOBAL)
        set_target_properties(dav1d_lib PROPERTIES
            IMPORTED_LOCATION             "${_dav1d_lib}"
            INTERFACE_INCLUDE_DIRECTORIES "${_dav1d_root}/include;${_dav1d_gen_dir}"
        )
        # dav1d on Windows pulls Bcrypt for thread-name + entropy
        target_link_libraries(dav1d_lib INTERFACE Bcrypt)
    endif()
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(DAV1D REQUIRED dav1d)
    add_library(dav1d INTERFACE)
    target_include_directories(dav1d INTERFACE ${DAV1D_INCLUDE_DIRS})
    # Header-only by default; dav1d_lib is the real-link target for
    # consumers that call dav1d_* functions (av1_parse_smoke etc.).
    add_library(dav1d_lib INTERFACE)
    target_include_directories(dav1d_lib INTERFACE ${DAV1D_INCLUDE_DIRS})
    target_link_libraries(dav1d_lib INTERFACE ${DAV1D_LIBRARIES})
    target_link_directories(dav1d_lib INTERFACE ${DAV1D_LIBRARY_DIRS})
endif()
