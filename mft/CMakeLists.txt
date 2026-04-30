# mft/ — user-mode helpers for the rkvdec MFT (Phase 3b: parser_glue + future
# regbuilder/dpb/buffer).  Builds as a static library so harness binaries and
# the eventual MFT DLL can link the same code.

add_library(rkmpp_mft STATIC
    parser_glue.cpp
    regbuilder_h264.cpp
    h264_packed_tables.cpp
    dpb.cpp
)

target_include_directories(rkmpp_mft PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/v4l2-h264-controls
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared
)

target_compile_features(rkmpp_mft PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(rkmpp_mft PRIVATE /W4 /WX)
endif()
