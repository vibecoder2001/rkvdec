# mft/ — user-mode helpers for the rkvdec MFT (Phase 3b: parser_glue + future
# regbuilder/dpb/buffer).  Builds as a static library so harness binaries and
# the eventual MFT DLL can link the same code.

add_library(rkmpp_mft STATIC
    au_iter.cpp
    parser_glue.cpp
    parser_glue_h265.cpp
    avcc_to_annexb.cpp
    regbuilder_h264.cpp
    regbuilder_h265.cpp
    regbuilder_av1.cpp
    av1_parser.cpp
    h264_packed_tables.cpp
    h265_packed_tables.cpp
    dpb.cpp
)

target_include_directories(rkmpp_mft PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/v4l2-h264-controls
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared
)

# regbuilder_av1.cpp consumes Dav1dSequenceHeader / Dav1dFrameHeader;
# expose dav1d's headers to consumers via PUBLIC link.  The dav1d
# target is imported by tests/harness/CMakeLists.txt before this
# subdirectory is added.
if(TARGET dav1d)
    target_link_libraries(rkmpp_mft PUBLIC dav1d)
endif()

target_compile_features(rkmpp_mft PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(rkmpp_mft PRIVATE /W4 /WX)
    # av1_parser.cpp performs intentional narrowing assignments from int
    # to uint8_t in tiling-parameter derivation (tile_log2 return → uint8_t
    # fields).  Suppress C4244 for that file only so the /WX flag on the
    # rest of the target is not defeated.
    set_source_files_properties(av1_parser.cpp PROPERTIES
        COMPILE_FLAGS "/wd4244 /wd4505")
endif()

# Engine library (decode_engine.* + decode_engine_av1.*) — wraps the
# parser/dpb/regbuilder pipeline with the rkmpp.sys IOCTL surface.
# Shared between the production MFT DLL (mft/dll/) and the rkmpp_decode
# test harness (tests/harness/rkmpp_decode/).
add_subdirectory(engine)
