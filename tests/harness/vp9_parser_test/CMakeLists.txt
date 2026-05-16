add_executable(vp9_parser_test main.cpp)
target_link_libraries(vp9_parser_test PRIVATE rkmpp_mft)
add_test(NAME vp9_parser_test
    COMMAND vp9_parser_test
        "${CMAKE_SOURCE_DIR}/../../tests/data/vp9/keyframe_only_720p.ivf"
        "${CMAKE_SOURCE_DIR}/../../tests/data/vp9/inter_2frames_720p.ivf")
