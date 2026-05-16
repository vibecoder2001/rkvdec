add_executable(vp9_bool_decoder_test main.cpp)
target_link_libraries(vp9_bool_decoder_test PRIVATE rkmpp_mft)
add_test(NAME vp9_bool_decoder_test
         COMMAND vp9_bool_decoder_test
                 ${CMAKE_SOURCE_DIR}/../data/vp9/keyframe_only_720p.ivf)
