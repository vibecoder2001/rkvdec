add_executable(vp9_probs_test main.cpp)
target_link_libraries(vp9_probs_test PRIVATE rkmpp_mft)
add_test(
    NAME vp9_probs_test_kf
    COMMAND vp9_probs_test
        ${CMAKE_SOURCE_DIR}/../data/vp9/keyframe_only_720p.ivf
        ${CMAKE_SOURCE_DIR}/../data/vp9/bsp_capture/probe_kf_000.bin)
add_test(
    NAME vp9_probs_test_inter
    COMMAND vp9_probs_test
        ${CMAKE_SOURCE_DIR}/../data/vp9/inter_2frames_720p.ivf
        ${CMAKE_SOURCE_DIR}/../data/vp9/bsp_capture/probe_inter_001.bin
        --frame 1)
