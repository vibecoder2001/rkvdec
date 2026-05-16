add_executable(vp9_dpb_test main.cpp)
target_link_libraries(vp9_dpb_test PRIVATE rkmpp_mft)
add_test(NAME vp9_dpb_test COMMAND vp9_dpb_test)
