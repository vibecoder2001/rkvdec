add_executable(dpb_test dpb_test.cpp)
target_link_libraries(dpb_test PRIVATE rkmpp_mft)
add_test(NAME dpb_test COMMAND dpb_test)
