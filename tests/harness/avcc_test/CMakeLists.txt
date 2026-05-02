add_executable(avcc_test avcc_test.cpp)
target_link_libraries(avcc_test PRIVATE rkmpp_mft)
add_test(NAME avcc_test COMMAND avcc_test)
