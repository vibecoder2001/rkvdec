add_executable(parser_test parser_test.cpp)
target_link_libraries(parser_test PRIVATE rkmpp_mft)
add_test(NAME parser_test COMMAND parser_test)
