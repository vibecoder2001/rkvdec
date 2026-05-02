add_executable(parser_test_h265 parser_test_h265.cpp)
target_link_libraries(parser_test_h265 PRIVATE rkmpp_mft)
# Test data lives off-tree (Z:\drivers-arm\bsp_capture); skip ctest unless
# present.  Always build, so the parser code keeps compiling.
if(EXISTS "Z:/drivers-arm/bsp_capture/first_idr.h265")
    add_test(NAME parser_test_h265 COMMAND parser_test_h265)
endif()
