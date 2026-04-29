# The deployable tool targets the RK3588 (ARM64); see top-level CMakeLists for
# the host-vs-target split. The unit-test binary is gated on RKMPP_HARNESS_TESTS
# so cross-compile builds skip it (the test exe wouldn't run on the host).
#
# devioctl.h note: shared/rkmpp_ioctl.h includes <devioctl.h>, which lives in
# the WDK "shared" include directory (not km-only) and resolves correctly from
# user-mode code as long as <windows.h> is included before rkmpp_ioctl.h.

add_executable(rkmpp_caps_dump main.cpp)
target_include_directories(rkmpp_caps_dump PRIVATE ${CMAKE_SOURCE_DIR}/../../shared)
target_link_libraries(rkmpp_caps_dump PRIVATE setupapi cfgmgr32)

if(RKMPP_HARNESS_TESTS)
    add_executable(rkmpp_caps_dump_test test_main.cpp main.cpp)
    target_compile_definitions(rkmpp_caps_dump_test PRIVATE RKMPP_CAPS_DUMP_TEST)
    target_include_directories(rkmpp_caps_dump_test PRIVATE ${CMAKE_SOURCE_DIR}/../../shared)
    target_link_libraries(rkmpp_caps_dump_test PRIVATE setupapi cfgmgr32)
    add_test(NAME rkmpp_caps_dump_test COMMAND rkmpp_caps_dump_test)
endif()
