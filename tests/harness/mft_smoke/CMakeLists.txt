add_executable(mft_smoke mft_smoke.cpp)
target_include_directories(mft_smoke PRIVATE
    ${CMAKE_SOURCE_DIR}/../../mft/dll
)
target_link_libraries(mft_smoke PRIVATE
    mfplat
    mfuuid
    mf
    ole32
    uuid
)
# Make sure the DLL is built before the smoke test runs (and lives next
# to the EXE so LoadLibrary("rkmpp_decoder_mft.dll") finds it).
add_dependencies(mft_smoke rkmpp_decoder_mft)

add_custom_command(TARGET mft_smoke POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:rkmpp_decoder_mft>
        $<TARGET_FILE_DIR:mft_smoke>)

if(RKMPP_HARNESS_TESTS)
    add_test(NAME mft_smoke COMMAND mft_smoke)
endif()
