add_executable(mft_decode mft_decode.cpp)
target_include_directories(mft_decode PRIVATE
    ${CMAKE_SOURCE_DIR}/../../mft/dll
)
target_link_libraries(mft_decode PRIVATE
    rkmpp_mft   # for au_iter (H264AuNext / H265AuNext)
    mfplat
    mfuuid
    mf
    ole32
    uuid
)
add_dependencies(mft_decode rkmpp_decoder_mft)

# Stage the DLL next to the EXE so LoadLibrary finds it without
# regsvr32 / registry plumbing.
add_custom_command(TARGET mft_decode POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:rkmpp_decoder_mft>
        $<TARGET_FILE_DIR:mft_decode>)
