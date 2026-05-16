add_executable(mft_play mft_play.cpp)
target_include_directories(mft_play PRIVATE
    ${CMAKE_SOURCE_DIR}/../../mft/dll
    ${CMAKE_SOURCE_DIR}/../../mft
)
target_link_libraries(mft_play PRIVATE
    rkmpp_mft
    mfplat
    mfuuid
    mf
    mfreadwrite
    evr
    strmiids
    shlwapi
    ole32
    uuid
    user32
    gdi32
)
add_dependencies(mft_play rkmpp_decoder_mft)

# Stage the DLL alongside the EXE.
add_custom_command(TARGET mft_play POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:rkmpp_decoder_mft>
        $<TARGET_FILE_DIR:mft_play>)
