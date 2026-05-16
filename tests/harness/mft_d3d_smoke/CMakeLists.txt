add_executable(mft_d3d_smoke mft_d3d_smoke.cpp)
target_include_directories(mft_d3d_smoke PRIVATE
    ${CMAKE_SOURCE_DIR}/../../mft/dll
)
target_link_libraries(mft_d3d_smoke PRIVATE
    rkmpp_mft   # for au_iter (H264AuNext / H265AuNext)
    mfplat
    mfuuid
    mf
    evr
    strmiids
    d3d11
    dxgi
    ole32
    uuid
)
add_dependencies(mft_d3d_smoke rkmpp_decoder_mft)

add_custom_command(TARGET mft_d3d_smoke POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:rkmpp_decoder_mft>
        $<TARGET_FILE_DIR:mft_d3d_smoke>)
