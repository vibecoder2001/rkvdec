# Host-only target — links libavcodec from the BtbN.FFmpeg.LGPL.Shared
# winget install for cross-validation against our parser/DPB.
set(_FFMPEG_DEV "C:/Users/vibecoder/AppData/Local/Microsoft/WinGet/Packages/BtbN.FFmpeg.LGPL.Shared.7.1_Microsoft.Winget.Source_8wekyb3d8bbwe/ffmpeg-n7.1.3-46-g957b06a788-win64-lgpl-shared-7.1")

if(NOT EXISTS "${_FFMPEG_DEV}/include/libavcodec/avcodec.h")
    message(STATUS "parser_lavc_diff: ffmpeg dev libs not found at ${_FFMPEG_DEV}, skipping target")
    return()
endif()

if(NOT (CMAKE_VS_PLATFORM_NAME STREQUAL "x64" OR CMAKE_VS_PLATFORM_NAME STREQUAL "Win32"))
    message(STATUS "parser_lavc_diff: host-only (x64/Win32), skipping for ${CMAKE_VS_PLATFORM_NAME}")
    return()
endif()

add_executable(parser_lavc_diff parser_lavc_diff.cpp)
target_include_directories(parser_lavc_diff PRIVATE "${_FFMPEG_DEV}/include")
target_link_libraries(parser_lavc_diff PRIVATE
    rkmpp_mft
    "${_FFMPEG_DEV}/lib/avcodec.lib"
    "${_FFMPEG_DEV}/lib/avutil.lib"
)

# Copy the avcodec/avutil DLLs next to the exe so it can run.
add_custom_command(TARGET parser_lavc_diff POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${_FFMPEG_DEV}/bin/avcodec-61.dll" "$<TARGET_FILE_DIR:parser_lavc_diff>/"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${_FFMPEG_DEV}/bin/avutil-59.dll"  "$<TARGET_FILE_DIR:parser_lavc_diff>/"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${_FFMPEG_DEV}/bin/swresample-5.dll"  "$<TARGET_FILE_DIR:parser_lavc_diff>/"
)
