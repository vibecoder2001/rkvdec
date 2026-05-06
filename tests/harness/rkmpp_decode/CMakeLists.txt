# decode_engine wraps the parser_glue → dpb → packed_tables → regbuilder
# pipeline + the rkmpp.sys IOCTL surface into a single-call API.  Built as
# a static lib so the rkmpp_decode CLI test AND the IMFTransform DLL
# (mft/dll/) can link the same code.
add_library(rkmpp_decode_engine STATIC
    decode_engine.cpp
    decode_engine_av1.cpp)
target_link_libraries(rkmpp_decode_engine PUBLIC rkmpp_mft dav1d)
target_link_libraries(rkmpp_decode_engine PRIVATE setupapi cfgmgr32)
target_include_directories(rkmpp_decode_engine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/../..
    ${CMAKE_SOURCE_DIR}/../../shared)

add_executable(rkmpp_decode main.cpp)
target_link_libraries(rkmpp_decode PRIVATE rkmpp_decode_engine setupapi cfgmgr32)
target_include_directories(rkmpp_decode PRIVATE ${CMAKE_SOURCE_DIR}/../..)
