# rkmpp_decode — CLI test harness around DecodeEngine.  The engine
# library itself lives in mft/engine/ and is consumed via target name
# `rkmpp_decode_engine` (defined there).

add_executable(rkmpp_decode main.cpp)
target_link_libraries(rkmpp_decode PRIVATE rkmpp_decode_engine setupapi cfgmgr32)
target_include_directories(rkmpp_decode PRIVATE ${CMAKE_SOURCE_DIR}/../..)
