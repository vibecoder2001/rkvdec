add_executable(rkmpp_decode main.cpp decode_engine.cpp)
target_link_libraries(rkmpp_decode PRIVATE rkmpp_mft setupapi cfgmgr32)
target_include_directories(rkmpp_decode PRIVATE ${CMAKE_SOURCE_DIR}/../..)
