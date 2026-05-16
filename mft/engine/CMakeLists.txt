# mft/engine — decode engine library shared by the production MFT DLL
# (mft/dll/) and the rkmpp_decode test harness.
#
# Wraps parser_glue → dpb → packed_tables → regbuilder → rkmpp.sys IOCTL
# into a single-call DecodeEngine_* API.  Static lib so both consumers
# can link the same code and stay binary-compatible.

add_library(rkmpp_decode_engine STATIC
    decode_engine.cpp
    decode_engine_av1.cpp
    backend_windows.cpp)

target_link_libraries(rkmpp_decode_engine PUBLIC rkmpp_mft)
target_link_libraries(rkmpp_decode_engine PRIVATE setupapi cfgmgr32)

# PUBLIC includes so consumers can `#include "decode_engine.h"` directly.
target_include_directories(rkmpp_decode_engine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/shared)

target_compile_features(rkmpp_decode_engine PUBLIC cxx_std_17)
# Note: not setting /W4 /WX here.  decode_engine_av1.cpp uses fopen/getenv
# (a few diagnostic dump/env paths) which trip C4996 under /W4 /WX.  The
# rest of the codebase (rkmpp_mft, rkmpp_decoder_mft) does enforce /WX.
