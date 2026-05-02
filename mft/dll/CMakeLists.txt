# mft/dll/CMakeLists.txt — Windows MFT DLL wrapping rkmpp_mft.
#
# Produces rkmpp_decoder_mft.dll. The four DllXxx functions are exported
# via decoder_mft.def + __declspec(dllexport) (belt-and-suspenders, since
# the C++ name decoration on extern "C" __stdcall on x64/ARM64 is the
# bare name anyway).

add_library(rkmpp_decoder_mft SHARED
    dllmain.cpp
    class_factory.cpp
    decoder_mft.cpp
    registration.cpp
    decoder_mft.def
)

target_include_directories(rkmpp_decoder_mft PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_compile_features(rkmpp_decoder_mft PRIVATE cxx_std_17)

target_link_libraries(rkmpp_decoder_mft PRIVATE
    rkmpp_mft
    rkmpp_decode_engine
    mfplat
    mfuuid
    mf
    d3d11
    dxgi
    ole32
    uuid
    advapi32
    setupapi
    cfgmgr32
)

if(MSVC)
    target_compile_options(rkmpp_decoder_mft PRIVATE /W4 /WX)
    # COM export functions need __stdcall calling convention; nothing
    # extra needed on x64/ARM64 (single calling convention).
endif()

# Make sure the .def file is consumed by the linker.
set_target_properties(rkmpp_decoder_mft PROPERTIES
    PREFIX ""
    OUTPUT_NAME "rkmpp_decoder_mft"
)
