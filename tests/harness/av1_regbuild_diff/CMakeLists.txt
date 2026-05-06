# tests/harness/av1_regbuild_diff/ — IVF → dav1d → regbuilder_av1 →
# BSP-shim formatted reg dump.  Used to diff our reg array against
# captured BSP traces (tests/data/av1/bsp_720p.log) on either Linux
# or Windows host without touching hardware.

add_executable(av1_regbuild_diff av1_regbuild_diff.cpp)
target_link_libraries(av1_regbuild_diff PRIVATE rkmpp_mft dav1d)
