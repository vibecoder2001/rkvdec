cmake_minimum_required(VERSION 3.20)

# av1_parse_smoke: links dav1d and feeds it a single AV1 OBU to confirm
# the submodule + cross-build wiring works end-to-end on ARM64 Windows.
add_executable(av1_parse_smoke av1_parse_smoke.cpp)
target_link_libraries(av1_parse_smoke PRIVATE dav1d)

# av1_parse_clean_smoke: same smoke test using the clean-room parser only;
# does NOT link dav1d (only headers.h is used), so this binary has no LGPL
# dependency.  Diff its stdout against av1_parse_smoke to cross-check.
add_executable(av1_parse_clean_smoke av1_parse_clean_smoke.cpp)
target_link_libraries(av1_parse_clean_smoke PRIVATE rkmpp_mft)
# rkmpp_mft exposes dav1d headers transitively via its PUBLIC include dirs;
# but we must NOT link the dav1d shared lib here — only the headers are used.
target_compile_definitions(av1_parse_clean_smoke PRIVATE
    DAV1D_HEADERS_ONLY=1)
