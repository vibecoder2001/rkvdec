cmake_minimum_required(VERSION 3.20)

# av1_parse_smoke: links real dav1d and feeds it a single AV1 OBU as a
# cross-check oracle for av1_parse_clean_smoke (our clean-room parser).
# Requires the dav1d static archive — only built when
# third_party/build_dav1d.bat has been run (or libdav1d-dev is installed
# on Linux).  The default `dav1d` target is header-only and isn't enough
# here.  Skipped silently if the lib isn't available.
if(TARGET dav1d_lib)
    add_executable(av1_parse_smoke av1_parse_smoke.cpp)
    target_link_libraries(av1_parse_smoke PRIVATE dav1d_lib)
else()
    message(STATUS "Skipping av1_parse_smoke: dav1d_lib not available "
                   "(run third_party/build_dav1d.bat to enable)")
endif()

# av1_parse_clean_smoke: same smoke test using the clean-room parser only;
# does NOT link dav1d (only headers.h is used), so this binary has no LGPL
# dependency.  Diff its stdout against av1_parse_smoke to cross-check.
add_executable(av1_parse_clean_smoke av1_parse_clean_smoke.cpp)
target_link_libraries(av1_parse_clean_smoke PRIVATE rkmpp_mft)
# rkmpp_mft exposes dav1d headers transitively via its PUBLIC include dirs;
# but we must NOT link the dav1d shared lib here — only the headers are used.
target_compile_definitions(av1_parse_clean_smoke PRIVATE
    DAV1D_HEADERS_ONLY=1)
