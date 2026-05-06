cmake_minimum_required(VERSION 3.20)

# av1_parse_smoke: links dav1d and feeds it a single AV1 OBU to confirm
# the submodule + cross-build wiring works end-to-end on ARM64 Windows.
add_executable(av1_parse_smoke av1_parse_smoke.cpp)
target_link_libraries(av1_parse_smoke PRIVATE dav1d)
