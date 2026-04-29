# rkvdec — RK3588 hardware video decode for Windows

Hardware-accelerated H.264 decode on RK3588 / Windows ARM64, exposed as a
Media Foundation Transform (MFT). Phase 1 ships only the kernel driver
skeleton (rkmpp.sys, rkmpp_ccu.sys, rkiommu.sys) and a `rkmpp_caps_dump`
tool. See `docs/superpowers/specs/` for the design and
`docs/superpowers/plans/` for phase plans.

## Build (drivers)

Open `driver/<name>/<name>.vcxproj` in Visual Studio with the WDK
extension installed; target Configuration=Debug, Platform=ARM64.

## Build (test harness)

The harness has two roles. The deployable tools (`rkmpp_caps_dump`,
`rkmpp_smoke`) target the RK3588 device and must be built ARM64. The TDD
unit tests (`*_test`) need to actually execute, so they're built for the
dev host's architecture and skipped when cross-compiling.

```bat
:: Deployable tools, ARM64 (run on the RK3588 target).
cmake -S tests/harness -B build/harness-arm64 -A ARM64 -DRKMPP_HARNESS_TESTS=OFF
cmake --build build/harness-arm64 --config Debug

:: Host-runnable unit tests. -A defaults to the host arch.
cmake -S tests/harness -B build/harness-host
cmake --build build/harness-host --config Debug
ctest --test-dir build/harness-host -C Debug -V
```

## Deploy to a target RK3588

```
scripts/deploy.ps1 -Target <ip-or-hostname>
```
