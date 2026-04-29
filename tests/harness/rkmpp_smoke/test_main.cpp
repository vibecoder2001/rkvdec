/* tests/harness/rkmpp_smoke/test_main.cpp — TDD the no-op job builder. */
#include <windows.h>
#include <initguid.h>
#include <cstdio>
#include "../../../shared/rkmpp_ioctl.h"

void BuildNoopJob(UINT64 scratchHandle, RKMPP_SUBMIT_JOB_IN *out);

static int Fail(const char *m) { std::fprintf(stderr, "FAIL: %s\n", m); return 1; }

int main()
{
    RKMPP_SUBMIT_JOB_IN job{};
    BuildNoopJob(0xCAFEBABEDEADBEEFull, &job);

    if (job.StructSize != sizeof(job)) return Fail("StructSize wrong");
    if (job.RegWriteCount == 0)        return Fail("expected >= 1 reg write");
    if (job.BufRefCount   != 1)        return Fail("expected 1 buf ref");
    if (job.BufRefs[0].BufferHandle != 0xCAFEBABEDEADBEEFull)
        return Fail("buf ref handle wrong");
    if (job.TimeoutMs == 0)            return Fail("timeout zero");
    return 0;
}
