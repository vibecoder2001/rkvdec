/* shared/rkmpp_peer_worker_ifc.h — In-kernel device interface exported
 * by RVD1 (rkvdec UID==1 instance) so RVD0 can dispatch jobs to it.
 *
 * Topology: there is exactly one peer worker on RK3588 (the RVD1
 * instance).  RVD0 queries this interface at PrepareHardware time
 * (or later via IoRegisterPlugPlayNotification) and uses it to kick
 * jobs cross-core.  Single-core fallback: query returns
 * STATUS_NOT_FOUND, RVD0 stays self-only.  See spec
 * docs/superpowers/specs/2026-05-18-rkvdec-multicore-dispatch-design.md
 * sections "Architecture" and "Start-order races". */
#pragma once

#include <wdm.h>

DEFINE_GUID(GUID_DEVINTERFACE_RKMPP_PEER_WORKER,
    0x9a7e0c81, 0x4f12, 0x4a3d, 0xb6, 0x88, 0x1c, 0x2f, 0x6e, 0x14, 0xa0, 0x55);

#define RKMPP_PEER_WORKER_IFC_VERSION 1u

/* Callback type — RVD1 invokes this from its DPC when a kicked job's
 * codec interrupt has been handled.  Runs at DISPATCH_LEVEL.  RVD0
 * uses CompletionCookie to look up which RKMPP_JOB this completion
 * matches in its queue. */
typedef VOID (*RKMPP_PEER_COMPLETION_CB)(
    _In_ PVOID    ConsumerContext,    /* RVD0's RKMPP_DEVICE pointer */
    _In_ UINT64   CompletionCookie,   /* opaque, supplied at KickJob */
    _In_ NTSTATUS JobStatus,
    _In_ UINT32   HardwareStatus);

/* RVD0 registers its completion callback once, at attach time. */
typedef NTSTATUS (*RKMPP_PEER_REGISTER_COMPLETION)(
    _In_ PVOID                    ProviderContext,
    _In_ PVOID                    ConsumerContext,
    _In_ RKMPP_PEER_COMPLETION_CB Callback);

/* RVD0 calls this to launch a job on RVD1.  RVD1 takes responsibility
 * for the dense bank + iova substitutions + MMIO writes + IRQ wait,
 * and invokes the registered completion callback when done.  Returns
 * synchronously after kicking (does not wait for completion).
 *
 * Bank is copied internally — caller's storage may be reused after
 * return.  IovaSlots are the post-substitution iovas in dense-bank
 * coordinate (slot_index, iova) pairs, already resolved against the
 * caller's File's buffer handles. */
typedef struct _RKMPP_PEER_KICK_PARAMS {
    const VOID                  *Bank;          /* RKMPP_DENSE_BANK */
    UINT32                       BankBytes;     /* sizeof(RKMPP_DENSE_BANK) */
    UINT32                       KickValue;     /* swreg0 value (TRIGGER) */
    const VOID                  *IovaSlots;     /* RKMPP_DENSE_IOVA_SLOT[] */
    UINT32                       IovaSlotCount;
    UINT64                       CompletionCookie;
} RKMPP_PEER_KICK_PARAMS;

typedef NTSTATUS (*RKMPP_PEER_KICK_JOB)(
    _In_ PVOID                          ProviderContext,
    _In_ const RKMPP_PEER_KICK_PARAMS  *Params);

/* Inverse notification: provider tells consumer it is about to go away
 * (PnP query-remove).  Consumer MUST stop using the provider's function
 * pointers, drain anything in flight on the peer core, and Dereference
 * + release the provider file-object before returning.  Returning from
 * the callback grants the provider permission to complete query-remove. */
typedef VOID (*RKMPP_PEER_QUERY_REMOVE_CB)(_In_ PVOID ConsumerContext);

typedef NTSTATUS (*RKMPP_PEER_REGISTER_QUERY_REMOVE)(
    _In_ PVOID                       ProviderContext,
    _In_ PVOID                       ConsumerContext,
    _In_ RKMPP_PEER_QUERY_REMOVE_CB  Callback);

typedef struct _RKMPP_PEER_WORKER_INTERFACE {
    INTERFACE                          Header;
    UINT32                             Hid;   /* always 0x3550 */
    UINT32                             Uid;   /* always 1 (RVD1) */
    RKMPP_PEER_REGISTER_COMPLETION     RegisterCompletion;
    RKMPP_PEER_KICK_JOB                KickJob;
    RKMPP_PEER_REGISTER_QUERY_REMOVE   RegisterQueryRemove;
} RKMPP_PEER_WORKER_INTERFACE, *PRKMPP_PEER_WORKER_INTERFACE;
