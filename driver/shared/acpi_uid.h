/* driver/shared/acpi_uid.h — ACPI _UID query via IRP_MN_QUERY_ID. */
#pragma once
#include <ntddk.h>
#include <wdf.h>

/* Returns the numeric _UID for an ACPI device PDO.
 * Sends IRP_MN_QUERY_ID(BusQueryInstanceID) and parses the result as
 * base-16.  Returns 0 on any failure (caller treats as UID 0). */
UINT32 RkSharedQueryAcpiUid(_In_ PDEVICE_OBJECT Pdo);

/* Reads the RKCPxxxx HID (4 hex digits after the "ACPI\RK" prefix) and
 * the device _UID from the PDO behind Device.  Five drivers reimplement
 * this same parse identically (rkvdec, rkav1d, rkmpp_ccu, rkiommu_vdec,
 * rkiommu_av1d) — when one of them drifts (e.g. accepts only RKCP35*)
 * the divergence is invisible until a new SoC ships.  Consolidating
 * here removes that risk.
 *
 * Returns STATUS_SUCCESS with *Hid + *Uid populated on a match,
 * STATUS_INVALID_DEVICE_REQUEST when no RKCPxxxx HID is found. */
NTSTATUS RkSharedReadAcpiHidUid(_In_ WDFDEVICE Device,
                                _Out_ PUINT32 Hid,
                                _Out_ PUINT32 Uid);
