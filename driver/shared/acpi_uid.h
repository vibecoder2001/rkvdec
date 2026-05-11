/* driver/shared/acpi_uid.h — ACPI _UID query via IRP_MN_QUERY_ID. */
#pragma once
#include <ntddk.h>

/* Returns the numeric _UID for an ACPI device PDO.
 * Sends IRP_MN_QUERY_ID(BusQueryInstanceID) and parses the result as
 * base-16.  Returns 0 on any failure (caller treats as UID 0). */
UINT32 RkSharedQueryAcpiUid(_In_ PDEVICE_OBJECT Pdo);
