#ifndef RKMPP_VERSION_H
#define RKMPP_VERSION_H

/*
 * Single source of truth for the driver-suite version.
 *
 * Consumed by version.rc (binary VERSIONINFO resource) and pinned into the
 * INF DriverVer directive via each project's stampinf metadata
 * (SpecifyDriverVerDirectiveVersion / TimeStamp).  Keep these two in sync:
 * bumping this header is not enough on its own — the vcxproj TimeStamp values
 * must be updated to match (they cannot reference this header).
 *
 * Pre-v1.0 release versioning.
 */

#define RKMPP_VERSION_MAJOR 0
#define RKMPP_VERSION_MINOR 8
#define RKMPP_VERSION_PATCH 0
#define RKMPP_VERSION_BUILD 0

/* Comma form for FILEVERSION / PRODUCTVERSION resource fields. */
#define RKMPP_VERSION_COMMA 0,8,0,0

/* String form for StringFileInfo and human-facing display. */
#define RKMPP_VERSION_STR "0.8.0.0"

#endif /* RKMPP_VERSION_H */
