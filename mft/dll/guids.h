/* mft/dll/guids.h — class GUIDs for the rkvdec MFT.
 *
 * Two distinct CLSIDs are defined: one for the H.264 decoder MFT, one
 * for the HEVC decoder MFT.  Both live in the same DLL; the class
 * factory dispatches on rclsid to set the codec selector.
 *
 * GUIDs were generated for the Rockchip RK3588 driver namespace.  They
 * are ABI-stable: changing them would orphan any previously-installed
 * registry entries.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once

#include <guiddef.h>

/* {7AB3D2C0-1E55-4E3A-8C7E-9F1A4D2B0001} */
DEFINE_GUID(CLSID_RkmppH264Decoder,
    0x7ab3d2c0, 0x1e55, 0x4e3a, 0x8c, 0x7e, 0x9f, 0x1a, 0x4d, 0x2b, 0x00, 0x01);

/* {7AB3D2C0-1E55-4E3A-8C7E-9F1A4D2B0002} */
DEFINE_GUID(CLSID_RkmppHevcDecoder,
    0x7ab3d2c0, 0x1e55, 0x4e3a, 0x8c, 0x7e, 0x9f, 0x1a, 0x4d, 0x2b, 0x00, 0x02);

/* {7AB3D2C0-1E55-4E3A-8C7E-9F1A4D2B0003} */
DEFINE_GUID(CLSID_RkmppAv1Decoder,
    0x7ab3d2c0, 0x1e55, 0x4e3a, 0x8c, 0x7e, 0x9f, 0x1a, 0x4d, 0x2b, 0x00, 0x03);
