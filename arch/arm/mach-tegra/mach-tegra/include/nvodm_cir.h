/*
 * PEGATRON Corp. Confidential and Proprietary
 *
 * The following software source code ("Software") is strictly confidential and
 * is proprietary to PEGATRON Corp. ("PEGATRON").  It may only be read,
 * used, copied, adapted, modified or otherwise dealt with by you if you have
 * entered into a confidentiality agreement with PEGATRON and then subject to the
 * terms of that confidentiality agreement and any other applicable agreement
 * between you and PEGATRON.  If you are in any doubt as to whether you are
 * entitled to access, read, use, copy, adapt, modify or otherwise deal with
 * the Software or whether you are entitled to disclose the Software to any
 * other person you should contact PEGATRON.  If you have not entered into a
 * confidentiality agreement with PEGATRON granting access to this Software you
 * should forthwith return all media, copies and printed listings containing
 * the Software to PEGATRON.
 *
 * PEGATRON reserves the right to take legal action against you should you breach
 * the above provisions.
 */

/** 
 * @file
 * <b>CIR Tegra ODM Kit:
 *         CIR Interface</b>
 *
 * @b Description: Defines the interface for the ODM CIR.
 * 
 */

#ifndef INCLUDED_NVODM_CIR_H
#define INCLUDED_NVODM_CIR_H

#include "nvodm_services.h"

/**
 * @defgroup nvodm_CIR ODM CIR Interface
 *
 * This is the interface for the ODM CIR. See also the
 * \link nvodm_cir CIR Controller Adaptation Interface\endlink and
 * the \link nvodm_query_kbc ODM Query KBC Interface\endlink.
 * @ingroup nvodm_adaptation
 * @{
 */

/**
 * Initializes the ODM CIR.
 * 
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */
NvBool NvOdmCirInit(void); /*jolen 0701*/

/**
 * Releases the ODM CIR resources that were acquired during the 
 * call to NvOdmCIRInit().
 */
void NvOdmCirDeInit(void);

/// Defines the scan code break flag.
//#define NV_ODM_CIR_SCAN_CODE_FLAG_BREAK (0x1)

/// Defines the scan code make flag.
//#define NV_ODM_CIR_SCAN_CODE_FLAG_MAKE  (0x2)

//define IR repeat!!
#define NV_ODM_CIR_SCAN_CODE_REPET 0x1
#define NV_ODM_CIR_SCAN_CODE_STOP 0x0

/** 
 * Gets the cir data from the ODM CIR. This function must be called in
 * an infinite loop to continue receiving the key scan codes.
 *
 * @param pCirScanCode A pointer to the returned scan code of the key.
 * @param pScanCodeFlags A pointer to the returned value specifying scan code
 *  make/break flags (may be ORed for special code that combines make and
 *  break sequences).
 * @param Timeout (Optional) Specifies the timeout in msec. Can be set
 *  to zero if no timeout needs to be used.
 * 
 * @return NV_TRUE if successful, or NV_FALSE otherwise.
 */ 
NvBool NvOdmCirGetKeyData(NvU8 showlog, NvU32 *pKeyScanCode, NvU8 *pScanCodeFlags, NvU32 Timeout);

#endif // INCLUDED_NVODM_CIR_H
