/*
 *
 *	xclibver.h	External	05-Nov-2009
 *
 *	Copyright (C)  1999-2009  EPIX, Inc.  All rights reserved.
 *
 *	PIXCI(R) Library: Names, Version ID, etc.
 *
 */

#if !defined(__EPIX_XCLIBVER_DEFINED)
#define __EPIX_XCLIBVER_DEFINED

/*
 * Library version ID's.
 * (Internal flag: SFUID!)
 *
 * #define XCLIB_IDNVR	   "PIXCI(R) 32 Bit Library V2.10.01 [01.10.04]"
 */
#if defined(C_GNU64)|defined(C_MSC64)
  #if defined(PIXCI_CLSER)
    #if PIXCI_CLSER==10
      #define XCLIB_IDN       "PIXCI(R) CameraLink API V1.0"
    #elif PIXCI_CLSER==11
      #define XCLIB_IDN       "PIXCI(R) CameraLink API V1.1"
    #else
      #define XCLIB_IDN       "PIXCI(R) CameraLink API V?.?"
    #endif
  #elif defined(PIXCI_LITE)
    #define XCLIB_IDN	    "PIXCI(R) 64 Bit Library Lite"
  #elif defined(PIXCI_VIRTUAL)
    #define XCLIB_IDN	    "PIXCI(R) 64 Bit Virtual Library"
  #else
    #define XCLIB_IDN	    "PIXCI(R) 64 Bit Library"
  #endif
#else
  #if defined(PIXCI_CLSER)
    #if PIXCI_CLSER==10
      #define XCLIB_IDN       "PIXCI(R) CameraLink API V1.0"
    #elif PIXCI_CLSER==11
      #define XCLIB_IDN       "PIXCI(R) CameraLink API V1.1"
    #else
      #define XCLIB_IDN       "PIXCI(R) CameraLink API V?.?"
    #endif
  #elif defined(PIXCI_LITE)
    #define XCLIB_IDN	    "PIXCI(R) 32 Bit Library Lite"
  #elif defined(PIXCI_VIRTUAL)
    #define XCLIB_IDN	    "PIXCI(R) 32 Bit Virtual Library"
  #else
    #define XCLIB_IDN	    "PIXCI(R) 32 Bit Library"
  #endif
#endif
/*
 * XCLIB_IDR is machine generated.
 */
#define XCLIB_IDV	"3.07.00"
#define XCLIB_IDV0	3
#define XCLIB_IDV1	07
#define XCLIB_IDV2	00
#define XCLIB_IDR       "[11.03.08]"
#define XCLIB_IDNVR	XCLIB_IDN " " XCLIB_IDV " " XCLIB_IDR


#endif				/* !defined(__EPIX_XCLIBVER_DEFINED) */
