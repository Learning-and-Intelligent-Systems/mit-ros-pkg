/*
 *
 *	cext_hp1.h	External	31-Jul-2002
 *
 *	Copyright (C)  1991-2002  EPIX, Inc.  All rights reserved.
 *
 *	GP: Standard .h prefix
 *	As required w. external visibility; for distributed .h's
 *
 */

#include "cext.h"

/*
 * Affix structure packing
 */
#if defined(C_MSC32)|defined(C_MSC16)|defined(C_MSC64)
  #pragma warning (disable:4103)    /* packing changes */
  #pragma pack(1)
#elif defined(C_BOR32)
  #pragma option -a1
#elif defined(C_BOR16)
  #if C_BOR16==0 || C_BOR16>=400
    #pragma option -a1
  #else
    #pragma option -a.		    /* restore to command line!! */
  #endif
#elif defined(C_HIC32)
  #pragma Push_align_members(1);
#elif defined(C_WAT32)|defined(C_WAT16)
  #pragma pack(1)
#elif defined(C_TMSC40)
#elif defined(C_GNU32)|defined(C_GNU64)
#else
  #error "Can't identify compiler."
#endif
/*
 * Check proper usage
 */
#if defined(EPIX_CEXT_HPE)
  #error "Bad usage of cext_hp[1248se].h."
#endif
#define EPIX_CEXT_HPE	 1
