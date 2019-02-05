/*
 *
 *	cext_hpe.h	External	31-Jul-2002
 *
 *	Copyright (C)  1991-2002  EPIX, Inc.  All rights reserved.
 *
 *	GP: Standard .h suffix
 *	As required w. external visibility; for distributed .h's
 *
 */


/*
 * Restore structure packing
 */
#if defined(C_MSC32)|defined(C_MSC16)|defined(C_MSC64)
  #pragma pack()
  #pragma warning (disable:4103)    /* packing changes */
  /*
   * The following would be better, but pack warning messages
   * are still generated?
   *
   *   #pragma pack()
   *   #pragma warning (default:4103)	     // packing changes
   */
#elif defined(C_BOR32)|defined(C_BOR16)
  #pragma option -a.		   /* no pop available, restore to command line!! */
#elif defined(C_HIC32)
  #pragma Pop_align_members;
#elif defined(C_WAT32)|defined(C_WAT16)
  #pragma pack()
#elif defined(C_TMSC40)
#elif defined(C_GNU32)|defined(C_GNU64)
#else
  #error "Can't identify compiler."
#endif
/*
 * Check proper usage
 */
#if EPIX_CEXT_HPE!=1
  #error "Bad usage of cext_hp[1248se].h."
#endif
#undef EPIX_CEXT_HPE
