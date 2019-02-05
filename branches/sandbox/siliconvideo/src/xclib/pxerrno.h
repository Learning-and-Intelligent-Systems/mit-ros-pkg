/*
 *
 *	pxerrno.h	External	10-Jun-2008
 *
 *	Copyright (C)  1989-2008  EPIX, Inc.  All rights reserved.
 *
 *	DVI All Driver: Error codes.
 *	Specific values may change, but they will always be negative!
 *
 */

#if !defined(__EPIX_PXERRNO_DEFINED)
#define __EPIX_PXERRNO_DEFINED

#ifdef  __cplusplus
extern "C" {
#endif


/*
 * Values for first four were defined in early documentation (circa 1988)
 * instead of mnemonics. Don't change.
 */
#define PXERROR 	    -1	    /* use error. bad or inconsistent args ...	*/
#define PXERMALLOC	    -2	    /* memory allocation error			*/
#define PXERLOGIC	    -5	    /* internal logic error			*/
#define PXERSEARCH	    -9	    /* specified name could not be matched	*/
#define PXERBADMNEMONIC     -9	    /* specified name could not be matched	*/

#define PXERHARD	    -3	    /* hardware device error			*/
#define PXERBREAK	    -4	    /* canceled by user (Ctrl+Break)		 */
#define PXERNOOPTION	    -6	    /* hardware option not installed/available	*/
#define PXERNOPTION	    -6	    /* PXERNOOPTION, historical (mis)spelling	*/
#define PXERDOSIO	    -7	    /* generic dos file read/write error	*/
#define PXERDRIVER	    -8	    /* undetermined driver error		*/
#define PXERFILEFORM	    -10     /* invalid file format			*/
#define PXERNOMODE	    -11     /* Option/Feature/Operation not (currently) supported */
#define PXERNOITEM	    -12     /* no such item/object			*/
#define PXERNOFILE	    -13     /* file not found				*/
#define PXERNEWFILE	    -14     /* file not creatable			*/
#define PXERTOOBIG	    -15     /* insufficient table space 		*/
#define PXERBADEXP	    -16     /* ill formed expression, constraint	*/
#define PXERMATHOP	    -17     /* invalid mathematical operation		*/
#define PXERBADPARM	    -18     /* invalid parameter or option specification*/
#define PXERNODATA	    -19     /* data type(s) not supported		*/
#define PXERNOTNOW	    -20     /* feature/operation not available in current environment */
#define PXERNOCOLOR	    -21     /* color(s) or data component(s) not supported */
#define PXERDEVIO	    -22     /* generic device (COM, PRT, I2C) access error   */
#define PXERISOPEN	    -23     /* item in use, driver already open 	*/
#define PXERINUSE	    -23     /* historical name, deprecated		*/
#define PXERTIMEOUT	    -24     /* operation timed out			*/
#define PXERNOTOPEN	    -25     /* driver/resource not opened for use	*/
#define PXERNODEVMEM	    -26     /* insufficent memory on device		*/
#define PXERNOFEATURE	    -27     /* feature is not installed 		*/
#define PXERFUNCSTUB	    -28     /* function is an incomplete stub		*/
#define PXERNOKEY	    -29     /* authorization key invalid or not found	*/
#define PXERTRIGGER	    -30     /* terminated by trigger			*/
#define PXERTOOSMALL	    -31     /* area too small				*/
#define PXERUNITS	    -32     /* not available for multiple units, or currently selected units */
#define PXERVIDFORM	    -33     /* invalid video format			    */
#define PXERCOMPRESS	    -34     /* compression mode not supported		    */
#define PXERMORESPACE	    -35     /* more space needed			    */
#define PXERTOOLATE	    -36     /* timed command arrived too late for execution */
#define PXERNORESOURCE	    -37     /* required resource not available		    */
#define PXERNOMODULE	    -38     /* required library/module not found	    */
#define PXERNOFUNCTION	    -39     /* required function in library/module not found*/
#define PXERBADSTRUCT	    -40     /* wrong (version of) parameter		    */
#define PXERBADCHAIN	    -41     /* internal error, bad chain		    */
#define PXERRESOURCEBUSY    -42     /* video resource already in use			  */
#define PXERDEVFAULT	    -43     /* unknown device fault			    */
#define PXEROPSYS	    -44     /* undetermined operating system error	    */
#define PXERBUFFERSPACE     -45     /* insufficient frame buffer memory 	    */
#define PXERSTATECHANGED    -46     /* state changed during operation		    */
#define PXERWRONGHARDWARE   -47     /* parameter(s) invalid for selected hardware   */
#define PXERX11NOMODE	    -48     /* XWindows/X11 mode not supported		    */
#define PXERX11ERROR	    -49     /* XWindows/X11 error			    */
#define PXERNOFEATSOFTVERS  -50     /* feature not available in current version of software */
#define PXERUNKNOWNERROR    -51     /* error code is undefined			    */
#define PXERBADDIM	    -52     /* invalid image dimensions 		    */
#define PXERNOVIDSTATE	    -53     /* video state not found/defined		    */
#define PXERDRIVERBADVERS   -54     /* driver is wrong version			    */
#define PXERDRIVERNOTFOUND  -55     /* driver not installed or accessible	    */
#define PXERNOOPERSYS	    -56     /* current operating system not supported	    */
#define PXEROPENFILE	    -57     /* file not found or creatable		    */
#define PXERTHUNK3264	    -58     /* can't convert value/parameter, or can't perform operation, in 32 bit app on 64 bit O.S. */
#define PXERNOCAMERAID	    -59     /* can't identify camera model                        */
#define PXERRARG1	    -1	    /* currently, same as PXERROR		    */
#define PXERRARG2	    -1	    /* ditto					    */
#define PXERRARG3	    -1	    /* ditto					    */
#define PXERRARG4	    -1	    /* ditto					    */
#define PXERRARG5	    -1	    /* ditto					    */
#define PXERRARG6	    -1	    /* ditto					    */
#define PXERRARG7	    -1	    /* ditto					    */
#define PXERRARG8	    -1	    /* ditto					    */
#define PXERRARG9	    -1	    /* ditto					    */
#define PXERRARG10	    -1	    /* ditto					    */
#define PXERRARG11	    -1	    /* ditto					    */
#define PXERRARG12	    -1	    /* ditto					    */
#define PXERRARG13	    -1	    /* ditto					    */
#define PXERRARG14	    -1	    /* ditto					    */
#define PXERRARG15	    -1	    /* ditto					    */
#define PXERRARG16	    -1	    /* ditto					    */
#define PXERRARG17	    -1	    /* ditto					    */
#define PXERRARG18	    -1	    /* ditto					    */
#define PXERRARG19	    -1	    /* ditto					    */
#define PXERUSER	    -8192   /* external additions start here		    */
#define PXERONDSP	    -16384  /* modifier: error from DSP, not host	    */

/*
 * pxerrno.c
 */
_cDcl(_dllpxobj _dllpxipl,_cfunfcc,char*) pxerrnomesg(int err);


#ifdef  __cplusplus
}
#endif

#endif				/* !defined(__EPIX_PXERRNO_DEFINED) */
