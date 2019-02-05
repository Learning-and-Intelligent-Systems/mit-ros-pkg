/*
 *
 *	pximage.h	External	27-Jul-2007
 *
 *	Copyright (C)  1984-2007  EPIX, Inc.  All rights reserved.
 *
 *	DVI Image Access: Standard Image Definition & Access
 *
 */



#if !defined(__EPIX_PXIMAGE_DEFINED)
#define __EPIX_PXIMAGE_DEFINED
#include "cext_hps.h"     

#ifdef  __cplusplus
extern "C" {
#endif


/*
 * Types
 *
 * The pxim3size_t is intended for sequences in memory,
 * and may not be large enough for sequences stored in files.
 */
typedef int	pxcoord_t;	/* x, y, z image coordinates		*/
typedef uint	pximcnt_t;	/* image data transfer count		*/
#define PXCOORD_MAX	INT_MAX
#define PXIMCNT_MAX	UINT_MAX
#if defined(C_MSC64)|defined(C_GNU64)
typedef uint	pxim1size_t;	/* line  & ypitch size			*/
typedef uint	pxim2size_t;	/* image & zpitch size			*/
#define PXIM1SIZE_MAX	UINT_MAX
#define PXIM2SIZE_MAX	UINT_MAX
  #if defined(C_MSC64)
    typedef unsigned long long pxim3size_t; /* image sequence size	*/
    #define PXIM3SIZE_MAX   ULONGLONG_MAX
  #else
    typedef ulong   pxim3size_t;	    /* image sequence size	*/
    #define PXIM3SIZE_MAX   ULONG_MAX
  #endif
typedef uint	pximaeiou_t;	/* return type of aeiou service, below. */
typedef sint	pximaeios_t;	/* .. and signed version, internal use. */
#else
typedef uint	pxim1size_t;	/* line  & ypitch size			*/
typedef ulong	pxim2size_t;	/* image & zpitch size			*/
typedef ulong	pxim3size_t;	/* image sequence size			*/
#define PXIM1SIZE_MAX	UINT_MAX
#define PXIM2SIZE_MAX	ULONG_MAX
#define PXIM3SIZE_MAX	ULONG_MAX
typedef ulong	pximaeiou_t;	/* return type of aeiou service, below. */
typedef slong	pximaeios_t;	/* .. and signed version, internal use. */
#endif



/*
 * A pair of coordinates.
 * Not unsigned, allowing "if ((this.x - 40) < that.x)"
 */
struct pxy
{
    pxcoord_t	  x;
    pxcoord_t	  y;
};
typedef struct pxy pxy_s;
typedef struct pxy pxypoint_s;

/*
 * A rectangular area (of interest).
 */
struct	pxywindow
{
    struct  pxy     nw;     /* north west corner inclusive	    */
    struct  pxy     se;     /* south east corner, usually exclusive */
};
typedef struct pxywindow pxywindow_s;

/*
 * A triplet of coordinates.
 */
struct pxyz
{
    pxcoord_t	  x;
    pxcoord_t	  y;
    pxcoord_t	  z;
};
typedef struct pxyz pxyz_s;
typedef struct pxyz pxyzpoint_s;

/*
 * A cubic area (of interest).
 */
struct	pxyzwindow
{
    struct  pxyz    nw;     /* north west inner corner inclusive	  */
    struct  pxyz    se;     /* south east outer corner, usually exclusive */
};
typedef struct pxyzwindow pxyzwindow_s;


/*
 * Fundamental data types specifiers and modifiers.
 * Historically, the UCHAR,USHORT,UINT,... are the primary definitions,
 * matching 'C' types; the UINT8, UINT16 are secondary, for convenience.
 * Some data types may not be supported, such as PXDATULONG under
 * 64 bit Linux where a long is 64 bits.
 * Never 0!
 */
#define PXDATATYPE	0x00FF
#define PXDATUCHAR	0x01	    /* matches a C unsigned char	    */
#define PXDATUSHORT	0x02	    /* matches a C unsigned short	    */
#define PXDATUINT	0x04	    /* matches a C unsigned int 	    */
#define PXDATULONG	0x08	    /* matches a C unsigned long	    */
#define PXDATFLOAT	0x10	    /* matches a C float		    */
#define PXDATDOUBLE	0x20	    /* matches a C double		    */

#if defined(C_MSC16)|defined(C_BOR16)|defined(C_WAT16)
#define PXDATUINT8	0x01
#define PXDATUINT16	0x02
#define PXDATUINT32	0x08
#define PXDATFLT4	0x10
#define PXDATFLT8	0x20
#elif defined(C_MSC32)|defined(C_BOR32)|defined(C_WAT32)|defined(C_TMSC40)|defined(C_GNU32)
#define PXDATUINT8	0x01
#define PXDATUINT16	0x02
#define PXDATUINT32	0x04
#define PXDATFLT4	0x10
#define PXDATFLT8	0x20
#elif defined(C_MSC64)
#define PXDATUINT8	0x01
#define PXDATUINT16	0x02
#define PXDATUINT32	0x04
#define PXDATFLT4	0x10
#define PXDATFLT8	0x20
#elif defined(C_GNU64)
#define PXDATUINT8	0x01
#define PXDATUINT16	0x02
#define PXDATUINT32	0x04
#define PXDATFLT4	0x10
#define PXDATFLT8	0x20
#else
    ??
#endif

				/* for uints with few significant bits: */
#define PXDATMSB    0x0100	/* - significant bits left justified	*/
#define PXDATLSB    0x0200	/* - significant bits right justified	*/
				/* - .. otherwise default justification */
#define PXDATDIRTY  0x0400	/* - unused bits may/might be nonzero	*/

				/* for uints larger than 1 byte:	*/
#define PXDATENDIAN 0x0800	/* wrong endian relative to current host*/
				/* new addition - partially implemented */

/*
 * A list of pixels to be accessed.
 */
struct pximadrs
{
    union {
	struct	pxy  xy;	    /* x, y coordinates 		    */
	struct	pxyz xyz;	    /* x, y, z coordinates		    */

	#if defined(C_MSC64)
	    unsigned long long devadrs; /* internal usage ...		    */
	#else
	    ulong   devadrs;	    /* internal usage ...		    */
	#endif
	uint	devstate[3];	    /*	  ...				    */
	void	     *devptr0;	    /*	  ...				    */
	void _far16p *devptr1;	    /*	  ...				    */
	void _farphy *devptr2;	    /*	  ...				    */
    } adrs;
};
typedef struct pximadrs pximadrs_s;


/*
 * Pixel list (bxt) operations
 */
#define PXPIXBLT    0	    /* bxt: exchange pixels			    */
#define PXPIXBATN   1	    /* bxt: add to pixels			    */
#define PXPIXBRT    2	    /* bxt: read pixels 			    */
#define PXPIXBWT    3	    /* bxt: write pixels			    */
#define PXPIXBXT    4	    /* bxt: xor with pixels (uint pixels only)	    */
#define PXPIXBUTN   5	    /* bxt: subtract from pixels		    */


/*
 * Mode verbs.
 * Internal Code would prefer that PXREAD, PXRITE, PXR*SCAN, PXRMOD*
 * be: a) contiguous and low values (for efficiency of switch)
 * and, b) occupy only low byte so the PXDATATYPE can be combined
 * with same for switching.
 */
#define PXREAD	    0x0000  /* read from device/object to pgm memory	    */
#define PXRITE	    0x0001  /* write to device/object from pgm memory	    */
#define PXRXSCAN    0x0000  /* read/write 2-d image in row (x, y) order     */
			    /* if 3-d: x, y, z order			    */
#define PXRYSCAN    0x0002  /* read/write 2-d image in column (y, x) order  */
			    /* if 3-d: y, x, z order			    */
#define PXRZSCAN    0x0004  /* read/write 3-d image in z, x, y order	    */
#define PXRXYZSCAN  0x0006  /* the PXRXSCAN,PXRYSCAN,PXRZSCAN bits	    */
#define PXRMODSL    0x0008  /* internal use				    */
#define PXRMODSR    0x0010  /* internal use				    */
#define PXRMODAM    0x0020  /* internal use				    */
#define PXRMODSB    0x0040  /* internal use				    */
#define PXRMODXX    0x0078  /* the PXRMOD bits				    */

#define PXIWRAP     0x0040  /* r/w of image wraps around right->left or     */
			    /* bottom->top edge of following row or column  */
#define PXIBXTC     0x0080  /* modify pximadrs, as needed, for bxta()	    */
#define PXIMAPINC   0x0400  /* imap() may provide access to pixels which    */
			    /* are not adjacent (relative to pixel size)    */
#define PXIMAP1     0x0000  /* imap() provides access to 1-dim sequence     */
#define PXIMAP2     0x8000  /* imap() provides access to 2(3)-dim sequence  */
			    /* using different X, Y (Z) length&increments   */
			    /* ! future !				    */
#define PXIMAPNOTFAR 0x4000 /* imap() must provide a pointer actually near  */
#define PXIASYNC    0x0800  /* do operation asynchronously, if possible     */
#define PXIMAYMOD   0x1000  /* iorite(...) may modify data buffer	    */
#define PXIXYVALID  0x2000  /* assume x, y coordinates are valid	    */
#define PXSETREAD   0x8000  /* setup hint: read used	 if neither spec'ed */
#define PXSETRITE   0x4000  /* setup hint: write used	 both r/w assumed   */


/*
 * Imapset() return flags
 */
#define PXIMAPSET   0x0001  /* imapset() success, imap (may) be available   */
#define PXIMAPOKOK  0x0004  /* imap() always available, will never fail     */
#define PXIMAPALL   0x0008  /* entire image accessible from imap(,,0,0)     */
			    /* (for PXIMAP1, iff imdim==wind and PXIWRAP)   */
#define PXIMAPNOTFAR 0x4000 /* if imap() succeeds, pointer is always near   */

/*
 * Async wait verb & status
 */
#define PXAWAIT 0x0000	    /* return when done 		    */
#define PXASYNC 0x1000	    /* return immediate 		    */
#define PXABORT 0x2000	    /* abort, complete & return immediate   */
#define PXAALL	0x3000	    /* bit mask 			    */
#define PXODONE 1	    /* operation done, resource free	    */
#define PXOPRUN 0	    /* operation not done yet		    */

/*
 * Hints on pixel interpretation
 */
#define PXHINTNONE	0x0000	/* unknown				*/
#define PXHINTFRAME	0x0010	/* raw frame buffer data		*/

				/* With pixies=1 (typically):		*/
#define PXHINTGREY	0x0001	/* grey (gray) scale monochrome 	*/
#define PXHINTGRAY	0x0001	/* gray (grey) scale monochrome 	*/
#define PXHINTINDEX	0x0011	/* index into unspecified palette	*/
#define PXHINTBAYER	0x0021	/* Bayer pattern RGB	G B G B ...	*/
				/* (raw)		R G R G ...	*/
				/*			or variation	*/
#define PXHINTBAYER0	0x1021	/* Raw Bayer w. Red at upper left	*/
#define PXHINTBAYER1	0x2021	/* Raw Bayer w. Grn-Red at upper left	*/
#define PXHINTBAYER2	0x3021	/* Raw Bayer w. Grn-Blu at upper left	*/
#define PXHINTBAYER3	0x4021	/* Raw Bayer w. Blue	at upper left	*/

				/* With pixies=2 (typically):		*/
#define PXHINTCOMPLEX	0x0002	/* complex, probably of monochrome	*/
#define PXHINTCBYCRY	0x0012	/* Cb,Y,Cr,Y,Cb,Y,Cr,Y,... (YCrCb,UYVY) */
#define PXHINTYCBYCR	0x0022	/* Y,Cb,Y,Cr,Y,Cb,Y,Cr,... (YCrCb,YUY2) */
#define PXHINTCRYCBY	0x0032	/* Cr,Y,Cb,Y,Cr,Y,Cb,Y,... (YCrCb,VYUY) */
#define PXHINTYCRYCB	0x0042	/* Y,Cr,Y,Cb,Y,Cr,Y,Cb,... (YCrCb,YVYU) */

				/* With pixies=3 (typically):		*/
#define PXHINTBGR	0x0003	/* B,G,R,... (RGB)			*/
#define PXHINTYCRCB	0x0013	/* Y,Cr,Cb,Y,Cr,Cb,... (YCrCb)		*/
#define PXHINTBSH	0x0023	/* Brightness,Saturation,Hue, (HSB)	*/
#define PXHINTRGB	0x0043	/* R,G,B,... (RGB)			*/
#define PXHINTYIQ	0x0053	/* Y,I,Q (YIQ)				*/
#define PXHINTCMY	0x0063	/* C,M,Y,... (CMY)			*/

				/* With pixies=4 (typically):		*/
#define PXHINTBGRX	0x0004	/* B,G,R,Pad (RGB)			*/
#define PXHINTYCRCBX	0x0014	/* Y,Cr,Cb,Pad,Y,Cr,Cb,Pad  (YCrCb)	*/
#define PXHINTRGBX	0x0024	/* R,G,B,Pad (RGB)			*/
#define PXHINTBAYERX4	0x0034	/* Bayer pattern w. R Gr Gb B each pixel*/
#define PXHINTCMYK	0x0044	/* C,M,Y,K,... (CMYK)			*/

#define PXHINTPIXIES	0x000F	/* mask # of pixies per pixel for hint	*/

#define PXHINTUSER	0x7000	/* user-defined types: from PXHINTUSER	*/
#define PXHINTUSERN	0x0FFF	/* thru PXHINTUSER+PXHINTUSERN		*/

/*
 * Pixel units.
 * In most applications, the X and Y units are the same,
 * the Z units are different. But there will be exceptions.
 */
#define PXUNITUNKNOWN	    0
#define PXUNITRATIO	    ('r')
#define PXUNITINCH	    (('i'<<1)^'n')
#define PXUNITFOOT	    (('f'<<1)^'t')
#define PXUNITMETER	    ('m')
#define PXUNITMILLIMETER    (('m'<<1)^'m')
#define PXUNITCENTIMETER    (('c'<<1)^'m')
#define PXUNITSECOND	    (('s'<<1)^'e')


/*
 * Pixel and pixie data descriptor.
 */
struct pximagedata
{
    int     pixietype;	    /* the PXDATATYPE (no modifiers)		       */
    int     pixies;	    /* samples/colors/components per pixel: 1,2,3,...  */
    int     pixelhint;	    /* PXHINT* hints on pixel color interpretation     */

    union {
	/*
	 * For integer pixels ...
	 */
	struct {
	    int bitsused;	/* significant bits per pixie 8,9,10,11,12,...	   */
	    int bitsxsb;	/* PXDATMSB | PXDATLSB: significant bits @ lsb|msb */
				/*		  both: all bits used		   */
	    uint16 dirtyread;	/* PXDATDIRTY: unused read bits may not be zero    */
				/*	    0: unused read bits are zero	   */
	    uint16 dirtyrite;	/* PXDATDIRTY: unused written bits needn't be zero?*/
				/*	    0: unused written bits must    be zero?*/

	    uint16 endian;	/* PXDATENDIAN: has wrong byte endian	for current*/
				/*	     0: has correct byte endian     host   */
	    uint16 pad; 	/* maintain historical size - future remove?	   */
	} i;
	/*
	 * For real pixels ...
	 * To conserve space, these are float, not double.
	 * The minvalue is often assumed to be 0.0.
	 * The maxvalue is commonly 1.0.
	 * If mivalue==maxvalue, no bounds.
	 */
	struct {
	    float   minvalue;	/* black is ...     */
	    float   maxvalue;	/* white is ...     */
	} r;
    } u;			/* anonymous union not supported?   */
};
typedef struct pximagedata  pximagedata_s;


/*
 * Hints on image interpretation.
 * This doesn't impact how data from the image is
 * read/written/appears, but how it is interpreted.
 * Some features may assume widthunits==heightunits!
 */
struct pximagehints
{
    float   pixelwidth;     /* X axis width of 1 pixel. 0 if unknown	    */
    float   pixelheight;    /* Y axis height of 1 pixel. 0 if unknown	    */
    float   pixeldepth;     /* Z axis depth of 1 pixel. 0 if unknown	    */
    uchar   widthunits;     /* PXUNIT* real world units 		    */
    uchar   heightunits;    /* PXUNIT* real world units 		    */
    uchar   depthunits;     /* PXUNIT* real world units 		    */
    uchar   pad;
};
typedef struct pximagehints pximagehints_s;


/*
 * Derived facts and other image status
 */
struct pximagefacts
{			    /* derived from d.pixies, d.pixietype:	    */
    int     pixiesize;	    /* bytes per component value		    */
    int     pixelsize;	    /* bytes per pixel				    */


    #if 0		    /* for integer pixels, derived ...		    */
    uint32  xsbmask;	    /* mask of significant bits 		    */
    uint    xsbshift;	    /* shift count to make lsb/msb justified	    */
    uint    xsbisnot;	    /* opposite of d.bitsxsb, 0 if all bits used    */
    #endif

			    /* derived from pixies, pixietype:		    */
    uint    pixieuint: 1;   /* - pixel components are positive ints	    */
    uint    pixiereal: 1;   /* - pixel components are real		    */
};
typedef struct pximagefacts pximagefacts_s;

/*
 * Returned from imap()
 */
struct pximap
{
    int 	    valid;  /* imap()'s return value, clr'ed by imapr()     */
    void _farphy    *p;     /* pointer to image memory, NULL if no access   */

			    /* if imap() mode 1 (PXIMAP1) ..		    */
    size_t	    inc;    /* increment to next pixel, always in ..	    */
			    /* sizeof(char), regardless of pixie type	    */
    size_t	    len;    /* number of following pixels that can be	    */
			    /* accessed at increasing x,y,z by using the inc*/

			    /* if imap() mode 2 (PXIMAP2): (future)	    */
    size_t	    xinc;   /* increment to next pixel (X) in row	    */
    size_t	    yinc;   /* increment to next row (Y) in image	    */
    size_t	    zinc;   /* increment to next image (Z) in sequence (3-D)*/
			    /* all in sizeof(char), regardless of pixie type*/
    size_t	    xdim;   /* # of accessible pixels at increasing X	    */
    size_t	    ydim;   /* # of accessible pixels at increasing Y	    */
    size_t	    zdim;   /* # of accessible pixels at increasing Z	    */
};
typedef struct pximap pximap_s;


/*
 * An Image.
 * Logically a C++ virtual class; but defined in C for sake of compatibility.
 * Intended to provide an ACCESS method; not maintain state;
 * specifically does NOT provide constructor/destructors, as C doesn't
 * have built in support for invoking these.
 *
 * Although many other characteristics control the video appearance of
 * an image, the following represents the essentials characteristics of
 * an image as stored in image memory.
 *
 * The imdim contains the image dimensions, a full window. The imdim.se.y
 * refers to the number of sample lines in the y dimension. Thus, for full
 * screen interlace imdim.se.y==480 (RS-170).
 * The wind allows definition of a rectangular subwindow of the image.
 *
 * Historically, external documentation has allowed/suggested
 * copying pximage structures at will, directly viewing the imdim,
 * and directly changing the wind. Using xwind() to change the wind
 * may become a future requirement.
 * Visual Basic sample code accesses the wind elements, assuming that
 * the wind is the first MOS.
 * Regardless, after changing the wind or using xwind(), the
 * ioset()/bxts()/imapset() functions must be re-invoked for the
 * new wind to have proper effect.
 *
 * The imdim.se.[xyz]*f.pixelsize are assumed to be no larger
 * than a pxcoord_t.
 *
 *  Current size:   216 in 16 bit environments (M Model)
 *		    224 in 16 bit environments (L Model)
 *		    312 in 32 bit environments
 *		    ??? in 64 bit environments
 */
struct	pximage
{
    /*
     * Size
     */
    struct pxywindow wind;	/* subwindow within imdim		    */
    pxcoord_t	     pad2[2];	/* alignment vis-a-vis pximage3 	    */
    struct pxywindow imdim;	/* nw.x = nw.y = 0, se.x & se.y 	    */
				/* .. is dimension of image		    */
    pxcoord_t	     pad1[2];	/* alignment vis-a-vis pximage3 	    */

    /*
     * Private to device driver
     * Future: larger for 64 bit environments
     */
    char    devname[8]; 	/* unique(?) char name of device handler    */
    struct {
	#if defined(C_GNU64)
	    long    l[16];	/* .. size = 192 in 64 bit environments     */
	    int     i[16];
	#elif defined(C_MSC64)
	    long long	l[16];
	    int 	i[16];
	#else
	    long    l[8];	/* .. size = 80  in 16 bit environments     */
	    int     i[24];	/* .. size = 128 in 32 bit environments     */
	#endif
    } devstate; 		/* for use of device handler		    */
    struct  pximage *filtip[4]; /* this pximage is filter to ..., or NULL   */
    int 	devflags;	/* private				    */

    /*
     * Data & Facts & Hints
     */
    struct  pximagedata  d;	/* pixel descriptor			    */
    struct  pximagefacts f;	/* derived & other facts		    */
    struct  pximagehints h;	/* interpretation hints 		    */

    /*
     * Read/write pixels into vector.
     *
     * The ioset() tests whether ioread()/iorite() supports the specified
     * options, and inits for later use of ioread()/iorite(); the mode is a
     * combination of: PXRXSCAN,PXRYSCAN,PXIWRAP,PXIXYVALID;
     * the data is a combination of PXDAT*; if PXDATLSB or PXDATMSB,
     * the significant pixels bits are left/right justified, if insufficient
     * room, the most significant are kept; if neither PXDATLSB or PXDATMSB,
     * the pixel data is never shifted, if insufficent room, the lsb are kept;
     * if PXDATDIRTY, the unused bits may be, or are, dirty;
     * the colormap is a bitmap: 0x03 for colors 0 & 1.
     * The ioread()/iorite() accesses pixels, returns number of pixels accessed;
     * the mode is a combination of PXIASYNC,PXIMAYMOD
     * in async modes, 0 is returned.
     * The iowait() must be used after ioread()/iorite() iff async is requested.
     * The wait is: PXAWAIT,PXASYNC,PXABORT.
     * The iolen() returns the number of pixels that ioread()/iorite() would xfer.
     *
     * If ioset() fails, the other functions always return 0, except iowait()
     * which returns PXODONE.
     */
    pximcnt_t (_cfunfcc *ioread)(struct pximage *tp,int mode,void _far16p *p,pximcnt_t n,pxcoord_t x,pxcoord_t y);
    pximcnt_t (_cfunfcc *iorite)(struct pximage *tp,int mode,void _far16p *p,pximcnt_t n,pxcoord_t x,pxcoord_t y);
    int       (_cfunfcc *iowait)(struct pximage *tp,int wait,void _far16p *p);
    int       (_cfunfcc *ioset)(struct pximage *tp,int mode,int data,uint colormap);
    pximcnt_t (_cfunfcc *iolen)(struct pximage *tp,pximcnt_t n,pxcoord_t x,pxcoord_t y);

    /*
     * Read/write/modify lists of pixels.
     *
     * The bxts() tests whether bxta() supports the specified options,
     * and inits for later use of bxta(); the
     * mode is a combination of: PXIXYVALID (currently assumed present);
     * the data is a combination of PXDAT*
     * the colormap is a bitmap: 0x03 for colors 0 & 1.
     * The bxtp() prepares addresses for bxta(); the
     * mode is a combination of PXIBXTC; the x, y coordinates are
     * assumed to be valid! Returns PXIBXTC bit if x,y coordinates were
     * overwritten.
     * The bxta() accesses the list of pixels, returns number of pixels
     * accessed; the mode is a combination of: PXIASYNC,PXIBXTC
     * in async modes, 0 is returned.
     * The data is a combination of PXDAT*
     * The wait is: PXAWAIT,PXASYNC,PXABORT
     *
     * If bxts() fails, the other functions always return 0, except bxtw()
     * which returns PXODONE.
     */
    int       (_cfunfcc *bxts)(struct pximage *tp,int mode,int data,uint colormap);
    int       (_cfunfcc *bxtp)(struct pximage *tp,int mode,void _far16p *p,struct pximadrs _far16p *adrsp,pximcnt_t n,int op);
    pximcnt_t (_cfunfcc *bxta)(struct pximage *tp,int mode,void _far16p *p,struct pximadrs _far16p *adrsp,pximcnt_t n,int op);
    int       (_cfunfcc *bxtw)(struct pximage *tp,int wait,void _far16p *p,struct pximadrs _far16p *adrsp);

    /*
     * Obtain pointer to actual image pixels, if possible.
     *
     * The imapset() tests if access is available and inits for
     * later use of imap().
     * The mode is a combination of: PXRXSCAN,PXRYSCAN,PXIWRAP,PXIMAPINC,
     * PXIMAPNOTFAR,PXIXYVALID.
     * The data is a combination of PXDAT*
     * the colormap is a bitmap: e.g. 0x03 for colors 0 & 1.
     * Returns <0 if imap() service unavailable; else >0 and PXIMAP* flags.
     *
     * The imap() requests access.
     * Returns !0 if access granted, else 0.
     *
     * The imapr() releases access; it can always be called with
     * the pximap structure given to imap(), even if imap() failed.
     * The mp->valid is cleared; other fields left alone.
     *
     * If imapset fails, imap() always returns 0.
     */
    int  (_cfunfcc *imapset)(struct pximage *tp,int mode,int data,uint colormap);
    int  (_cfunfcc *imap)(struct pximage *tp,struct pximap *mp,pxcoord_t x,pxcoord_t y);
    void (_cfunfcc *imapr)(struct pximage *tp,struct pximap *mp);

    /*
     * Advice/Errors/Info/Options/Universal.
     *
     * Advice on optimal, reasonable or suggested access size.
     * The service is 'I' for ioread()/iorite(), 'B' for bxta().
     * For 'B', the size is always <= UINT_MAX/sizs(pximadrs).
     * For 'I', the size is always <= UINT_MAX.
     * Cast to uint.
     *
     * Info on image access time & overhead; see PXACC* below.
     * The service is 'i' for ioread()/iorite(), 'b' for bxta(), 'm' for imap().
     * Info for imap() isn't for the imap function per se, but
     * time & overhead of using the returned pointer.
     * NB: Values returned may change from moment to moment (i.e. when
     * accessing some imaging boards with video on versus off, etc.).
     * Very approximate!!!
     *
     * Errors from underlying device, service is 'e'.
     * Accessing or attempting to access pixels beyond the image AOI
     * is not a recordable error; Disk I/O failure, for example, is.
     * If parm1!=0, the error is cleared after reporting; multiple
     * errors are individually cleared as they are reported.
     * If parm2!=0 and this pximage is used as a filter, any/all
     * underlying pximage's are also checked for error; 1st such error is
     * reported and optionally cleared (as per parm1).
     * Cast to signed int, interpret as PXER* code.
     *
     * Service 'f': undocumented.
     * Service 'p': undocumented.
     * Service 'z': undocumented.
     * Service 'r': undocumented.
     * Service 's*': undocumented.
     * Service 'g*': undocumented.
     * Service '*x': undocumented.
     * Service '*X': undocumented.
     * Service 'cm': undocumented.
     */
    pximaeiou_t (_cfunfcc *aeiou)(struct pximage *tp, int service, int parm1, int parm2, void *parm3);

    /*
     * Get dimensions or set new window.
     * The mode is 'i' to get full image dimensions,
     * 'w' to get window dimensions, 's' to set window dimensions.
     * For gets, the window is both copied to *wp (if !NULL) and returned.
     * For set, if !wp, the image dimension is set.
     */
    struct pxywindow * (_cfunfcc * xwind)(struct pximage *tp, struct pxywindow *wp, int mode);
};
typedef struct pximage pximage_s;

/*
 * Like pximage, but for 3-D.
 * See pximage for discussions.
 */
struct	pximage3
{
    struct pxyzwindow wind;	/* subwindow within imdim		    */
    struct pxyzwindow imdim;	/* nw.x = nw.y = nw.z = 0,		    */
				/* .. se.* is dimension of image	    */

    char    devname[8]; 	/* unique char name of device		    */
    struct {
	#if defined(C_GNU64)
	    long     l[16];
	    int      i[16];
	#elif defined(C_MSC64)
	    long long	l[16];
	    int 	i[16];
	#else
	    long    l[8];
	    int     i[24];
	#endif
    } devstate; 		/* for use of device handler		    */
    struct  pximage3 *filtip[4];/* this pximage is filter to ..., or NULL   */
    int 	devflags;	/* private				    */

    struct  pximagedata  d;	/* pixel descriptor			    */
    struct  pximagefacts f;	/* derived & other facts		    */
    struct  pximagehints h;	/* interpretation hints 		    */

    pximcnt_t (_cfunfcc *ioread)(struct pximage3 *tp,int mode,void _far16p *p,pximcnt_t n,pxcoord_t x,pxcoord_t y,pxcoord_t z);
    pximcnt_t (_cfunfcc *iorite)(struct pximage3 *tp,int mode,void _far16p *p,pximcnt_t n,pxcoord_t x,pxcoord_t y,pxcoord_t z);
    int       (_cfunfcc *iowait)(struct pximage3 *tp,int wait,void _far16p *p);
    int       (_cfunfcc *ioset)(struct pximage3 *tp,int mode,int data,uint colormap);
    pximcnt_t (_cfunfcc *iolen)(struct pximage3 *tp,pximcnt_t n,pxcoord_t x,pxcoord_t y,pxcoord_t z);

    int       (_cfunfcc *bxts)(struct pximage3 *tp,int mode,int data,uint colormap);
    int       (_cfunfcc *bxtp)(struct pximage3 *tp,int mode,void _far16p *p,struct pximadrs _far16p *adrsp,pximcnt_t n,int op);
    pximcnt_t (_cfunfcc *bxta)(struct pximage3 *tp,int mode,void _far16p *p,struct pximadrs _far16p *adrsp,pximcnt_t n,int op);
    int       (_cfunfcc *bxtw)(struct pximage3 *tp,int wait,void _far16p *p,struct pximadrs _far16p *adrsp);

    int  (_cfunfcc *imapset)(struct pximage3 *tp,int mode,int data,uint colormap);
    int  (_cfunfcc *imap)(struct pximage3 *tp,struct pximap *mp,pxcoord_t x,pxcoord_t y,pxcoord_t z);
    void (_cfunfcc *imapr)(struct pximage3 *tp,struct pximap *mp);

    pximaeiou_t (_cfunfcc *aeiou)(struct pximage3 *tp, int service, int parm1, int parm2, void *parm3);

    struct pxyzwindow * (_cfunfcc * xwind)(struct pximage3 *tp, struct pxyzwindow *wp, int mode);
};
typedef struct pximage3 pximage3_s;


/*
 * Result flags and fields for aeiou's 'i', 'b', 'm' services.
 * These are primarily intended for internal use, may
 * drastically change from release to release, and
 * significant 'artistic license' is to be expected in
 * interpretation and implementation.
 */
#define PXACCDEPTH  0x0000FF	/* levels of pximage's and/or processing,                                   */
				/* code may assume these are low order bits				    */
#define PXACCOCAM   0x000100	/* access raw data on/in/via cache					    */
#define PXACCORAM   0x000200	/* access raw data on/in/via ram					    */
#define PXACCOROM   0x000400	/* access raw data on/in/via rom					    */
#define PXACCOBUS   0x000800	/* access raw data on/in/via bus					    */
#define PXACCOSYS   0x001000	/* access raw data on/in/via device driver(s)				    */
#define PXACCOFIL   0x002000	/* access raw data on/in/via file I/O					    */
#define PXACCOEXT   0x004000	/* access raw data on/in/via external event (e.g. video blanking)	    */
				/* On 16 bit platforms, only above passed as so-called 'accflags'	    */
				/* to constructors							    */
#define PXACCIT1P   0x010000	/* r/w involves 1-d or 1.5-d multi-tap correction of pixel order	    */
#define PXACCIT2P   0x020000	/* r/w involves 2-d multi-tap correction of pixel order 		    */
#define PXACCIOSS   0x040000	/* r/w involves offset & scaling & shifting of values			    */
#define PXACCI3M3   0x080000	/* r/w involves 3x3 matrix (or worse) operations on color values	    */
#define PXACCI3N3   0x100000	/* r/w involves 3x3 neigborhoods (or worse) of values such as Bayer pattern */
#define PXACCI5N5   0x200000	/* r/w involves 5x5 neigborhoods (or worse) of values such as Bayer pattern */
#define PXACCIBIC   0x400000	/* r/w involves bit manipulation - compression				    */
#define PXACCGEMM   0x800000	/* r/w involves overlay geometry & masking				    */


#ifdef  __cplusplus
}
#endif

#include "cext_hpe.h"     
#endif				/* !defined(__EPIX_PXIMAGE_DEFINED) */
