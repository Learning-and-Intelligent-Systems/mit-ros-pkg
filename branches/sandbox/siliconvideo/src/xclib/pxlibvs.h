/*
 *
 *	pxlibvs.h	External	09-Jun-2010
 *
 *	Copyright (C)  1999-2010  EPIX, Inc.  All rights reserved.
 *
 *	Frame Grabber Library: Video State (Format) Definitions
 *
 *	Most end-users will not modify these structures directly,
 *	but either use the defaults or change and export the
 *	default with SVIP/4MIP/XCIP/XCAP and import the new
 *	structures into the frame grabber library.
 *
 */


#if !defined(__EPIX_PXLIBVS_DEFINED)
#define __EPIX_PXLIBVS_DEFINED
#include "cext_hps.h"     


/*
 * Fundamental types.
 */
#if defined(C_GNU64)|defined(C_MSC64)
    /*
     * The pxbuffer_t has long been equivalent to 'long';
     * Kept so, even tho different between 64 bit Linux and 64 bit Windows,
     * and even tho 64 bit buffer indices are unlikely.
     */
    typedef long	    pxbuffer_t;     /* an image buffer number		*/
    typedef uint32	    pxvbtime_t;     /* a line/field/frame count 	*/
    typedef uint64	    pxapiadrs_t;    /* address in frame buffer memory	*/
    typedef uint64	    pximaplen_t;    /* length in/of frame buffer memory */
#else
    typedef long	    pxbuffer_t;     /* an image buffer number		*/
    typedef long	    pxvbtime_t;     /* a line/field/frame count 	*/
    typedef uint32	    pxapiadrs_t;
    typedef ulong	    pximaplen_t;
#endif


/*
 * Camera timing mode constants.
 */
#define PXCAM_PERIODIC		0x001	/* periodic, free-run					    */
#define PXCAM_TRIGEXT		0x002	/* not periodic, externally triggered one shot snap	    */
#define PXCAM_TRIGSOFT		0x004	/* not periodic, software triggered one shot snap	    */
#define PXCAM_TRIGEXTRPT	0x008	/* quasi-periodic, sequence of externally triggered fields  */
#define PXCAM_APERIODIC 	0x010	/* aperiodic, unspecified				    */
#define PXCAM_TRIGINITPERIODIC	0x020	/* periodic after trigger start 			    */
#define PXCAM_RESETTABLE	0x100	/* resettable - adjective				    */

/*
 * Field Selection constants.
 */
#define PXFIELD_NXT  0x0001	/* do at start of next field		    */
#define PXFIELD_ODD  0x0002	/* do at start of next odd field	    */
#define PXFIELD_EVN  0x0004	/* do at start of next even field	    */
#define PXFIELD_NOW  0x0100	/* do immediately, even in middle of field  */

/*
 * Genlock/master mode constants.
 */
#define PXSYNC_MASTER	0x01
#define PXSYNC_SLAVE	0x02

/*
 * Camera/format class constants.
 */
#define PXSCN_UNKNOWN	    0x0000
#define PXSCN_AREASCAN	    0x0002
#define PXSCN_LINESCAN	    0x0001
#define PXSCN_RS170	    0x0102
#define PXSCN_NTSC	    0x0202
#define PXSCN_NTSC_J	    0x0302
#define PXSCN_CCIR	    0x0402
#define PXSCN_PAL	    0x0502
#define PXSCN_PAL_M	    0x0602
#define PXSCN_PAL_N	    0x0702
#define PXSCN_SECAM	    0x0802
#define PXSCN_PAL_N_COMBO   0x0902
#define PXSCN_PAL_60	    0x0A02
#define PXSCN_NTSC_443	    0x0B02
#define PXSCN_YC	    0x0010	// adjective for s-video
#define PXSCN_CUSTOM	    0x8000	// adjective

/*
 * Video mode/activity constants.
 */
#define PXMODE_DIGI	('z'<<8)
#define PXMODE_DISP	('p'<<8)
#define PXMODE_NVOT	('0'<<8)
#define PXMODE_BLCK	('b'<<8)

/*
 * Limits supported in the video state structures.
 * These are upper bounds, the hardware and/or driver
 * typically have lower limits, as appropriate.
 */
#define PXMAX_PIXBITS	16	/* max bits per pixel component 	*/
#define PXMAX_PIXIES	4	/* max component values per pixel	*/
#define PXMAX_FIELDS	6	/* max fields per frame 		*/
#define PXMAX_CHANNELS	4	/* max simultaneous input channels	*/
#define PXMAX_UNITS	16	/* max number of units (boards) 	*/
#define PXMAX_LHCM	4	/* see pxvidformat			*/
#define PXMAX_LH	2	/* see pxvidformat			*/
#define PXMAX_LOVC	8	/* see pxvidformat			*/
#define PXMAX_MMBM	4	/* see pxvidformat			*/
#define PXMAX_STATES	16	/* max number of states per unit	*/


/*
 * Pixel type encoding constants.
 * Does not use standard pximage PXDAT*, as hardware's pixel packing
 * may/might not correspond to 'normal' datatypes.
 */
#define PXMEM_1BYTE	0x0001	    /* 1 byte per pixie (or pixel(s))	*/
#define PXMEM_2BYTE	0x0002	    /* 2 byte per pixie (or pixel(s))	*/
#define PXMEM_3BYTE	0x0003	    /* 3 byte per pixie (or pixel(s))	*/
#define PXMEM_4BYTE	0x0004	    /* 4 byte per pixie (or pixel(s))	*/
#define PXMEM_5BYTE	0x0005	    /* 5 byte per pixie (or pixel(s))	*/
#define PXMEM_6BYTE	0x0006	    /* 6 byte per pixie (or pixel(s))	*/
#define PXMEM_7BYTE	0x0007	    /* 7 byte per pixie (or pixel(s))	*/
#define PXMEM_8BYTE	0x0008	    /* 8 byte per pixie (or pixel(s))	*/
#define PXMEM_9BYTE	0x0009	    /* 9 byte per pixie (or pixel(s))	*/
#define PXMEM_10BYTE	0x000A	    /* 10 byte per pixie (or pixel(s))	*/
#define PXMEM_11BYTE	0x000B	    /* 11 byte per pixie (or pixel(s))	*/
#define PXMEM_12BYTE	0x000C	    /* 12 byte per pixie (or pixel(s))	*/
#define PXMEM_NBYTE	0x000F	    /* N byte per pixie (or pixel(s))	*/
#define PXMEM_LSB	0x0000	    /* left justified (pixiebits < max) */
#define PXMEM_MSB	0x0010	    /* right justified (pixiebits < max)*/
#define PXMEM_LTLEND	0x0000	    /* little endian (> 1 byte) 	*/
#define PXMEM_BIGEND	0x0100	    /* big endian    (> 1 byte) 	*/
#define PXMEM_LTOR	0x0000	    /* pixies packed left to right	*/
#define PXMEM_RTOL	0x0200	    /* pixies packed right to left	*/
#define PXMEM_DIRTY	0x0800	    /* unused bits, if any, are dirty	*/
#define PXMEM_PACKED	0x0400	    /* multiple pixies are packed, the	*/
				    /* NBYTE is per pixel, not pixie	*/
#define PXMEM_POCKED	0x1000	    /* multiple pixels are packed, the	*/
				    /* NBYTE is per pixel group 	*/


/*
 * Structure field characterizations.
 * Preliminary.
 */
#define PXDEF_USED	0x0001	    /* is this value used, in any manner    */
#define PXDEF_SETTABLE	0x0002	    /* value is settable		    */
#define PXDEF_REPORT	0x0004	    /* value is sensed and reported	    */
#define PXDEF_FIXED	0x0008	    /* value is preset			    */
#define PXDEF_HARD	0x0010	    /* value fixed due to hardware	    */
#define PXDEF_STATE	0x0010	    /* value a function of video state	    */
#define PXDEF_SETTINGS	0x0020	    /* value a function of other settings   */
#define PXDEF_HMOD	0x1000	    /* can implement change on-the-H-fly    */
#define PXDEF_VMOD	0x2000	    /* can implement change on-the-V-fly    */

/*
 * Board/library family codes.
 */
#define PXFAM_SILICONVIDEO  (('s'<<8)|'v')  // nickname SV
#define PXFAM_4MEGVIDEO     (('4'<<8)|'m')  // nickname 4M (or M4)
#define PXFAM_PIXCI	    (('x'<<8)|'c')  // nickname XC
#define PXFAM_1394	    (('f'<<8)|'w')  // nickname FW

/*
 * PXMAX_LHCM decomposition
 */
#define PXLHCM_MIN	    0	    /* min	    */
#define PXLHCM_MAX	    1	    /* max	    */
#define PXLHCM_CNG	    2	    /* congruent    */
#define PXLHCM_MOD	    3	    /* modulus	    */


/*
 * PXMAX_MMBM decomposition
 */
#define PXMMBM_MINBITS	    0	    /* min # of bits		*/
#define PXMMBM_MAXBITS	    1	    /* max # of bits		*/
#define PXMMBM_MASKBITS     2	    /* bitmask of allowed bits	*/
#define PXMMBM_RSVD	    3	    /* reserved 		*/


/*
 * PXMAX_LH decomposition
 */
#define PXLH_MIN	    0	    /* min	    */
#define PXLH_MAX	    1	    /* max	    */


/*
 * Prefix component to other video state structures.
 * The 'mos' is the number of initializer values,
 * the 'len' is the byte len; both, of course, including
 * the structure in which this is embedded.
 * Other fields are for internal use.
 */
struct pxddch {
    uint16  len;	    /* version id: byte len  */
    uint16  mos;	    /* version id: m.o.s.'s  */
    uint16  id; 	    /* internal use	     */
    uint16  op; 	    /* internal use	     */
    uint16  un; 	    /* internal use	     */
    uint16  rw; 	    /* internal use	     */
    uint16  fu; 	    /* internal use	     */
    uint16  rs; 	    /* internal use	     */
};
#define PXMOS_DDCH  (8)


/*
 * Video settings/state/format is represented by a set of structures,
 * collected here as a vector of pointers.
 * Some structures may not be used, NULLs are to be expected.
 */
struct pxvidstate {
    struct  pxddch  ddch;

    /*
     * Common components, tho not all components
     * are necessarily supported by any particular
     * frame grabber.
     */
    struct  pxvidformat     *vidformat;
    struct  pxvidres	    *vidres;
    struct  pxvidmode	    *vidmode;
    struct  pxvidphys	    *vidphys;
    struct  pxvidimage	    *vidimage;
    struct  pxvidopt	    *vidopt;
    struct  pxvidmem	    *vidmem;
    struct  pxcamcntl	    *camcntl;

    /*
     * Internal use.
     */
    void	*i;	    /* ?  */
    void	*d;	    /* ??  */
    int 	id;	    /* ??? */

    /*
     * Board family specific structures; NULL if not used.
     */
    struct {	/* PIXCI(R) imaging boards	    */
	struct	xcsv2format	*sv2format;
	struct	xcsv2mode	*sv2mode;
	struct	xcdxxformat	*dxxformat;
    } xc;

    struct {	/* SILICON VIDEO(R) imaging boards  */
	struct	svformat    *format;
    } sv;

    struct {	/* 4MEG VIDEO(tm) imaging boards    */
	struct	m4format    *format;
    } m4;
};
typedef struct pxvidstate   pxvidstate_s;
#define PXMOS_VIDSTATE	    (PXMOS_DDCH+6+3+(3+1+1))


/*
 * Device independent video format attributes.
 * Many fields are limits on allowable values in
 * other structures, allowing those other structures
 * to be carelessly 'tweaked' by a user, and then corrected
 * against the contents of this structure.
 *
 * Using the SVIP, 4MIP, XCIP, or XCAP (as appropriate) GUI
 * to load/edit/save video state structures is recommended,
 * but especially for the pxvidformat!
 */
struct	pxvidformat {
    struct  pxddch  ddch;

    /*
     * Frame grabber family, model & submodel for which
     * this format was created. Supports exporting/importing
     * formats with version control.
     */
    int     fgfamily;	    /* family code				*/
    int     fgmodel;	    /* family dependent model code		*/
    int     fgsubmodel;     /* family/model dependent submodel code	*/
    int     fgcustomid;     /* customer customization code, if any	*/
    int     fgdefdate[3];   /* year/month/day format originally created */
    char    fgcamera[80];   /* camera created for, tested with		*/
    int     fgsignature;    /* authenticate predefined formats		*/
    int     fgcamdope[4];   /* internal 				*/
			    /* camdope[3]: flags for compat. guide	*/

    /*
     * Limits on pxvidres.
     * The [PXMAX_LHCM] values are min/max/congruence/modulus
     * respectively, a modulus and congurence of 0 is ignored.
     * The [PXMAX_LOVC] values are lists of valid choices,
     * a list with list[0]==0 is ignored.
     * The [PXMAX_MMBM] are min # of bits, max # of bits, and
     * bitmask of allowed bits.
     */
    int     xviddim[PXMAX_LHCM];    /* allowed video pixels per line	   */
    int     xdatdim[PXMAX_LHCM];    /* allowed data pixels per line	   */
    int     yviddim[PXMAX_LHCM];    /* allowed video lines per field	   */
    int     ydatdim[PXMAX_LHCM];    /* allowed data lines per field	   */
    int     xvidoffset[PXMAX_LHCM]; /* allowed video offset per line	   */
    int     yvidoffset[PXMAX_LHCM]; /* allowed video offset lines in field */
    int     xvidoffdim[PXMAX_LHCM]; /* allowed video pixels+offset per line*/
    int     yvidoffdim[PXMAX_LHCM]; /* allowed video lines+offset in field */

    int     datfields[PXMAX_LHCM];  /* allowed datfields		   */
    int     datchannels[PXMAX_LHCM];/* allowed datchannels		   */
    int     datphylds[PXMAX_LHCM];  /* allowed datphylds		   */
    int     rsvd2[4];


    /*
     * Limits on pxvidmode
     */
    int     vidylaces[PXMAX_MMBM];	 /* allowed vidylace			*/
    int     vidchannels[PXMAX_MMBM];	 /* allowed vidchannels 		*/
    int     inputmuxs[PXMAX_LOVC];	 /* allowed inputmux			*/
    int     outputmuxs[PXMAX_LOVC];	 /* allowed outputmux			*/
    int     splitscreens[PXMAX_LOVC];	 /* allowed splitscreen 		*/
    int     syncsources[PXMAX_LOVC];	 /* allowed locking modes, PXSYNC*	*/
    int     videomodes[PXMAX_LOVC];	 /* expected PXMODE*, particularly w.	*/
					 /* using loadable microcode		*/
    int     fieldmods[PXMAX_LOVC];	 /* allowed field mods, PXFIELD*	*/
    int     buffers[PXMAX_LHCM];	 /* allowed bufffer			*/
    int     vidxlaces[PXMAX_MMBM];	 /* allowed vidxlace			*/
    int     rsvd3[4];

    /*
     * Additional, common video characteristics.
     * The vperiod may not be the reciprocal of framerate,
     * such as a camera outputing 33 msec frames, but triggered
     * to do so 10 times a second.
     */
    struct pxvidformat_cvc {
	int	hoffset;	    /* offset of minimal left edge of active	*/
				    /* video from horizontal blanking		*/
	int	voffset;	    /* offset of minimal top edge of active	*/
				    /* video from vertical blanking		*/
	int	fieldsperframe;     /* fields per frame 			*/
	int	pixelsperclock;     /* pixels per pixel clock, or 0		*/
	int	pixiesperclock;     /* 0 or pixies per pixel clock if above 0	*/
	int	linesperframe;	    /* lines per frame, active & inactive	*/
	int	pixelsperline;	    /* pixels per line, active & inactive	*/
	sint32	pixelclkfreq[2];    /* pixel clock frequency, MHz, rational	*/
	sint32	vperiod[2];	    /* V period, seconds, rational		*/
	sint32	hperiod[2];	    /* H period, seconds, rational		*/
	sint32	framerate[2];	    /* frames per sec, rational 		*/
				    /* same as 1/(vperiod) in periodic modes	*/
	int	fieldperiodicity;   /* PXCAM*, etc				*/
	int	lineperiodicity;    /* as above, per line, for line scan	*/
	int	hscanorder;	    /* 'l'|'r': scan left to rt or vice versa	*/
	int	vscanorder;	    /* 't'|'b': scan top to btm or vice versa	*/
	int	framespertrigger;   /* frames captured per trigger		*/
	int	scanclass;	    /* PXSCN*					*/
	int	phyldsperframe;     /* phylds per frame 			*/
	int	rsvd4[7];
    } as, is;			    /* as: declarative, 0 if can't                      */
				    /* is: descriptive, 0 if not known, or copy of as	*/

    /*
     * Aspect ratio at full resolution.
     * Just like pximagehints, but rationals instead of floats.
     */
    sint32  pixelwidth[2];  /* X axis width of 1 pixel. 0 if unknown	*/
    sint32  pixelheight[2]; /* Y axis height of 1 pixel. 0 if unknown	*/
    sint32  pixeldepth[2];  /* not used. should be 0			*/
    int     widthunits;     /* PXUNIT* real world units 		*/
    int     heightunits;    /* PXUNIT* real world units 		*/
    int     depthunits;     /* not used. should be 0			*/

    /*
     * Misc
     */
    int     boardspercamera;	/* multiple boards needed per camera	    */
    int     linesyncupdelay;	/* spurious lines before resynced	    */
    int     fieldsyncupdelay;	/* spurious fields before resynced	    */
    int     framesyncupdelay;	/* spurious fields before resynced	    */
    int     fieldflip;		/* flip field sense (xor)		    */
    sint32  nosynctimeout;	/* no sync timeout after ... milliseconds   */
    int     phyldflip;		/* flip phyld sense (xor)		    */
    int     prefixdata; 	/* extra data bytes captured at start and   */
    int     postfixdata;	/* .. at end. May be 0 iff implied (new)    */
    int     imagesperframe;
    uint32  camlinkconfig;	/* camera link channel configuration, in 4  */
				/* bit nibbles. channel A=1, channel B=2,...*/
    int     rsvd5[3];

    /*
     * Pixel formats, PXMAX_LOVC 'entries'
     */
    struct pxvidformat_pix {
	int	pixhints[PXMAX_LOVC];	/* allowed pixel format 	    */
					/* and allowed pixies per pixel     */
					/* 0 if entry unused.		    */
	int	pixbits[PXMAX_LOVC][PXMAX_PIXIES][PXMAX_LH];
					/* allowed bits per pixie	    */
	int	pixmems[PXMAX_LOVC];	/* memory configuration, PXMEM*     */
	int	pixdopes[PXMAX_LOVC];	/* internal			    */
    } vid, dat, img;			/* for video, data, image	    */

			    /*						    */
			    /* Typical dat.pixhint's:                       */
			    /* PIXCI SV2,SV3:				    */
			    /*	    PXHINTCBYCRY			    */
			    /* PIXCI SV4,SV5				    */
			    /*	    PXHINTCBYCRY, PXHINTYCBYCR		    */
			    /*	    PXHINTBGR, PXHINTBGRX, PXHINTGREY	    */
			    /*	    (PXHINTUSER+PXHINTBGR+0xF0) 	    */
			    /* PIXCI SV7				    */
			    /* PIXCI D/D24/D32/D2X/CL1/CL2/E1/E4/E1DB/EL1DB/E4DB/EC1/EB1/EB1-PoCL/ECB1/ECB1-34/ECB2/EL1/SI:*/
			    /*	    PXHINTGREY, PXHINTBEYER		    */
			    /*	    (PXHINTUSER+PXHINTBGR+0xF0) 	    */
			    /*	    (PXHINTUSER+2+((PXHINTCBYCRY+[0-5])<<4) */
			    /*	    (PXHINTUSER+1+(PXHINTBAYER<<4))	    */
			    /*	    PXHINTBGRX				    */
			    /*	    (PXHINTUSER+PXHINTRGB+0x80) 	    */
			    /*	    (PXHINTUSER+PXHINTRGBX+0x80)	    */
			    /*	    PXHINTUSER+PXHINTRGB+(30<<4)	    */
			    /*	    PXHINTUSER+PXHINTBGR+(30<<4)	    */
			    /*	    PXHINTUSER+PXHINTBGR+((30^6)<<4)	    */
			    /* PIXCI A: 				    */
			    /*	    PXHINTGREY				    */
			    /*	0 for board default (replaced with real hint*/

};
typedef struct pxvidformat  pxvidformat_s;
#define PXMOS_VIDFORMAT     (PXMOS_DDCH +4+3+80+1+4 \
			    +11*PXMAX_LHCM+4+3*PXMAX_MMBM \
			    +6*PXMAX_LOVC+1*PXMAX_LHCM+4 \
			    +2*(7+4*2+7+7) +9 +7 +7 \
			    +3*(PXMAX_LOVC*(1+PXMAX_PIXIES*PXMAX_LH+2)))



/*
 * Data or color space conversions as required
 * to create useful image.
 * Note: None of these were used for SV or 4M.
 */
struct pxvidimage {
    struct  pxddch  ddch;

    /*
     * Multitap corrections
     */
    struct pxvidimage_multitap {
	int	   order;
	int	   mode;
	int	   arg[4];
    } mt;

    /*
     * Bayer pattern corrections/conversions
     */
    struct pxvidimage_bayerpattern {
	int	   order;
	int	   mode;
	int	   arg[4];
    } bp;

    /*
     * Color space conversion via matrix
     */
    struct pxvidimage_colorspace {
	int	    order;
	int	    mode;
	sint32	    intorgb[3][4];  /* convert to RGB, from YCrCb1	    */
	sint32	    fromrgb[3][4];  /* convert from RGB1, to YCrCb	    */
	sint32	    scale;	    /* results scaled down by .. (256)	    */
      /*int	    rsvd[4];	    // future addition			    */
    } cs;

    /*
     * Color white balancing & gamma correction
     */
    struct pxvidimage_whitebalance {
	int	    order;
	int	    mode;	    /* 0 : unused, 3: used on RGB	*/
	sint32	    darkreference  [PXMAX_FIELDS][PXMAX_PIXIES];
	sint32	    darktarget	   [PXMAX_FIELDS][PXMAX_PIXIES];
	sint32	    brightreference[PXMAX_FIELDS][PXMAX_PIXIES];
	sint32	    brighttarget   [PXMAX_FIELDS][PXMAX_PIXIES];
	sint32	    gamma	   [PXMAX_FIELDS][PXMAX_PIXIES];
				    /* scaled by 1000			*/

	    /* formerly rsvdargs[]:							     */
	    /*	GAMMA		6						    */
	    /*	DARKREF 	0 - 2	DARKTARGET	7   BRIGHTODDREF    9 -11   */
	    /*	BRIGHTREF	3 - 5	BRIGHTTARGET	8   BRIGHTODDTARGET 12	    */
      /*int	    rsvd[4];	    // future addition			    */
    } wb;

    /*
     * Multi frame planar color
     */
    struct pxvidimage_planarcolor {
	int	    order;
	int	    mode;
	int	    arg[4];
    } pc;

    /*
     * Multi color shifts
     */
    struct pxvidimage_shiftcolor {
	int	    order;
	int	    mode;
	int	    arg[4];
    } sc;

    /*
     * Resize
     */
    struct pxvidimage_resize {
	int	    order;
	int	    mode;
	int	    xdim;
	int	    ydim;
	int	    arg[2];
    } rz;

    /*
     * Per pixel offset & gain & defect replacement
     * It is misnamed - historical.
     */
    struct pxvidimage_gainoffset {
	int	    order;
	int	    mode;
	int	    offsbuffer[2];  /* may be negative - relative to last buffer! */
	int	    gainbuffer[2];  /* ditto. N.B. Must be next to offsbuffer[]   */
	int	    arg[6];
	int	    dfctbuffer[2];  /* late addition - historical ordering!	  */
    } go;

    /*
     * Normalization
     */
    struct pxvidimage_normalize {
	int	    order;
	int	    mode;
	sint32	    black[2][PXMAX_PIXIES]; // future: should be PXMAX_FIELDS
	sint32	    coef1[2][PXMAX_PIXIES]; // but insufficient space in this
	sint32	    coef2[2][PXMAX_PIXIES]; // version of struct
	sint32	    scale;
	int	    arg[3];
    } no;

    /*
     * Future?
    struct pxvidimage_vbi {
	int	    order;
	int	    mode;
	int	    xdim;
	int	    ydim;
	int	    xoffs;
	int	    yoffs;
	int	    arg[4];
    } vb;
    */

    struct pxvidimage_sharpen {
	int	    order;
	int	    mode;
	sint32	    into[3];	    /* coefficients			*/
	sint32	    from[3];	    /* coefficients			*/
	sint32	    scale;	    /* results scaled down by .. (256)	*/
	int	    arg[2];
    } sh;

    struct pxvidimage_colorspace2 {
	int	    arg[4];	    /* future: move into colorspace	*/
    } cs2;

    struct _pxvidimage_pixelprecision {
	int	    order;
	int	    mode;
	int	    arg[4];
    } pp;

    int rsvd1[22-11-4-6];
};
typedef struct pxvidimage   pxvidimage_s;
#define PXMOS_VIDIMAGE	    (PXMOS_DDCH +6 +6 +2+2*3*4+1 \
			     +2+5*PXMAX_FIELDS*PXMAX_PIXIES +6 +6 +6 +14 \
			     +2+3*2*PXMAX_PIXIES+1+3 \
			     +20/*+2*/)


/*
 * Parameters controlling the viewable image, as distinct
 * from timing formats in pxvidformat; there is, of course, overlap.
 * However these can be adjusted at whim, and will be corrected
 * against the video format and the frame grabber's capabilities.
 * Unlike pxvidmode, these affect the size of frame buffers,
 * and thus the partitioning of frame buffer memory.
 */
struct	pxviddim {	    /* common control over x and y			    */

    int     datsamples;     /* data pixels per line or column			    */
    int     vidsamples;     /* video pixels per line/column			    */
			    /* SV & 4M allow integral ratios			    */
			    /* XC SV2/3/4/5 allow arbitrary ratios		    */
			    /* XC D/D24/D32/A/D2X/CL1/CL2/E1/E4/E1DB/EL1DB/E4DB     */
			    /*	  EB1/EB1PoCL/EC1/ECB1/ECB1-34/ECB2/EL1 	    */
			    /*	  SI/SI1/SI2/SI4/CL3SD/SV7 only allow equality	    */

    int     vidoffset;	    /* video pixel offset from left/top edge		    */
			    /* relative to offset in format structure!		    */
    int     vidoffsend;     /* reported: video pixel right/bottom edge inclusive    */
			    /* relative to offset in format structure!		    */
    int     rsvd[8];

    int     setmaxdatsamples;  /* 0|1: force data samples per line/field to max, reset bit if can't  */
			       /* 0|2: ditto, but leave bit set if can't (due to insufficient memory)*/
    int     setmaxvidsamples;  /* 0|1: force video samples per line/field to max, reset bit if can't */
			       /* 0|2: ditto, but leave bit set if can't (due to insufficient memory)*/
    int     setmidvidoffset;   /* 0|1: force offset to center video				     */

    struct pxviddim_vs {    /* for variable width pixel/line sampling intervals (SV&4M) */
	int	on;	    /* 0|1: variable sampling interval in use			*/
	int	parm[8];    /* parms interpreted as per code value of vs.on		*/
    } vs;

    struct pxviddim_sv {    /* for split screen digitize/display (SV & 4M): */
	int	on;	    /* 0|1: split video in use			    */
	int	pos;	    /*	division position, pixels/lines 	    */
	int	spc;	    /*	division spacing. blank pixels/lines	    */
    } sv;
};
struct	pxvidres {
    struct  pxddch  ddch;

    struct  pxviddim   x;	/* for x, horizontal direction			    */
    struct  pxviddim   y;	/* for y, vertical direction			    */

    int     datfields;		/* number of data fields per frame buffer	    */
				/* e.g. if 1, both fields of interlaced video are   */
				/* capture/displayed from the same 'field buffer'.  */
    int     datchannels;	/* number of data channels per frame buffer	    */

    int     setmaxdatfields;	/* 0|1: force datfields to max			    */
    int     setmaxdatchannels;	/* 0|1: force datchannels to max		    */
    int     datphylds;		/* number of data phylds per frame buffer	    */
    int     setmaxdatphylds;	/* 0|1: force datphylds to max			    */
    int     rsvd1[6];

    struct pxvidres_pix {
	int	pixies; 	/* color components per pixel				*/
	int	pixiebits[PXMAX_PIXIES];  /* bits per data component			*/
	int	pixhint;	/* pixel format/interpretation. 0 for default(replaced) */
	int	pixmem; 	/* internal use 					*/
	int	pixdope;	/* internal use 					*/
	int	setpixbits;	/* 0|1|2: force pixies/pixhint/pixiebits to max 	*/
				/*	  force pixies/pixhint/pixiebits to default	*/
    } vid, dat, img;	    /* for video, data, image					*/
			    /* e.g. for UYVY: dat.pixies=2, img.pixies=3		*/
			    /* Currently, only dat.* need be set, img.* is set by the	*/
			    /* library, vid.* is reserved.				*/
			    /* CL2/E1/E4/EB1/EB1PoCL/EC1/ECB1/ECB1-34/ECB2/EL1/E1DB/EL1DB/E4DB uses  */
			    /* both dat.* and vid.*					*/

    /*
     * Reported.
     * Any values when saved, loaded, defined, ... are ignored.
     * N.B. For interlaced capture into line order (not field
     * order), the field values are the same as frame values!
     */
    struct pxvidres_mem {   /* memory requirement			*/
	uint32	pd;	    /* pixel data				*/
	uint32	pdod;	    /* pixel & other data			*/
	uint32	pdodal;     /* pixel & other data & alignment		*/
	uint32	pdodalsf;   /* pixel & other data & alignment		*/
			    /*	& software fields			*/
	uint32	rsvd2[1];
    } line, field, frame;

    pxbuffer_t framebuffers;/* # of usable frame buffers in frame buffer memory */
    pxbuffer_t rsvdbuffers; /* # of rsvd buffers in frame buffer memory   */
    int     rsvd3[7];
};
typedef struct pxvidres     pxvidres_s;
#define PXMOS_VIDRES	    (PXMOS_DDCH+2*(27)+6+6+3*(1+PXMAX_PIXIES+4)+3*(3+2)+2+7)


/*
 * Physical description, addresses and strides, of frame buffer.
 * Usage and interpretation is frame grabber specific.
 * Almost never set directly, rather the pxvidmode.buffer
 * is set, and this automatically filled as per the
 * specified sequential buffer number.
 * Some applications might disable the auto filling and
 * set this directly.
 */
struct pxvidphys {
    struct  pxddch  ddch;

    pxapiadrs_t startadrs [PXMAX_FIELDS][PXMAX_PIXIES]; /* start address	    */
    pxapiadrs_t endadrs   [PXMAX_FIELDS][PXMAX_PIXIES]; /* end address, if used     */
    uint32	linelength[PXMAX_FIELDS][PXMAX_PIXIES]; /* width of line, bytes     */
    uint32	linestride[PXMAX_FIELDS][PXMAX_PIXIES]; /* extra stride per line    */

    pxapiadrs_t bufadrs;    /* start address			    */
    pxapiadrs_t bufsize;    /* next buffer is at bufadrs+bufsize    */
    uint32  bufxdim;	    /* pixels per line			    */
    uint32  bufydim;	    /* lines per field			    */
    uint32  bufidim;	    /* fields per frame 		    */
    uint32  bufzdim;	    /* number of buffers		    */

    uint32  upatita;	    /* 0: Addresses relative to frame buffer memory.*/
			    /* .. Otherwise, addresses are user space or    */
			    /* .. physical and this is the signed	    */
			    /* .. authorization to make it so		    */

    uint32  bufjdim;	    /* phylds per frame 		    */
    uint32  rsvd[3];


    pxbuffer_t	buffer;     /* arbitrary flags 'carried' with the above info */
    int 	stateid;    /* .. derived from pxvidmode.buffer if !useimphys*/
    int 	tracker;    /* internal use				     */
  //int 	flags[4];   /* internal use				     */
};
typedef struct pxvidphys pxvidphys_s;
#define PXMOS_VIDPHYS	    (PXMOS_DDCH+4*PXMAX_FIELDS*PXMAX_PIXIES+6+1+4+3)

#if defined(OS_WINNT)|defined(OS_WINNT_DLL)
struct pxvidphys_thunk3264 {
    struct  pxddch  ddch;

    uint64	startadrs [PXMAX_FIELDS][PXMAX_PIXIES]; /* start address	    */
    uint64	endadrs   [PXMAX_FIELDS][PXMAX_PIXIES]; /* end address, if used     */
    uint32	linelength[PXMAX_FIELDS][PXMAX_PIXIES]; /* width of line, bytes     */
    uint32	linestride[PXMAX_FIELDS][PXMAX_PIXIES]; /* extra stride per line    */

    uint64	bufadrs;    /* start address			    */
    uint64	bufsize;    /* next buffer is at bufadrs+bufsize    */
    uint32  bufxdim;	    /* pixels per line			    */
    uint32  bufydim;	    /* lines per field			    */
    uint32  bufidim;	    /* fields per frame 		    */
    uint32  bufzdim;	    /* number of buffers		    */

    uint32  upatita;	    /* 0: Addresses relative to frame buffer memory.*/
			    /* .. Otherwise, addresses are user space or    */
			    /* .. physical and this is the signed	    */
			    /* .. authorization to make it so		    */

    uint32  bufjdim;	    /* phylds per frame 		    */
    uint32  rsvd[3];


    pxbuffer_t	buffer;     /* arbitrary flags 'carried' with the above info */
    int 	stateid;    /* .. derived from pxvidmode.buffer if !useimphys*/
    int 	tracker;    /* internal use				     */
  //int 	flags[4];   /* internal use				     */
};
typedef struct pxvidphys_thunk3264 pxvidphys_thunk3264_s;
#endif

/*
 * Video modes.
 * Some parameters can be controlled at video rate.
 * Not all fields applicable to every format or supported by every device,
 * but they do have a consistent interpretation when used, unlike
 * the frame_grabber_mode's struct.
 */
struct	pxvidmode {
    struct  pxddch  ddch;

    int 	vidchannels;	    /* bit map of channels active		*/
    pxbuffer_t	buffer; 	    /* current buffer # 			*/
    pxy_s	panxy;		    /* pan/scroll upper left coordinate (SV&4M) */
    int 	useimphys;	    /* flag: use raw imphys, ignore 'buffer'	*/
    int 	rsvd2[8];

    int     vidmode    [PXMAX_CHANNELS];    /* activity,  PXMODE*		*/
    int     vidylace   [PXMAX_CHANNELS];    /* bit map of video fields active	*/
    int     syncsource [PXMAX_CHANNELS];    /* PXSYNC*				*/
    int     vidopt     [PXMAX_CHANNELS];    /* reserved 			*/
    int     inputmux   [PXMAX_CHANNELS];    /* video input & output ... 	*/
    int     outputmux  [PXMAX_CHANNELS];    /* ..multiplexor selection, 1 based!*/
    int     splitscreen[PXMAX_CHANNELS];    /* split screen selection (SV&4M)	*/
    int     fieldmod   [PXMAX_CHANNELS];    /* switch video after PXFIELD_*	*/
					    /* ignored for non interlaced video */
					    /* or vidylace selects only 1 field */
    int     vidxlace   [PXMAX_CHANNELS];    /* bit map of video phylds active	*/

    int     rsvd1      [PXMAX_CHANNELS*3];

    int     setmaxvidylace;		    /* 0|1: force vidylace to all   */
    int     setmaxvidchannels;		    /* 0|1: force vidchannels to all*/
    int     setmaxvidxlace;		    /* 0|1: force vidxlace to all   */

    int     rsvd3[7];
};
typedef struct pxvidmode    pxvidmode_s;
#define PXMOS_VIDMODE	    (PXMOS_DDCH+5+8+9*PXMAX_CHANNELS +3*PXMAX_CHANNELS+3+7)


/*
 * Relationships between states, and misc options.
 * Note: Used with SV and 4M; currently unused with XC family.
 */
struct pxvidopt {

    struct  pxddch  ddch;

    int     imopstate;	    /* !0: image buffer dimensions, adrs, and	    */
			    /* partitioning is defined by imopstate,	    */
			    /* this state is a subservient varient	    */
    int     imopmode;	    /* ignore if imopstate==0, else this is ...     */
			    /*	0: undefined				    */
			    /*	1: on a DISP: a video (sub)window of a	    */
			    /*	   DIGI imopstate			    */
			    /*	2: on a DIGI: alternate dimension but	    */
			    /*	   same # of pixels as DISP imopstate,	    */
			    /*	   which is the correct x,y  interpretation */
    int     panwraprnd;     /* allow pxvidmode.panxy to cause image	    */
			    /* wrap around: any panxy is legit		    */

    int     rsvd[8];
};
typedef struct pxvidopt pxvidopt_s;
#define PXMOS_VIDOPT	(PXMOS_DDCH+3+8)


/*
 * Which portion of frame buffer memory this state uses,
 * and partitions into pxvidres.framebuffers.
 * Currently unused.
 */
struct pxvidmem {
    struct  pxddch  ddch;

    pxapiadrs_t space[PXMAX_UNITS];
    pxapiadrs_t start[PXMAX_UNITS];
    pxapiadrs_t length[PXMAX_UNITS];	/* may be larger than memory size to indicate all   */
    pxapiadrs_t rsvd[PXMAX_UNITS];
};
typedef struct pxvidmem pxvidmem_s;
#define PXMOS_VIDMEM	(PXMOS_DDCH+4*PXMAX_UNITS)


/*
 * Data with which to set the camera parameters.
 * This is a raw data depository with minimal interpretation,
 * allowing camera parameters to be 'carried' along with the
 * other components of the video format.
 * Original version,and a version with more space for camera parameters.
 */
#define PXCAMCTRL_SERIAL    0x01    /* ascii text for RS-232 port	  */
#define PXCAMCTRL_I2C	    0x02    /* I2C address/data pairs		  */
#define PXCAMCTRL_REGVALUE  0x03    /* register values, base adrs assumed */
#define PXCAMCTRL_CAMLINK   0x04    /* ascii text for serial camera link  */
#define PXCAMCTRL_SPI	    0x05    /* SPI address/data pairs		  */

struct pxcamcntl {
    struct  pxddch  ddch;

    int     fgfamily;	    /* family code				*/
    int     fgmodel;	    /* family dependent model code		*/
    int     fgsubmodel;     /* family/model dependent submodel code	*/
    int     fgcustomid;     /* customer customization code, if any	*/
    char    fgcamera[80];   /* camera intended for, readable description*/
    int     rsvd1[16];

    int     cntlmode;	    /* PXCAMCTRL_* interpretation		*/
    int     cntlcntinit;    /* data[] used	      third  in data[]	*/
    int     cntlcntadj;     /* data[] used	      fourth in data[]	*/
    int     cntlcntdeinit;  /* data[] used	      fifth  in data[]	*/
    int     cntlparms[8];   /* variations to above			*/
    int     cntlcntxmteot;  /* data[] used	      first  in data[]	*/
    int     cntlcntrcveot;  /* data[] used	      second in data[]	*/
    int     cntloptions;    /* options to cntlmode			*/
    int     cntlcntreset;   /* data[] used	      zero'th in data[] */
    int     cntlrcvtout;    /* ackowledgment timeout			*/
    int     cntlrsttout;    /* reset pause/timeout			*/
    int     cntlretries;    /* retries					*/
    int     cntlpause;	    /* pause rcveot before xmt next command	*/
    int     cntlthrottle;   /* throttle xmt chars to this rate		*/
    int     cntladrs;	    /* varies					*/
    int     rsvd2[6];

    union {
	uint32	 data32[128];	/* regardless of intepretation, all info    */
				/* is imported/exported as 32 bit values    */
	uint16	 data16[256];
	uint8	 data8 [512];	/* future: larger?			    */
    } cntldata;
};
struct pxcamcntl2k {
    struct  pxddch  ddch;

    int     fgfamily;	    /* family code				*/
    int     fgmodel;	    /* family dependent model code		*/
    int     fgsubmodel;     /* family/model dependent submodel code	*/
    int     fgcustomid;     /* customer customization code, if any	*/
    char    fgcamera[80];   /* camera intended for, readable description*/
    int     rsvd1[16];

    int     cntlmode;	    /* PXCAMCTRL_* interpretation		*/
    int     cntlcntinit;    /* data[] used	      third  in data[]	*/
    int     cntlcntadj;     /* data[] used	      fourth in data[]	*/
    int     cntlcntdeinit;  /* data[] used	      fifth  in data[]	*/
    int     cntlparms[8];   /* variations to above			*/
    int     cntlcntxmteot;  /* data[] used	      first  in data[]	*/
    int     cntlcntrcveot;  /* data[] used	      second in data[]	*/
    int     cntloptions;    /* options to cntlmode			*/
    int     cntlcntreset;   /* data[] used	      zero'th in data[] */
    int     cntlrcvtout;    /* ackowledgment timeout			*/
    int     cntlrsttout;    /* reset pause/timeout			*/
    int     cntlretries;    /* retries					*/
    int     cntlpause;	    /* pause rcveot before xmt next command	*/
    int     cntlthrottle;   /* throttle xmt chars to this rate		*/
    int     cntladrs;	    /* varies					*/
    int     rsvd2[6];

    union {
	uint32	 data32[512];	/* regardless of intepretation, all info    */
				/* is imported/exported as 32 bit values    */
	uint16	 data16[1024];
	uint8	 data8 [2048];	/* future: larger?			    */
    } cntldata;
};
typedef struct pxcamcntl pxcamcntl_s;
#define PXMOS_CAMCNTL	 (PXMOS_DDCH+4+80+16+4+8+2+1+1+12+128)
#define PXMOS_CAMCNTL2K  (PXMOS_DDCH+4+80+16+4+8+2+1+1+12+512)


#include "cext_hpe.h"     
#endif				/* !defined(__EPIX_PXLIBVS_DEFINED) */
