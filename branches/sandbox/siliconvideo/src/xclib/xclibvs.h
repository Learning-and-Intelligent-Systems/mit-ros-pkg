/*
 *
 *	xclibvs.h	External	03-Jun-2009
 *
 *	Copyright (C)  1999-2009  EPIX, Inc.  All rights reserved.
 *
 *	PIXCI(R) Library: Video State (Format) Definitions
 *
 *	Most end-users will not modify these structures directly,
 *	but either use the default or change and export the
 *	default with SVIP/4MIP/XCIP/XCAP and import the new
 *	structures into the frame grabber library.
 *
 */


#if !defined(__EPIX_XCLIBVS_DEFINED)
#define __EPIX_XCLIBVS_DEFINED
#include "cext_hps.h"     


/*
 * For PIXCI(R) SV2/SV3/SV4/SV5/SV6 imaging boards.
 * Familiarity with the underlying Bt819/Bt829/Bt848/Bt878/Cx2388x
 * chips is implied.
 */
struct xcsv2format {
    struct  pxddch  ddch;


    uint8   Bt8x9format;    /* video format code			    */
			    /* All:   0 derived from pxvidformat	    */
			    /* Bt819: 1 NTSC, 3 PAL(B,D,G,H,I)		    */
			    /*						    */
			    /* Bt829: 1 NTSC, 3 PAL(B,D,G,H,I)		    */
			    /*	      4 PAL(M), 5 PAL(N), 6 SECAM	    */
			    /* Bt848: 1 NTSC, 2 NTSC Japan,		    */
			    /*	      3 PAL(B,D,G,H,I)			    */
			    /*	      4 PAL(M), 5 PAL(N), 6 SECAM	    */
			    /* Bt878: 1 NTSC, 2 NTSC Japan,		    */
			    /*	      3 PAL(B,D,G,H,I)			    */
			    /*	      4 PAL(M), 5 PAL(N), 6 SECAM	    */
			    /*	      7 PAL(N Combo)			    */
			    /* Cx2388x: 1 NTSC-M, 2 NTSC-J, 3 PAL-BDGHI     */
			    /*		4 PAL-M, 5 PAL-N, 7 PAL-Nc,	    */
			    /*		8 PAL 60, 6 SECAM		    */

    uint8   Bt8x9xtal;	    /* 1: Xtal 1, 2: Xtal 2			    */
    uint8   Bt8x9control;   /* bits 0b11110000 for CONTROL register	    */
			    /* also used for Bt848, Bt878, Cx2388x	    */

    uint8   Bt819oform;     /* bits 0b11100000 for OFORM register	    */
    uint8   Bt819vscale;    /* bits 0b11100000 for VSCALE_HI register	    */
    uint8   Bt819adc;	    /* bits 0b11110110 for ADC register 	    */

    uint8   Bt829oform;     /* bits 0b11100000 for OFORM register	    */
			    /* also used for Bt848, Bt878, Cx2388x	    */
    uint8   Bt829vscale;    /* bits 0b11100000 for VSCALE_HI register	    */
			    /* also used for Bt848, Bt878, Cx2388x	    */
			    /* for Bt878: bits 0x03 refine use of bit 0x80  */
    uint8   Bt829adc;	    /* bits 0b00110110 for ADC register 	    */
    uint8   Bt829vtc;	    /* bits 0b00000011 for VTC register 	    */
			    /* also used for Bt848, Bt878, Cx2388x	    */

    uint8   Bt829scloop;    /* bits 0b01100000 for SCLOOP register	    */
			    /* also used for Bt848, Bt878, Cx2388x	    */
    uint8   Bt848adc;	    /* bits 0b00110111 for ADC register 	    */
			    /* also used for Cx2388x			    */
    uint8   Bt848clrctl;    /* bits 0b01010000 for COLOR CONTROL register   */
			    /* also used for Cx2388x			    */

    uint8   Bt819options;   /* Bit 0: monochrome: underlay color in same    */
			    /* image buffer, reducing image buffer size, but*/
			    /* buffer should not be examined during capture */
			    /* Bt819 and Bt829 only.			    */

    uint32  Bt848faultmask; /* INT_STAT bits that should generate faults    */
    uint32  Bt848haltmask;  /* INT_STAT bits that should terminate video    */
			    /* Some error conditions may always fault/halt  */
			    /* For Bt848, Bt878, Cx2388x		    */

    uint8   rsvd[8];
};
typedef struct xcsv2format  xcsv2format_s;
#define XCMOS_SV2FORMAT     (PXMOS_DDCH+14+2+8)


/*
 * For PIXCI(R) SV7 imaging boards.
 * Must be identical to xcsv2format, alebit with different m.o.s. names
 * Familiarity with the underlying MAX9526 chip is implied.
 */
struct xcsv7format {
    struct  pxddch  ddch;

    uint8   Max9526format;	/* video format code			*/
				/* t.b.d.				*/
    uint8   Max9526standard;	/* t.b.d.				*/
    uint8   Max9526videoin;	/* t.b.d.				*/
    uint8   Max9526gainctrl;	/* t.b.d.				*/
    uint8   Max9526colorkill;	/* t.b.d.				*/
    uint8   Max9526test;	/* t.b.d.				*/
    uint8   Max9526clock;	/* t.b.d.				*/
    uint8   Max9526pll; 	/* t.b.d.				*/
    uint8   Max9526misc;	/* t.b.d.				*/

    uint8   rsvd1[5];

    uint32  rsvd2[2];

    uint8   rsvd3[8];
};
typedef struct xcsv7format  xcsv7format_s;
#define XCMOS_SV7FORMAT     (PXMOS_DDCH+9+5+2+8)



/*
 * For PIXCI(R) SV2/SV3/SV4/SV5 imaging boards.
 * Familiarity with the underlying Bt819/Bt829/Bt848/Bt878 chips is implied.
 */
struct xcsv2mode {
    struct  pxddch  ddch;

    sint8   brightness;     /* add to luma channel, -128-+127, 0 default    */
    sint8   hue;	    /* add to demod subcarrier phase, -128-+127,    */
			    /* in units of .7 degrees, 0 default	    */
    uint16  lumagain;	    /* contrast: multiply w. luma channel, 0-511, 216 default */
    uint16  chromaUgain;    /* multiply w. U component, 0-511, 254 default  */
    uint16  chromaVgain;    /* multiply w. V component, 0-511, 180 default  */
    uint16  rsvd[8];
};
typedef struct xcsv2mode    xcsv2mode_s;
#define XCMOS_SV2MODE	    (PXMOS_DDCH+5+8)


/*
 * For PIXCI(R) A/CL3SD/D/D24/D32/D2X/CL1/CL2/E1/E4/EB1POCL/EC1/ECB1/ECB134/ECB2/EL1/E1DB/EL1DB/ELS2/E4DB/SI/SI1/SI2/SI4 imaging boards.
 * Familiarity with the underlying imaging board documentation is implied.
 */
struct xcdxxformat {
    struct  pxddch  ddch;

    uint16  PRINC;	/* PRINC bits	   (formerly initialargs[0]&0xFFFF)	    */
    uint16  EXSYNC;	/* EXSYNC bits	   (formerly initialargs[1]&0xFFFF)	    */
    uint16  exsync;	/*		   (formerly (initialargs[1]>>16)&0xFFFF    */
    uint16  prin;	/*		   (formerly (initialargs[0]>>16)&0xFFFF    */
    uint16  MWETS;	/* some MWETS bits (formerly initialargs[2])		    */
			/* (2LSBF)						    */
    uint16  LCD;	/* some LCD bits   (formerly initialargs[3])		    */
			/* (VDEND)						    */

    /*
     * Additions for PIXCI(R) A
     */
    uint16  PLLMX;		/* PLLMX bits (formerly initialargs[4]) 	    */
    uint16  clampreference;	/*	      (formerly initialargs[5]&0xFFFF	    */
    uint16  pixelclkreference;	/*	      (formerly (initialargs[5]>>16)&0xFFFF */

    /*
     * Additions for PIXCI(R) CL3SD
     */
    sint32  sdclkfreq[2];	/* SDRAM clock frequency, MHz, rational 	    */
    uint32  SDIET;		/* option bits					    */

    /*
     * Additions for D2X
     */
    uint16  PCWD;	/* some  PCWD bits					    */

    /*
     * Additions for CL2, E1, E4, EB1, EB1POCL, EC1, ECB1, ECB1-34, ECB2, EL1, E1DB, E4DB, EL1DB
     */
    uint16  CLCCSE;	/* some CLCCSE bits					    */
    uint16  TEST;	/* some VIDEOTEST bits					    */
    uint16  CLCCSEhi;	/* some CLCCSE high bits				    */
    uint16  MSSMUB;	/* some MSSMUB bits					    */

    /*
     * Additions for PIXCI(R) D w. Roper (Kodak) MASD Toucan.
     * Requires familiarity with Toucan documentation.
     * Should be 0 if Toucan not used.
     */
    uint32  ToucanOpt;		/* 00 thru 06: Black Setup		    */
				/* 07 thru 08: Toucan.dll rslt: BlueGain    */
				/* 09 thru 10: sharp gain		    */
				/* 11 thru 11: med edge only		    */
				/* 12 thru 12: detail only		    */
				/* 13 thru 13: clip on 2		    */
				/* 15 thru 15: one to enable loading Toucan */
				/* 16 thru 17: Bayer phase		    */
				/* (formerly initialargs[12])		    */
    uint32  ToucanCoef1;	/* Toucan.dll rslt: CCMRow[0]		    */
    uint32  ToucanCoef2;	/* Toucan.dll rslt: CCMRow[1]		    */
    uint32  ToucanCoef3;	/* Toucan.dll rslt: CCMRow[2]		    */
				/*   Col A: bits  0-9 : SNNN.DDDDDD (2's comp fract) */
				/*   Col B: bits 10-19: SNNN.DDDDDD (2's comp fract) */
				/*   Col C: bits 20-29: SNNN.DDDDDD (2's comp fract) */
				/* (formerly initialargs[13-15])		     */


    /*
     * Info
     */
    uint32  exsyncinfo[4];	/* min value, max value, scaling (rational, microsec)	*/
				/* formerly initialargs[8], [9], [10], [11]		*/
    uint32  prininfo[4];	/* min value, max value, scaling (rational, microsec)	*/

    /*
     * Additions for various custom boards
     */
    uint16  XXX;

    /*
     * Additions for CL2, E1, E4, EB1, EB1POCL, EC1, ECB1, ECB1-34, ECB2, EL1, E1DB, E4DB, EL1DB
     */
    uint16  CLCCGP;	    /* enable/disable CLCC control: video state vs. g.p.    */


    uint16  MSSMUBhi;	    /* some MSSMUB high bits				    */
    uint16  rsvd2[5];
};
typedef struct xcdxxformat xcdxxformat_s;
#define XCMOS_DXXFORMAT    (PXMOS_DDCH+6+3+8 +4 +2*4 +1+1+6)



#include "cext_hpe.h"     
#endif				/* !defined(__EPIX_XCLIBVS_DEFINED) */
