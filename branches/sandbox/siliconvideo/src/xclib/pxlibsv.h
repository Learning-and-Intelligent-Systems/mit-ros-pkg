/*
 *
 *	pxlibsv.h	External	01-Jul-2008
 *
 *	Copyright (C)  1999-2007  EPIX, Inc.  All rights reserved.
 *
 *	Frame Grabber Library: API, Services, and Support Structs
 *
 */


#if !defined(__EPIX_PXLIBSV_DEFINED)
#define __EPIX_PXLIBSV_DEFINED
#include "cext_hps.h"     

#ifdef  __cplusplus
extern "C" {
#endif


/*
 * Types.
 *
 * The _farimap is intentionally an adjective rather than
 * a typedef, allowing safe type casting:
 *   xx = (ushort _farimap *) ..
 * without adding or loosing farness.
 *
 */
#if  defined(OS_WIN95)|defined(OS_WIN95_VXD)|defined(OS_WIN95_DLL) \
    |defined(OS_WINNT)|defined(OS_WINNT_KMD)|defined(OS_WINNT_DLL)|defined(OS_WINNT_WDM) \
    |defined(OS_WIN64)|defined(OS_WIN64_DLL)|defined(OS_WIN64_WDM)
  #define _farimap		    /* the farness of an imap pointer	*/
#elif defined(OS_LINUX_GNU)
  #define _farimap		    /* the farness of an imap pointer	*/
#else
  #define _farimap    _farphy	    /* the farness of an imap pointer	*/
#endif

/*
 * Abort predicate callback for functions with
 * potentially long or continuous execution times.
 */
#if !defined(__EPIX_PXABORTFUNC_DEFINED)
typedef int (_cfunfcc pxabortfunc_t)(void*,int,int);
#define __EPIX_PXABORTFUNC_DEFINED
#endif


/*
 * Limits supported in the library.
 * These are upper bounds, the hardware and/or
 * driver might have lower limits.
 */
#define PXMAX_CLIENTS	4	/* max open clients				*/
#define PXMAX_GPIN	16	/* max general purpose inputs			*/
#define PXMAX_IRBGF	14	/* worst case # of pximage structs required	*/
				/* by initPximage generator functions - not	*/
				/* the maximum number of images supported!	*/
#define PXMAX_FIRBGF	(4+PXMAX_IRBGF)
				/* worst case # of pximage structs required	*/
				/* by initFilteredPximage generator functions	*/
				/* - not the maximum number of images supported!*/



/*
 * Common static info extracted from board, or from driver.
 * For shared boards, the nclients is a non-static exception.
 */
struct pxdevinfo {
    struct  pxddch  ddch;

    int     family;	    /* board family code			    */
    int     model;	    /* board family dependent model code	    */
    int     submodel;	    /* board family/model dependent submodel code   */
    int     customid;	    /* customer customization code, if any	    */
    int     nunits;	    /* # of units controlled			    */
    int     physunitid;     /* physical sequential board #		    */
    int     nclients;	    /* control of units is shared by N clients	    */
    pxapiadrs_t memsize;    /* frame buffer memory size, per unit	    */
    pxapiadrs_t blksize;    /* future use				    */
    char    driverid[80];   /* driver id string 			    */
    char    libraryid[80];  /* library id string			    */
};
typedef struct pxdevinfo    pxdevinfo_s;
#define PXMOS_DEVINFO	    (PXMOS_DDCH+7+80+80)


/*
#define PXTIME_VIDEO	    0x0001
#define PXTIME_SYSTEM	    0x0002
 */


/*
 * A 'time' stamp or specification.
 * Two versions, one smaller for embedding in
 * other service structures.
 * Preliminary: the stamp vs specification aspects
 * may get broken into two distinct structures.
 */
struct pxtimestamp {
			    /* video time			    */
    pxvbtime_t	vcnt;	    /* v time				    */
    pxvbtime_t	hcnt;	    /* h time, if supported, else 0	    */

			    /* system time					*/
    uint32	ticks[2];   /* low & high tick count		    */
    uint32	ticku[2];   /* tick units, rational, microsec	    */

			    /* for time stamp, not specification    */
    uint16	field;	    /* current field #			    */

			    /* for time specification, not stamp    */
    uint16	fieldmod;   /* PXFIELD_*			    */
};
typedef struct pxtimestamp   pxtimestamp_s;
#define PXMOS_TIMESTAMP      (8)


struct pxtimespec {
    struct  pxddch  ddch;
    pxtimestamp_s   time;
};
typedef struct pxtimespec    pxtimespec_s;
#define PXMOS_TIMESPEC	     (PXMOS_DDCH+PXMOS_TIMESTAMP)


/*
 * Video status.
 */
struct pxvidstatus {
    struct  pxddch  ddch;

    /*
     * Video & system time when this status
     * was recorded (not the time at which
     * the status was requested).
     */
    pxtimestamp_s time;


    /*
     * The current or immediate future capture/display state.
     * This is interpreted relative to video rate events;
     * if this status was snapped in v-drive the capture/display
     * state is that of the upcoming field (which, as IRQ latency
     * is never 0, is quite appropriate as the status may have actually
     * been recorded within the following field).
     */
    pxbuffer_t	buffer;     /* frame buffer, or 0 if none		    */
    int 	stateid;    /* video state/format id, or 0 if none	    */
    int 	tracker;    /* internal use				    */

    /*
     * Tally of video errors.
     * Some may generate a pxdevfault, others may not.
     */
    uint32	tallylostvsync;  /* # times expected/waited/gaveup for lack of vsync	    */
    uint32	tallylosthsync;  /* # times expected/waited/gaveup for lack of hsync	    */
    uint32	tallylostfield;  /* # times expected/waited/gaveup for lack of field	    */
    uint32	tallylostdata;	 /* # times has lost data (fifo overflow, etc.) 	    */
    uint32	tallylostxfer;	 /* # times has lost data (bus errors, etc.)		    */
    uint32	tallylostcmnd;	 /* # times commands discarded (command queue full, etc.)   */
    uint32	tallylostlink;	 /* # times has lost communication link 		    */
    uint32	tallylatexfer;	 /* # times has delayed data (bus delays, etc.) 	    */
    uint32	rsvd[8];

    /*
     * Tally of vertical and horizontal video events, to the extent supported.
     * Future: Flags to indicate wether these are supported,
     * and whether they occur in a given video mode.
     */
    struct {
	pxvbtime_t  cntsoa;	/* start of active video		    */
	pxvbtime_t  cnteoa;	/* end of active video			    */
	pxvbtime_t  cnteoc;	/* end of captured video		    */
	pxvbtime_t  cntsoc;	/* start of captured video		    */
	pxvbtime_t  cntsvb;	/* start of v/h blanking		    */
	pxvbtime_t  cntevb;	/* end of v/h blanking			    */
	pxvbtime_t  cntsvd;	/* start of v/h drive			    */
	pxvbtime_t  cntevd;	/* end of v/h drive			    */
	int	    lastevent;	/* last item above changed at (a PXVID_*)   */
    } v, h;
    int     vcntevent;	    /* the time.vcnt is derived from .. (a PXVID_*) */
    int     hcntevent;	    /* the time.hcnt is derived from .. (a PXVID_*) */

    /*
     * Current tally and/or value of general purpose input.
     * Provided only for boards where the input
     * can be read with almost no extra latency,
     * allowing these to be monitored in conjunction
     * with video events.
     */
    pxvbtime_t	cntgpin[PXMAX_GPIN];	/* count of transitions, as/if detected by hardware */
    pxvbtime_t	valgpin[PXMAX_GPIN];	/* current values, as/if detected by hardware	    */

};
typedef struct pxvidstatus pxvidstatus_s;
#define PXMOS_VIDSTATUS (PXMOS_DDCH+PXMOS_TIMESTAMP+3+8+8+2*(8+1)+2+PXMAX_GPIN*2)


/*
 * Video event verbs.
 * Also used as IRQ 'which' flags.
 * Frame grabbers don't necessarily support all modes.
 *
 * In some contexts, it is useful to 'track' the general
 * purpose inputs (gpin) in a consistent and synchronized
 * manner as video events; thus their inclusion here.
 */
#define PXVID_SVD	0x00010 /* start of v/h drive			*/
#define PXVID_EVD	0x00020 /* end of v/h drive			*/
#define PXVID_EVB	0x00040 /* end of v/h blanking			*/
#define PXVID_SOA	0x00080 /* start of v/h active video		*/
#define PXVID_SOC	0x00100 /* start of v/h captured video		*/
#define PXVID_EOC	0x00200 /* end of v/h captured video		*/
#define PXVID_EOA	0x00400 /* end of v/h active video		*/
#define PXVID_SVB	0x00800 /* start of v/h blanking		*/
#define PXVID_VERT	0x00001 /* adjective: vertical events		*/
#define PXVID_HORZ	0x00002 /* adjective: horizontal events 	*/
#define PXVID_VSP	0x10000 /* verb for video switch point (v.s.p.) */
				/* the v.s.p. is synonymous with one of */
				/* the above PXVID_*, altho this define */
				/* is obviously not identical		*/
#define PXVID_GPIN	0x00004 /* adjective: gpin events		*/
#define PXVID_GPIN0	0x00010 /* gpin 0				*/
#define PXVID_GPIN1	0x00020 /* gpin 1				*/
#define PXVID_GPIN2	0x00040 /* gpin 2				*/
#define PXVID_GPIN3	0x00080 /* gpin 3				*/
#define PXVID_GPIN4	0x00100 /* gpin 4				*/
#define PXVID_GPIN5	0x00200 /* gpin 5				*/
#define PXVID_GPIN6	0x00400 /* gpin 6				*/
#define PXVID_GPIN7	0x00800 /* gpin 7				*/
#define PXVID_GPTRIG	0x00005 /* adjective: gptrig events		*/
#define PXVID_GPTRIG0	0x00010 /* gpin 0				*/
#define PXVID_GPTRIG1	0x00020 /* gpin 1				*/
#define PXVID_GPTRIG2	0x00040 /* gpin 2				*/
#define PXVID_GPTRIG3	0x00080 /* gpin 3				*/
#define PXVID_GPTRIG4	0x00100 /* gpin 4				*/
#define PXVID_GPTRIG5	0x00200 /* gpin 5				*/
#define PXVID_GPTRIG6	0x00400 /* gpin 6				*/
#define PXVID_GPTRIG7	0x00800 /* gpin 7				*/

#define PXSTAT_LASTEOC	0x01000 /* as of last e.o.c			*/
#define PXSTAT_LASTVSP	0x02000 /* as of last v.s.p			*/
#define PXSTAT_LASTIRQ	0x04000 /* as of last interrupt 		*/
#define PXSTAT_UPDATED	0x08000 /* updated now				*/

#define PXVID_FAULT	0x00FF6 /* fault activity, for use in ::setIrq	*/
#define PXVID_SERIAL	0x0F001 /* serial activity, for use in ::setIrq */


/*
 * Fault & error reporting
 *
 * The message text may have printf escapes, rather than
 * a completely formatted message; avoiding wasting time
 * for formatting when many faults are simply discarded.
 */
struct pxdevfault {
    struct  pxddch  ddch;

    pxtimestamp_s time; 	/* time fault was logged			*/

    int 	level;		/* fault code, below. 0 is no fault		*/
    int 	model;		/* model and submodel ..			*/
    int 	submodel;	/* .. information, board family specific	*/
    int 	unitmap;	/* bitmap of units, physical, on which fault	*/
				/* occurred, may be 0 if fault not		*/
				/* associated with one or more specific units	*/
    uint32	faultcnt;	/* tally of all faults generated		*/
    int 	mesgcode;	/* numeric error code - internal use		*/
    char	mesg[4][80];	/* printable messages, 4 lines			*/
    long	args[4][5];	/* if args[i][0]!=0 then mesg[i] is printf	*/
				/* plus args style, such as:			*/
				/*	printf(p->mesg[i],			*/
				/*	       p->args[i][1],			*/
				/*	       p->args[i][2],			*/
				/*	       p->args[i][3],			*/
				/*	       p->args[i][4]);			*/
				/* otherwise mesg[i] has no printf escapes	*/
};
typedef struct pxdevfault pxdevfault_s;
#define PXMOS_DEVFAULT	(PXMOS_DDCH+PXMOS_TIMESPEC+6+3*80+3*5)

/*
 * Fault levels. These aren't explanatory, but suggestive of action.
 */
#define PXFLT_WARNING	0x01	/* operation delay, timeout, recovery successful*/
#define PXFLT_FAILURE	0x02	/* operation failed, but might succeed in	*/
				/* .. different circumstances			*/
#define PXFLT_ABORT	0x04	/* unrecoverable, catastrophic, wrong hardware? */
				/* .. abort use of driver			*/
#define PXFLT_INIT	0x08	/* fault during open/initialization		*/
#define PXFLT_PLUGGED	0x10	/* hardware added, removed, powered down/up	*/



/*
 * Async specification and notification.
 */
typedef int (_cfunfcc pxasyncfunc_t)(void*,int,int);
struct pxasync {
    #if defined(OS_DOS4GW)
	pxasyncfunc_t	*funcp;
	void		*statep;
    #elif  defined(OS_WIN95)|defined(OS_WIN95_DLL)|defined(OS_WIN95_VXD) \
	  |defined(OS_WINNT)|defined(OS_WINNT_DLL)|defined(OS_WINNT_KMD)|defined(OS_WINNT_WDM) \
	  |defined(OS_WIN64)|defined(OS_WIN64_DLL)|defined(OS_WIN64_WDM)
	HANDLE	*hEvent;
    #elif defined(OS_LINUX_GNU)
	int	sig;
	void	*rsvd;
    #endif
};
typedef struct pxasync pxasync_s;


/*
 * An image buffer; not to be confused with a frame buffers!
 *
 * The former is typically in application space malloc'ed memory and
 * must be free'ed when done, and could, at least in principle,
 * be kept after the driver/libary is closed.
 *
 * The later is in the driver's frame buffer memory, (de)allocation is
 * the responsibility of the driver, and can't be kept after the
 * driver/libary is closed.
 */
struct pximagebuf;
typedef struct pximagebuf  pximagebuf_s;


/*
 * Triggered capture specification.
 * This is used when the general purpose (g.p.) inputs
 * and outputs are used to initiate and control
 * a 'triggered' capture from an otherwise untriggered
 * video source. It is not used with camera and frame grabber
 * combinations that have dedicated triggering hardware support.
 */
struct pxtrigspec
{
    struct  pxddch  ddch;

			    /* Phase 1: initial reset of g.p.in:	    */
    uint    gpin10mask;     /* g.p.in bit mask (for latching g.p.in)	    */

			    /* Phase 2: initial set of g.p.out		    */
    uint    gpout20value;   /* g.p.out bit values			    */
    uint    gpout20mask;    /* g.p.out bit mask 			    */
    uint    gpout20when;    /* 0: async to VB, 1: sync to VB		    */

			    /* Phase 3: wait for g.p.in change: 	    */
    uint    gpin30wait;     /* 0: no wait				    */
			    /* 1: wait for change as sampled at VB	    */
			    /* 3: wait for rising edge as sampled at VB     */
			    /* 5: wait for falling edge as sampled at VB    */
    uint    gpin30mask;     /* bit mask 				    */

			    /* Phase 4: notify g.p.in change:		    */
    uint    gpout40value;   /* g.p.out bit values			    */
    uint    gpout40mask;    /* g.p.out bit mask 			    */

			    /* Phase 5 & 6: optional trig control:	    */
    uint    option50;	    /* 0: ignore, 1: do 			    */
    uint    field50;	    /*	  at next PXFIELD_NXT/ODD/EVN field	    */
    uint    gpout50value;   /*	  .. g.p.out bit values 		    */
    uint    gpout50mask;    /*	  .. g.p.out bit mask			    */
    uint    delay60;	    /*	  wait N fields 			    */
    uint    gpout60value;   /*	  .. g.p.out bit values 		    */
    uint    gpout60mask;    /*	  .. g.p.out bit mask			    */

			    /* Phase 7: delay and capture:		    */
    uint    delay70;	    /* wait N fields				    */
    uint    field70;	    /* at next PXFIELD_NXT/ODD/EVN field	    */
    uint    capture70;	    /* 0: capture				    */

			    /* Phase 8: notify capture: 		    */
    uint    gpin80mask;     /* reset g.p.in bits (for latching bits)	    */
    uint    gpout80value;   /* g.p.out bit values			    */
    uint    gpout80mask;    /* g.p.out bit mask 			    */
};
#define PXMOS_TRIGSPEC	    (PXMOS_DDCH+21)
typedef struct pxtrigspec pxtrigspec_s;


/*
 * Triggered sequence capture specification.
 * This is used when the general purpose (g.p.) trigger inputs
 * and outputs are used to initiate and control
 * a 'triggered' sequence capture from an otherwise untriggered
 * video source. It is not used with camera and frame grabber
 * combinations that have dedicated triggering hardware support.
 */
struct pxtrigseqspec
{
    struct  pxddch  ddch;

			    /* Phase 1: Initial set of g.p.out: 	    */
    uint    gpout10value;   /* g.p.out bit values			    */
    uint    gpout10mask;    /* g.p.out bit mask 			    */

			    /* Phase 2.0: Trigger first image of sequence   */
    uint    trig20wait;     /* 0x000: no wait				    */
			    /* 0x100: wait for trigger			    */
    uint    trig20slct;     /* rsvd					    */
    pxvbtime_t trig20delay; /* rsvd					    */
			    /* Phase 2.5: Notify trigger observed:	    */
    uint    gpout25value;   /* g.p.out bit values			    */
    uint    gpout25mask;    /* g.p.out bit mask 			    */

			    /* Phase 3.0: Trigger next image of sequence    */
    uint    trig30wait;     /* 0x000: no wait				    */
			    /* 0x100: wait for trigger			    */
    uint    trig30slct;     /* rsvd					    */
    pxvbtime_t trig30delay; /* rsvd					    */
			    /* Phase 3.5: Notify trigger observed:	    */
    uint    gpout35value;   /* g.p.out bit values			    */
    uint    gpout35mask;    /* g.p.out bit mask 			    */

			    /* Phase 4.0: Trigger end of sequence	    */
    uint    trig40wait;     /* 0x000: no wait				    */
			    /* 0x100: wait for trigger			    */
    uint    trig40slct;     /* rsvd					    */
    pxvbtime_t trig40delay; /* rsvd					    */
			    /* Phase 4.5: Notify trigger observed:	    */
    uint    gpout45value;   /* g.p.out bit values			    */
    uint    gpout45mask;    /* g.p.out bit mask 			    */

			    /* Phase 5: Final set of g.p.out:		    */
    uint    gpout50value;   /* g.p.out bit values			    */
    uint    gpout50mask;    /* g.p.out bit mask 			    */

    uint    rsvd1;
    uint    rsvd2;
};
#define PXMOS_TRIGSEQSPEC   (PXMOS_DDCH+21)
typedef struct pxtrigseqspec pxtrigseqspec_s;


/*
 * Buffer capture status & stamp.
 */
struct pxbufstatus {
    struct  pxddch  ddch;

    int 	stateid;	/* video state/format id			    */
    int 	tracker;	/* internal use 				    */

    pxvbtime_t	captvcnt;	/* v time when captured 			    */

    uint32	captticks[2];	/* system time when captured: high/low tick count   */
    uint32	captticku[2];	/* tick units, rational, microsec - use in future?  */

    int 	captgpin;	/* G.P. In when captured. Only for boards where the */
				/* input can be read with almost no extra latency.  */
};
#define PXMOS_BUFSTATUS     (PXMOS_DDCH+6)
typedef struct pxbufstatus pxbufstatus_s;

/*
 * Buffer capture status & stamp.
 * For use in files.
 */
struct pxbufstatusfile {
    uint16	ddch_len;
    uint16	ddch_mos;
    uint32	captvcnt;	/* v time				*/
    uint32	captticks[2];	/* system time: high/low tick count	*/
    uint32	captticku[2];	/* tick units, rational, microsec	*/
    uint32	captgpin;	/* G.P. In when captured		*/
    char	rsvd[64-sizeof(uint16)*2-sizeof(uint32)*6];
};
#define PXMOS_BUFSTATUSFILE (2+1+2+2+1)
typedef struct pxbufstatusfile pxbufstatusfile_s;


/*
 * Common low level, but device independent, services.
 */
typedef struct pxdevservice pxdevservice_s;
struct pxdevservice {
    void    _farimap	*stuff;  /* internal stuff */

    /*
     * Get any posted faults.
     * Get current video status.
     * Get single value from video status, more efficiently than
     * retrieving the entire video status.
     * Set driver parms; "-configure" use only (future).
     * Get driver parms; "-configure" use only (future).
     * Get static (fixed) device info.
     */
    int    (_cfunfcc *getFault) 	(pxdevservice_s *me,int unitmap,int rsvd1,pxdevfault_s *fault);
    int    (_cfunfcc *getVidStatus)	(pxdevservice_s *me,int unitmap,int rsvd1,pxvidstatus_s *status,int mode);
    uint32 (_cfunfcc *getVidStatusValue)(pxdevservice_s *me,int unitmap,int rsvd1,int mode,size_t offset);
    int    (_cfunfcc *setDevParms)	(pxdevservice_s *me,int unitmap,int rsvd1,char *parms);
    int    (_cfunfcc *getDevParms)	(pxdevservice_s *me,int unitmap,int rsvd1,char *parms, size_t parmsize); // future
    int    (_cfunfcc *getDevInfo)	(pxdevservice_s *me,int unitmap,int rsvd1,pxdevinfo_s *info);

    /*
     * Frame buffer or other memory space direct access, if available.
     * Using the pximage services, either for 'copy to/from buffer'
     * access or for direct pointer access, provides a more device
     * independent API and is highly recommended.
     */
    int (_cfunfcc *getImap)	 (pxdevservice_s *me,int options,int unitmap,pxapiadrs_t adrs,uint altspace,void _farimap **accessp,pximaplen_t *accesslen);
    int (_cfunfcc *freeImap)	 (pxdevservice_s *me,int options,int unitmap,pxapiadrs_t adrs,uint altspace,void _farimap **accessp,pximaplen_t *accesslen);
    int (_cfunfcc *io)		 (pxdevservice_s *me,int options,int unitmap,pxapiadrs_t adrs,uint altspace,void *buf,size_t cnt,uint rw,pxasync_s *async,pxtimespec_s *time);
    int (_cfunfcc *iosparse)	 (pxdevservice_s *me,int options,int unitmap,pxapiadrs_t adrs,uint altspace,void *buf,size_t cnt,uint rw,pxasync_s *async,pxtimespec_s *time,size_t blksize,size_t blkpitch);

    /*
     * Misc control of misc auxilliary features.
     */
    int (_cfunfcc *ioctl)	 (pxdevservice_s *me,int options,int unitmap,uint32 adrs,uint space,uint subspace,void *buf,size_t cnt,uint rw,pxasync_s *async,pxtimespec_s *time);

    /*
     * Set polling interval from which psuedo
     * interrupts are (may be) derived. Future.
     */
    int (_cfunfcc *setPolling)	 (pxdevservice_s *me,int options,int unitmap,pxtimespec_s *time);

    /*
     * Interrupt notifications or callbacks
     */
    int (_cfunfcc *setIRQ)	 (pxdevservice_s *me,int options,int unitmap,pxasync_s *async,int irq,int select);
    int (_cfunfcc *clrIRQ)	 (pxdevservice_s *me,int options,int unitmap,pxasync_s *async,int irq,int select);

    /*
     * Future expansion
     */
    int (_cfunfcc *rsvd1)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd2)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd3)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd4)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd5)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd6)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd7)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd8)	 (pxdevservice_s *me);

    /*
     * Internal use.
     */
    pxdevfault_s    fault;
};
#define PXMOS_DEVSERVICE    (1+6+4+1+1+2+8+PXMOS_DEVFAULT)



/*
 * Common high level, device independent, services
 */
typedef struct pxlibservice pxlibservice_s;
struct pxlibservice {
    void    _farimap	*stuff;     /* internal stuff	*/

    /*
     * Image frame buffer access.
     *
     * These provide 'live' access to the frame buffers, in that
     * after using initPximage (or related functions),
     * each read/write via the struct pximage created
     * directly and repeatedly accesses data from the actual
     * frame buffer. If the frame buffer is being captured into,
     * repeated read's will return different data.
     *
     * For some frame grabbers, the 'altspace' allows
     * access via the pximage API to timing information captured
     * with the buffers, or access to input/output lookup tables,
     * or other non pixel information.
     *
     * The use and function of the pximage is identical to the
     * 1990's era driver; however the old driver was limited to a
     * single struct pximage to reference its frame buffers.
     * The new version provides for an array of struct pximage;
     * allowing support of more formats (altho 99% will still require
     * only a single pximage element).
     *
     * The initPximage* provide efficient access in the frame buffer's
     * 'natural' format; you may ask for a specific color format, but may
     * receive a different color format.
     *
     * The initFilteredPximage* will provide the color format requested,
     * but may do so by (transparently) adding additional processing
     * to the pixel data. For color formats, they also allow requesting
     * access to one pixie of the pixel; such a pximage can be passed
     * to a monochrome-only PXIPL function.
     *
     * None of these ever affect the board's settings; there is
     * no conflict if two pximage's are created for the same frame buffer
     * using different color formats.
     *
     * See pximage.h for the API presented by a pximage.
     */
    int (_cfunfcc *initPximage) 	(pxlibservice_s *me,int unitmap,pximage_s  *ip,int ipcnt,int colorhint,int altspace,int stateid,pxbuffer_t buffer,int mbpcihint);
    int (_cfunfcc *initPximage3)	(pxlibservice_s *me,int unitmap,pximage3_s *ip,int ipcnt,int colorhint,int altspace,int stateid,pxbuffer_t unused,int mbpcihint);
    int (_cfunfcc *initFilteredPximage) (pxlibservice_s *me,int unitmap,pximage_s  *ip,int ipcnt,int colorhint,int altspace,int stateid,pxbuffer_t buffer,int mbpcihint,int colormap);
    int (_cfunfcc *initFilteredPximage3)(pxlibservice_s *me,int unitmap,pximage3_s *ip,int ipcnt,int colorhint,int altspace,int stateid,pxbuffer_t unused,int mbpcihint,int colormap);

    /*
     * Image frame buffer access.
     *
     * These provide a frozen copy of the frame buffer,
     * with services for refreshing the copy from the 'live'
     * frame buffer, or writing the copy into the frame buffer.
     */
    int (_cfunfcc *makePximageBuf)  (pxlibservice_s *me,int options,int unitmap,pximagebuf_s **ipbuf,pxbuffer_t rsvd,int colorhint,int altspace,int stateid,int mbpcihint,int colormap);
    int (_cfunfcc *freePximageBuf)  (pxlibservice_s *me,int options,int unitmap,pximagebuf_s **ipbuf,pxbuffer_t rsvd,int colorhint,int altspace,int stateid,int mbpcihint,int colormap);
    int (_cfunfcc *loadPximageBuf)  (pxlibservice_s *me,int options,int unitmap,pximagebuf_s  *ipbuf,pxbuffer_t buffer,int colorhint,int altspace,int stateid,int mbpcihint,int colormap);
    int (_cfunfcc *savePximageBuf)  (pxlibservice_s *me,int options,int unitmap,pximagebuf_s  *ipbuf,pxbuffer_t buffer,int colorhint,int altspace,int stateid,int mbpcihint,int colormap);


    /*
     * Video state management.
     * The stateid is usually assigned by user, must be > 0.
     * Defining a state doesn't force video to that state.
     */
    int (_cfunfcc *defineState)  (pxlibservice_s *me,int options,int stateid,pxvidstate_s *statep);
    int (_cfunfcc *deleteState)  (pxlibservice_s *me,int options,int stateid,pxvidstate_s *statep);   // statep is ignored
    int (_cfunfcc *getState)	 (pxlibservice_s *me,int options,int stateid,pxvidstate_s *statep);

    /*
     * Video state utilities.
     * Available in "-offline" mode, they never have
     * direct effect on library's or hardware's state.
     */
    int (_cfunfcc *importStateCopy)   (pxlibservice_s *me,int options,int stateid,pxvidstate_s *statep,int stateindex,char *pathname,char *filemode);
    int (_cfunfcc *exportStateCopy)   (pxlibservice_s *me,int options,int stateid,pxvidstate_s *statep,int stateindex,char *pathname,char *filemode,char *progname,char *filecomment);
    int (_cfunfcc *allocStateCopy)    (pxlibservice_s *me,int options,int stateid,pxvidstate_s **statep);    // stateid ignored!
    int (_cfunfcc *freeStateCopy)     (pxlibservice_s *me,int options,int stateid,pxvidstate_s **statep);    // stateid ignored!
    int (_cfunfcc *initStateCopy)     (pxlibservice_s *me,int options,int stateid,pxvidstate_s *statep,pxdevinfo_s *infop,char *formatname,int vidmode); // stateid ignored!
    int (_cfunfcc *compareStateCopy)  (pxlibservice_s *me,int options,int stateid,pxvidstate_s *statep,pxvidstate_s *state2p);	// stateid ignored!
  //int (_cfunfcc *fixxStateCopy)     (pxlibservice_s *me,int options,int stateid,pxvidstate_s *statep,pxdevinfo_s *infop);	// stateid ignored!
    int (_cfunfcc *copyStateCopy)     (pxlibservice_s *me,int options,int stateid,pxvidstate_s *statep,pxvidstate_s *state2p);	// stateid ignored!

    /*
     * Continuous video mode commands.
     * Returns when the specified state take effect (top of field)
     * unless, of course, an async or timed option is used.
     *
     * Future.
     */
    int (_cfunfcc *goState)  (pxlibservice_s *me,int options,int unitmap,int stateid,pxasync_s *async,pxtimespec_s *time);
    int (_cfunfcc *goBuf)    (pxlibservice_s *me,int options,int unitmap,int stateid,pxasync_s *async,pxtimespec_s *time,pxbuffer_t buf);
    int (_cfunfcc *goMode)   (pxlibservice_s *me,int options,int unitmap,int stateid,pxasync_s *async,pxtimespec_s *time,pxvidmode_s *vidmode);

    /*
     * Continuous video mode status.
     *
     * Get current video state.
     * Get current video status.
     * Get current VB time.
     * Get video status at last buffer captured.
     *
     * Future.
     */
    int 	(_cfunfcc *goingState)	     (pxlibservice_s *me,int options,int unitmap,pxtimespec_s *time);
    int 	(_cfunfcc *goingVidStatus)   (pxlibservice_s *me,int options,int unitmap,pxvidstatus_s *status);
    pxvbtime_t	(_cfunfcc *goingVBTime)      (pxlibservice_s *me,int options,int unitmap);
    int 	(_cfunfcc *goingBufStatus)   (pxlibservice_s *me,int options,int unitmap,pxbuffer_t buffer,pxbufstatus_s *status);

    /*
     * Snap Video Commands.
     *
     * Versions with explicit pxvidstate_s are particularly
     * useful when the library is being shared amongst several processes
     * as it allows acquiring without dependeding on a stored state
     * remaining unchanged.
     *
     * Future.
     */
    int (_cfunfcc *snapState)  (pxlibservice_s *me,int options,int stateid,pxasync_s *async,pxtimespec_s *time);
    int (_cfunfcc *snapBuf)    (pxlibservice_s *me,int options,int stateid,pxasync_s *async,pxtimespec_s *time,pxbuffer_t buf);
    int (_cfunfcc *snapMode)   (pxlibservice_s *me,int options,int stateid,pxasync_s *async,pxtimespec_s *time,pxvidmode_s *vidmode);
    int (_cfunfcc *snepState)  (pxlibservice_s *me,int options,pxvidstate_s *statep,pxasync_s *async,pxtimespec_s *time);
    int (_cfunfcc *snepBuf)    (pxlibservice_s *me,int options,pxvidstate_s *statep,pxasync_s *async,pxtimespec_s *time,pxbuffer_t buf);
    int (_cfunfcc *snepMode)   (pxlibservice_s *me,int options,pxvidstate_s *statep,pxasync_s *async,pxtimespec_s *time,pxvidmode_s *vidmode);

    /*
     * Snap into a new image buffer; typcially a frame
     * buffer must also be used or reused to implement these.
     *
     * Future.
     */
    int (_cfunfcc *snabState)  (pxlibservice_s *me, int options, pximagebuf_s *imbuf, int stateid, pxasync_s *async, pxtimespec_s *time);
    int (_cfunfcc *snabBuf)    (pxlibservice_s *me, int options, pximagebuf_s *imbuf, int stateid, pxasync_s *async, pxtimespec_s *time, pxbuffer_t buf);
    int (_cfunfcc *snabMode)   (pxlibservice_s *me, int options, pximagebuf_s *imbuf, int stateid, pxasync_s *async, pxtimespec_s *time, pxvidmode_s *vidmode);
    int (_cfunfcc *snebState)  (pxlibservice_s *me, int options, pximagebuf_s *imbuf, pxvidstate_s *statep, pxasync_s *async, pxtimespec_s *time);
    int (_cfunfcc *snebBuf)    (pxlibservice_s *me, int options, pximagebuf_s *imbuf, pxvidstate_s *statep, pxasync_s *async, pxtimespec_s *time, pxbuffer_t buf);
    int (_cfunfcc *snebMode)   (pxlibservice_s *me, int options, pximagebuf_s *imbuf, pxvidstate_s *statep, pxasync_s *async, pxtimespec_s *time, pxvidmode_s *vidmode);

    /*
     * Future expansion
     */
    int (_cfunfcc *rsvd1)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd2)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd3)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd4)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd5)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd6)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd7)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd8)	 (pxdevservice_s *me);
};
#define PXMOS_LIBSERVICE    (1+4+4+3+7+3+4+6+6+8)


/*
 * Auxilliary services.
 */
typedef struct pxauxservice pxauxservice_s;
struct pxauxservice {
    void    _farimap	*stuff;     /* internal stuff	*/

    /*
     * Frame buffer access.
     * These are simple and easy to use, although the
     * initPximage offers more options and is more efficient
     * if reading/writing small number of pixels repeatedly.
     *
     * Future: Rename these and/or clarify that they operate on 8 bit,
     * 16 bit, and 32 bit words instead of C data types.
     */
    int (_cfunfcc *imageReadUChar)  (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,
				     uchar *membuf,size_t cnt,int colorspace,int colormap,int iomode,int iodata);
    int (_cfunfcc *imageWriteUChar) (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,
				     uchar  *membuf,size_t cnt,int colorspace,int colormap,int iomode,int iodata);
    int (_cfunfcc *imageReadUShort) (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,
				     ushort *membuf,size_t cnt,int colorspace,int colormap,int iomode,int iodata);
    int (_cfunfcc *imageWriteUShort)(pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,
				     ushort *membuf,size_t cnt,int colorspace,int colormap,int iomode,int iodata);
    int (_cfunfcc *imageReadUInt)   (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,
				     uint *membuf,size_t cnt,int colorspace,int colormap,int iomode,int iodata);
    int (_cfunfcc *imageWriteUInt)  (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,
				     uint *membuf,size_t cnt,int colorspace,int colormap,int iomode,int iodata);


    /*
     * Image load & save.
     * These are simple and easy to use, altho the
     * PXIPL features, which use a pximage
     * created by initPximage, offer more options
     * and file formats.
     */
    int (_cfunfcc *imageLoadBmp)    (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *pathname,int loadmode);
    int (_cfunfcc *imageSaveBmp)    (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *pathname,int savemode);
    int (_cfunfcc *imageSaveTga)    (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *pathname,int savemode);
    int (_cfunfcc *imageSavePcx)    (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *pathname,int savemode);
    int (_cfunfcc *imageLoadTiff)   (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *pathname,int loadmode);
    int (_cfunfcc *imageSaveTiff)   (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *pathname,int savemode);

    /*
     * Frame buffer load & save, as a byte by byte copy.
     * Fast, but not portable to other applications
     * (or even to other EPIX(r) imaging cards)
     * and not intended for image archival.
     */
    int (_cfunfcc *bufferSaveStream) (pxauxservice_s *me,pxabortfunc_t**,int options,int unitmap,int stateid,pxbuffer_t startbuf,pxbuffer_t endbuf,char *pathname,void *filehandle,pxbuffer_t fileoffset,uint32 alignsector, uint32 rsvd2, uint32 rsvd3);
    int (_cfunfcc *bufferLoadStream) (pxauxservice_s *me,pxabortfunc_t**,int options,int unitmap,int stateid,pxbuffer_t startbuf,pxbuffer_t endbuf,char *pathname,void *filehandle,pxbuffer_t fileoffset,uint32 alignsector, uint32 rsvd2, uint32 rsvd3);
    int (_cfunfcc *bufferSaveStreamInit) (pxauxservice_s *me,pxabortfunc_t**,int options,int unitmap,int stateid,pxbuffer_t startbuf,pxbuffer_t endbuf,char *pathname,void *filehandle,pxbuffer_t fileoffset,uint32 alignsector);   // future?
    int (_cfunfcc *bufferSaveStreamAdd)  (pxauxservice_s *me,pxabortfunc_t**,int options,int unitmap,int stateid,pxbuffer_t startbuf,pxbuffer_t endbuf,char *pathname,void *filehandle,pxbuffer_t fileoffset,uint32 alignsector);   // future?
    int (_cfunfcc *bufferSaveStreamDone) (pxauxservice_s *me,pxabortfunc_t**,int options,int unitmap,int stateid,pxbuffer_t startbuf,pxbuffer_t endbuf,char *pathname,void *filehandle,pxbuffer_t fileoffset,uint32 alignsector);   // future?

    /*
     * OS specific features.
     */
    #if  defined(OS_WIN3X)|defined(OS_WIN3X_DLL) \
	|defined(OS_WIN95)|defined(OS_WIN95_DLL) \
	|defined(OS_WINNT)|defined(OS_WINNT_DLL) \
	|defined(OS_WIN64)|defined(OS_WIN64_DLL)

	/*
	 * Make a copy of a frame buffer as a D.I.B.
	 * in global memory and free same. D.I.B.'s
	 * are always 8 bits per pixel component.
	 */
	int (_cfunfcc *makeDIB)     (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,HANDLE *hDIB);
	int (_cfunfcc *freeDIB)     (pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,HANDLE *hDIB);

	/*
	 * Display frame buffer, once, on an hDC using the
	 * Windows StretchDiBits. This is simple and easy to use,
	 * altho the PXIPL features, which use a pximage
	 * created by initPximage, offer more options and speed.
	 */
	int (_cfunfcc *StretchDIBits)	(pxauxservice_s *me,int options,int unitmap,int stateid,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,
					 int winopts,HDC hDC,uint nX,uint nY,uint nWidth,uint nHeight);

	/*
	 * Check for any fault via getFault, or use explicitly
	 * provided fault, and show in a messagebox.
	 */
	int (_cfunfcc *faultMessageBox) (pxauxservice_s *me,int options,int unitmap,pxdevfault_s *faultp,HWND hWnd,LPCSTR lpCaption,UINT uType);

	/*
	 * Check for any fault via getFault, or use explicitly provided fault,
	 * and show via the FILE* (here declared as struct _iobuf *),
	 * typically stderr or stdout.
	 */
	int (_cfunfcc *faultPrintf)	(pxauxservice_s *me,int options,int unitmap,pxdevfault_s *faultp,struct _iobuf *stdiop,char *head,char *tail);
    #else

	/*
	 * Check for any fault via getFault, or use explicitly provided fault,
	 * and show via the FILE* (here declared as void *),
	 * typically stderr or stdout.
	 */
	int (_cfunfcc *faultPrintf)	(pxauxservice_s *me,int options,int unitmap,pxdevfault_s *faultp,void *stdiop,char *head,char *tail);
    #endif

    /*
     * Translate a PXER* error code to string.
     * As the returned string is static, and need not be freed,
     * using a char* return convention (rather than filling the caller's
     * buffer convention) allows simpler, inline, code.
     */
    char* (_cfunfcc *errorCodeString) (pxauxservice_s *me,int errorcode);


    /*
     * Internal use.
     */
    pxdevfault_s    fault;

    /*
     * Future expansion
     */
    int (_cfunfcc *rsvd1)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd2)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd3)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd4)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd5)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd6)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd7)	 (pxdevservice_s *me);
    int (_cfunfcc *rsvd8)	 (pxdevservice_s *me);
};
#define PXMOS_AUXSERVICE    (1+6+6+5+(2+1+1+3)+1+1+8)


#ifdef  __cplusplus
}
#endif

#include "cext_hpe.h"     
#endif				/* !defined(__EPIX_PXLIBSV_DEFINED) */
