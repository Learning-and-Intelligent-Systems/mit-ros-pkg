/*
 *
 *	xclibsc.h	External	16-Jan-2011
 *
 *	Copyright (C)  1999-2011  EPIX, Inc.  All rights reserved.
 *
 *	Frame Grabber Library: Simple, 'C' function oriented, interface.
 *
 */


#if !defined(__EPIX_XCLIBSC_DEFINED)
#define __EPIX_XCLIBSC_DEFINED
#include "cext_hps.h"     

#ifdef  __cplusplus
extern "C" {
#endif

/*
 * Open/close/faults
 */
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_PIXCIopen(char *driverparms, char *formatname, char *formatfile);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_PIXCIclose();
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_mesgFault(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,char*)     pxd_mesgErrorCode(int err);
#if defined(CTOBAS)  // alternate declaration for VB/CTOBAS
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_mesgFaultText(int unitmap, char buf[], size_t bufsize);
#endif
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_mesgFaultText(int unitmap, char *buf, size_t bufsize);

struct pxdstate;
typedef struct pxdstate pxdstate_s;

_cDcl(_dllpxlib,_cfunfcc,pxdstate_s*)	pxe_XCLIBinstantiate(void);
_cDcl(_dllpxlib,_cfunfcc,void)		pxe_XCLIBuninstantiate(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_PIXCIopen(pxdstate_s*, char *driverparms, char *formatname, char *formatfile);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_PIXCIclose(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_mesgFault(pxdstate_s*, int unitmap);
#if defined(CTOBAS)  // alternate declaration for VB/CTOBAS
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_mesgFaultText(pxdstate_s*, int unitmap, char buf[], size_t bufsize);
#endif
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_mesgFaultText(pxdstate_s*, int unitmap, char *buf, size_t bufsize);

/*
 * Board/library/driver info
 */
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_infoModel(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_infoSubmodel(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_infoUnits();
_cDcl(_dllpxlib,_cfunfcc,char*) 	pxd_infoDriverId();
_cDcl(_dllpxlib,_cfunfcc,char*) 	pxd_infoLibraryId();
#define 				pxd_infoIncludeId() XCLIB_IDNVR
#if defined(OS_WIN64) | defined(OS_WIN64_DLL) | defined(C_GNU64)
_cDcl(_dllpxlib,_cfunfcc,uint64)	pxd_infoMemsize(int unitmap);
#else
_cDcl(_dllpxlib,_cfunfcc,ulong) 	pxd_infoMemsize(int unitmap);
#endif

_cDcl(_dllpxlib,_cfunfcc,int)		pxe_infoModel(pxdstate_s*, int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_infoSubmodel(pxdstate_s*, int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_infoUnits(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,char*) 	pxe_infoDriverId(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,char*) 	pxe_infoLibraryId(pxdstate_s*);
#if defined(OS_WIN64) | defined(OS_WIN64_DLL) | defined(C_GNU64)
_cDcl(_dllpxlib,_cfunfcc,uint64)	pxe_infoMemsize(pxdstate_s*, int unitmap);
#else
_cDcl(_dllpxlib,_cfunfcc,ulong) 	pxe_infoMemsize(pxdstate_s*, int unitmap);
#endif

/*
 * Image info
 */
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageXdim();
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageYdim();
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageCdim();
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageBdim();
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageZdim();
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageIdim();
_cDcl(_dllpxlib,_cfunfcc,double)    pxd_imageAspectRatio();
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageXdims(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageYdims(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageCdims(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageBdims(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageZdims(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_imageIdims(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)    pxd_imageAspectRatios(int unitmap);

_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageXdim(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageYdim(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageCdim(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageBdim(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageZdim(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageIdim(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,double)    pxe_imageAspectRatio(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageXdims(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageYdims(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageCdims(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageBdims(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageZdims(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_imageIdims(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)    pxe_imageAspectRatios(pxdstate_s*,int unitmap);

/*
 * Image read/write
 */
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_readuchar  (int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,uchar  *membuf,size_t cnt,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_writeuchar (int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,uchar  *membuf,size_t cnt,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_readushort (int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,ushort *membuf,size_t cnt,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_writeushort(int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,ushort *membuf,size_t cnt,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,pximage_s*)	pxd_defineImage   (int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,pximage3_s*)	pxd_defineImage3  (int unitmap,pxbuffer_t startbuf,pxbuffer_t endbuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,pximage_s*)	pxd_definePximage (int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,pximage3_s*)	pxd_definePximage3(int unitmap,pxbuffer_t startbuf,pxbuffer_t endbuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,void)		pxd_definePximageFree (pximage_s*);
_cDcl(_dllpxlib,_cfunfcc,void)		pxd_definePximage3Free(pximage3_s*);

_cDcl(_dllpxlib,_cfunfcc,int)		pxe_readuchar  (pxdstate_s*,int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,uchar  *membuf,size_t cnt,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_writeuchar (pxdstate_s*,int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,uchar  *membuf,size_t cnt,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_readushort (pxdstate_s*,int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,ushort *membuf,size_t cnt,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_writeushort(pxdstate_s*,int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,ushort *membuf,size_t cnt,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,pximage_s*)	pxe_defineImage   (pxdstate_s*,int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,pximage3_s*)	pxe_defineImage3  (pxdstate_s*,int unitmap,pxbuffer_t startbuf,pxbuffer_t endbuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,pximage_s*)	pxe_definePximage (pxdstate_s*,int unitmap,pxbuffer_t framebuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,pximage3_s*)	pxe_definePximage3(pxdstate_s*,int unitmap,pxbuffer_t startbuf,pxbuffer_t endbuf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,char *colorspace);
_cDcl(_dllpxlib,_cfunfcc,void)		pxe_definePximageFree (pxdstate_s*,pximage_s*);
_cDcl(_dllpxlib,_cfunfcc,void)		pxe_definePximage3Free(pxdstate_s*,pximage3_s*);




/*
 * Video capture
 */
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_doSnap(int unitmap, pxbuffer_t buffer, ulong timeout);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_goSnap(int unitmap, pxbuffer_t buffer);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_goSnapPair(int unitmap, pxbuffer_t buffer1, pxbuffer_t buffer2);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_goLive(int unitmap, pxbuffer_t buffer);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_goUnLive(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_goAbortLive(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_goLivePair(int unitmap, pxbuffer_t buffer1, pxbuffer_t buffer2);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_goLiveSeq(int unitmap, pxbuffer_t startbuf,pxbuffer_t endbuf,pxbuffer_t incbuf,pxbuffer_t numbuf,int period);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_goLiveTrig(int unitmap, pxbuffer_t buffer,uint gpin10mask,uint gpout20value,uint gpout20mask,uint gpout20when,
						       uint gpin30wait,uint gpin30mask,uint gpout40value,uint gpout40mask,uint option50,uint field50,
						       uint gpout50value,uint gpout50mask,uint delay60,uint gpout60value,uint gpout60mask,uint delay70,
						       uint field70,uint capture70,uint gpin80mask,uint gpout80value,uint gpout80mask);

_cDcl(_dllpxlib,_cfunfcc,int)		pxd_goLiveSeqTrig(int unitmap, pxbuffer_t startbuf,pxbuffer_t endbuf,pxbuffer_t incbuf,pxbuffer_t numbuf,int period,
						       uint rsvd1,uint rsvd2,uint trig20wait,uint trig20slct,pxvbtime_t trig20delay,uint rsvd3,uint rsvd4,
						       uint rsvd5,uint rsvd6,pxvbtime_t rsvd7,uint rsvd8,uint rsvd9,uint trig40wait,uint trig40slct,
						       pxvbtime_t trig40delay,uint rsvd10,uint rsvd11,uint rsvd12,uint rsvd13,uint rsvd14,uint rsvd15);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_goneLive(int unitmap, int rsvd);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_videoFieldsPerFrame(void);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_videoFieldsPerFrames(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,pxvbtime_t)	pxd_videoFieldCount(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,pxbuffer_t)	pxd_capturedBuffer(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,uint32)	pxd_capturedSysTicks(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,pxvbtime_t)	pxd_capturedFieldCount(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,pxvbtime_t)	pxd_buffersFieldCount(int unitmap, pxbuffer_t buffer);
_cDcl(_dllpxlib,_cfunfcc,uint32)	pxd_buffersSysTicks(int unitmap, pxbuffer_t buffer);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_buffersSysTicks2(int unitmap, pxbuffer_t buffer, uint32 ticks[2]);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_buffersGPIn(int unitmap, pxbuffer_t buffer);
#define pxd_getFieldCount(u)		pxd_videoFieldCount(u)	// deprecated name

_cDcl(_dllpxlib,_cfunfcc,int)		pxe_doSnap(pxdstate_s*,int unitmap, pxbuffer_t buffer, ulong timeout);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_goSnap(pxdstate_s*,int unitmap, pxbuffer_t buffer);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_goSnapPair(pxdstate_s*,int unitmap, pxbuffer_t buffer1, pxbuffer_t buffer2);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_goLive(pxdstate_s*,int unitmap, pxbuffer_t buffer);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_goUnLive(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_goAbortLive(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_goLivePair(pxdstate_s*,int unitmap, pxbuffer_t buffer1, pxbuffer_t buffer2);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_goLiveSeq(pxdstate_s*,int unitmap, pxbuffer_t startbuf,pxbuffer_t endbuf,pxbuffer_t incbuf,pxbuffer_t numbuf,int period);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_goLiveTrig(pxdstate_s*,int unitmap, pxbuffer_t buffer,uint gpin10mask,uint gpout20value,uint gpout20mask,uint gpout20when,
						       uint gpin30wait,uint gpin30mask,uint gpout40value,uint gpout40mask,uint option50,uint field50,
						       uint gpout50value,uint gpout50mask,uint delay60,uint gpout60value,uint gpout60mask,uint delay70,
						       uint field70,uint capture70,uint gpin80mask,uint gpout80value,uint gpout80mask);

_cDcl(_dllpxlib,_cfunfcc,int)		pxe_goLiveSeqTrig(pxdstate_s*,int unitmap, pxbuffer_t startbuf,pxbuffer_t endbuf,pxbuffer_t incbuf,pxbuffer_t numbuf,int period,
						       uint rsvd1,uint rsvd2,uint trig20wait,uint trig20slct,pxvbtime_t trig20delay,uint rsvd3,uint rsvd4,
						       uint rsvd5,uint rsvd6,pxvbtime_t rsvd7,uint rsvd8,uint rsvd9,uint trig40wait,uint trig40slct,
						       pxvbtime_t trig40delay,uint rsvd10,uint rsvd11,uint rsvd12,uint rsvd13,uint rsvd14,uint rsvd15);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_goneLive(pxdstate_s*,int unitmap, int rsvd);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_videoFieldsPerFrame(pxdstate_s*);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_videoFieldsPerFrames(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,pxvbtime_t)	pxe_videoFieldCount(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,pxbuffer_t)	pxe_capturedBuffer(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,uint32)	pxe_capturedSysTicks(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,pxvbtime_t)	pxe_capturedFieldCount(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,pxvbtime_t)	pxe_buffersFieldCount(pxdstate_s*,int unitmap, pxbuffer_t buffer);
_cDcl(_dllpxlib,_cfunfcc,uint32)	pxe_buffersSysTicks(pxdstate_s*,int unitmap, pxbuffer_t buffer);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_buffersSysTicks2(pxdstate_s*,int unitmap, pxbuffer_t buffer, uint32 ticks[2]);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_buffersGPIn(pxdstate_s*,int unitmap, pxbuffer_t buffer);



/*
 * SV2/SV3/SV4/SV5 Video adjust
 */
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_setVidMux(int unitmap, int inmux);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_getVidMux(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_setContrastBrightness(int unitmap, double contrast, double brightness);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_setHueSaturation(int unitmap, double hue, double Ugain, double Vgain);
_cDcl(_dllpxlib,_cfunfcc,double)    pxd_getContrast(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)    pxd_getBrightness(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)    pxd_getHue(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)    pxd_getUGain(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)    pxd_getVGain(int unitmap);

_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_setVidMux(pxdstate_s*,int unitmap, int inmux);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_getVidMux(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_setContrastBrightness(pxdstate_s*,int unitmap, double contrast, double brightness);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_setHueSaturation(pxdstate_s*,int unitmap, double hue, double Ugain, double Vgain);
_cDcl(_dllpxlib,_cfunfcc,double)    pxe_getContrast(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)    pxe_getBrightness(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)    pxe_getHue(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)    pxe_getUGain(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)    pxe_getVGain(pxdstate_s*,int unitmap);


/*
 * D/D24/D32/A/D2X/D3X/CL1/CL2/E1/E4/EB1/EB1POCL/EC1/ECB1/ECB134/ECB2/EL1/E1DB/EL1DB/ELS2/E4DB/CL3SD/SI/SI1/SI2/SI4 Video adjust.
 *
 * Use of pxd_setExsyncPrincMode() should be avoided in preference
 * to loading an appropriate video setup file! If used, it must
 * not change resolution, trigger mode, etc!
 */
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_setExsyncPrin(int unitmap, uint exsync, uint prin);
_cDcl(_dllpxlib,_cfunfcc,uint)	pxd_getExsync(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,uint)	pxd_getPrin(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_setExsyncPrincMode(int unitmap, uint exsyncbits, uint princbits);
_cDcl(_dllpxlib,_cfunfcc,uint)	pxd_getExsyncMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,uint)	pxd_getPrincMode(int unitmap);

_cDcl(_dllpxlib,_cfunfcc,int)	pxe_setExsyncPrin(pxdstate_s*,int unitmap, uint exsync, uint prin);
_cDcl(_dllpxlib,_cfunfcc,uint)	pxe_getExsync(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,uint)	pxe_getPrin(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_setExsyncPrincMode(pxdstate_s*,int unitmap, uint exsyncbits, uint princbits);
_cDcl(_dllpxlib,_cfunfcc,uint)	pxe_getExsyncMode(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,uint)	pxe_getPrincMode(pxdstate_s*,int unitmap);


/*
 * G.P. In/Out
 */
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_getGPIn(int unitmap, int data);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_setGPIn(int unitmap, int data);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_setGPOut(int unitmap, int data);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_getGPOut(int unitmap, int data);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_getGPTrigger(int unitmap, int which);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_setCameraLinkCCOut(int unitmap, int data);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_getCameraLinkCCOut(int unitmap, int data);

_cDcl(_dllpxlib,_cfunfcc,int)	pxe_getGPIn(pxdstate_s*,int unitmap, int data);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_setGPIn(pxdstate_s*,int unitmap, int data);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_setGPOut(pxdstate_s*,int unitmap, int data);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_getGPOut(pxdstate_s*,int unitmap, int data);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_getGPTrigger(pxdstate_s*,int unitmap, int which);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_setCameraLinkCCOut(pxdstate_s*,int unitmap, int data);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_getCameraLinkCCOut(pxdstate_s*,int unitmap, int data);

/*
 * Display.
 */
#if  defined(OS_WIN95)|defined(OS_WIN95_DLL) \
    |defined(OS_WINNT)|defined(OS_WINNT_DLL) \
    |defined(OS_WIN64)|defined(OS_WIN64_DLL)
_cDcl(_dllpxlib,_cfunfcc,HGLOBAL)   pxd_renderDIBCreate(int unitmap, pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry, int mode, int options);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_renderDIBFree(HGLOBAL hDIB);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxd_renderStretchDIBits(int unitmap,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int options,
						HDC hDC,uint nX,uint nY,uint nWidth,uint nHeight,int winoptions);

_cDcl(_dllpxlib,_cfunfcc,HGLOBAL)   pxe_renderDIBCreate(pxdstate_s*,int unitmap, pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry, int mode, int options);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_renderDIBFree(pxdstate_s*,HGLOBAL hDIB);
_cDcl(_dllpxlib,_cfunfcc,int)	    pxe_renderStretchDIBits(pxdstate_s*,int unitmap,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int options,
						HDC hDC,uint nX,uint nY,uint nWidth,uint nHeight,int winoptions);

#endif

/*
 * Load/Save.
 */
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_saveBmp(int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int savemode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_loadBmp(int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int loadmode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_saveTga(int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int savemode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_savePcx(int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int savemode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_saveTiff(int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int savemode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_loadTiff(int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int loadmode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_saveRawBuffers(int unitmap,char *pathname, pxbuffer_t startbuf, pxbuffer_t endbuf, void *filehandle, pxbuffer_t fileoffset, uint32 alignment,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_loadRawBuffers(int unitmap,char *pathname, pxbuffer_t startbuf, pxbuffer_t endbuf, void *filehandle, pxbuffer_t fileoffset, uint32 alignment,int options);

_cDcl(_dllpxlib,_cfunfcc,int)	pxe_saveBmp(pxdstate_s*,int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int savemode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_loadBmp(pxdstate_s*,int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int loadmode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_saveTga(pxdstate_s*,int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int savemode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_savePcx(pxdstate_s*,int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int savemode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_saveTiff(pxdstate_s*,int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int savemode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_loadTiff(pxdstate_s*,int unitmap, char *pathname,pxbuffer_t buf,pxcoord_t ulx,pxcoord_t uly,pxcoord_t lrx,pxcoord_t lry,int loadmode,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_saveRawBuffers(pxdstate_s*,int unitmap,char *pathname, pxbuffer_t startbuf, pxbuffer_t endbuf, void *filehandle, pxbuffer_t fileoffset, uint32 alignment,int options);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_loadRawBuffers(pxdstate_s*,int unitmap,char *pathname, pxbuffer_t startbuf, pxbuffer_t endbuf, void *filehandle, pxbuffer_t fileoffset, uint32 alignment,int options);


/*
 * Display w. S/VGA support.
 */
#if  defined(OS_WIN95)|defined(OS_WIN95_DLL) \
    |defined(OS_WINNT)|defined(OS_WINNT_DLL) \
    |defined(OS_WIN64)|defined(OS_WIN64_DLL)
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_renderDirectVideoUnLive(int unitmap, HWND hWnd);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_renderDirectVideoLive(int unitmap, HWND hWnd, uint nX, uint nY,
					    uint nWidth,uint nHeight,COLORREF ClrKey1,COLORREF ClrKey2);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_renderDirectVideoDone(int unitmap, HWND hWnd);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_renderDirectVideoInit(int unitmap, HWND hWnd);

_cDcl(_dllpxlib,_cfunfcc,int)	pxe_renderDirectVideoUnLive(pxdstate_s*,int unitmap, HWND hWnd);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_renderDirectVideoLive(pxdstate_s*,int unitmap, HWND hWnd, uint nX, uint nY,
					    uint nWidth,uint nHeight,COLORREF ClrKey1,COLORREF ClrKey2);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_renderDirectVideoDone(pxdstate_s*,int unitmap, HWND hWnd);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_renderDirectVideoInit(pxdstate_s*,int unitmap, HWND hWnd);

#endif

/*
 * Events.
 */
#if  defined(OS_WIN95)|defined(OS_WIN95_DLL) \
    |defined(OS_WINNT)|defined(OS_WINNT_DLL) \
    |defined(OS_WIN64)|defined(OS_WIN64_DLL)
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxd_eventFieldCreate(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxd_eventFieldCreate2(int unitmap, int type);
_cDcl(_dllpxlib,_cfunfcc, void)  pxd_eventFieldClose(int unitmap, HANDLE hEvent);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxd_eventCapturedFieldCreate(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxd_eventCapturedFieldCreate2(int unitmap, int type);
_cDcl(_dllpxlib,_cfunfcc, void)  pxd_eventCapturedFieldClose(int unitmap, HANDLE hEvent);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxd_eventGPTriggerCreate(int unitmap, int which, int rsvd);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxd_eventGPTriggerCreate2(int unitmap, int which, int rsvd, int type);
_cDcl(_dllpxlib,_cfunfcc, void)  pxd_eventGPTriggerClose(int unitmap, int which, int rsvd, HANDLE hEvent);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxd_eventFaultCreate(int unitmap, int rsvd);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxd_eventFaultCreate2(int unitmap, int rsvd, int type);
_cDcl(_dllpxlib,_cfunfcc, void)  pxd_eventFaultClose(int unitmap, int rsvd, HANDLE hEvent);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxd_eventSerialCreate(int unitmap, int which, int rsvd, int type);
_cDcl(_dllpxlib,_cfunfcc, void)  pxd_eventSerialClose(int unitmap, int which, int rsvd, HANDLE hEvent);

_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxe_eventFieldCreate(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxe_eventFieldCreate2(pxdstate_s*,int unitmap, int type);
_cDcl(_dllpxlib,_cfunfcc, void)  pxe_eventFieldClose(pxdstate_s*,int unitmap, HANDLE hEvent);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxe_eventCapturedFieldCreate(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxe_eventCapturedFieldCreate2(pxdstate_s*,int unitmap, int type);
_cDcl(_dllpxlib,_cfunfcc, void)  pxe_eventCapturedFieldClose(pxdstate_s*,int unitmap, HANDLE hEvent);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxe_eventGPTriggerCreate(pxdstate_s*,int unitmap, int which, int rsvd);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxe_eventGPTriggerCreate2(pxdstate_s*,int unitmap, int which, int rsvd, int type);
_cDcl(_dllpxlib,_cfunfcc, void)  pxe_eventGPTriggerClose(pxdstate_s*,int unitmap, int which, int rsvd, HANDLE hEvent);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxe_eventFaultCreate(pxdstate_s*,int unitmap, int rsvd);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxe_eventFaultCreate2(pxdstate_s*,int unitmap, int rsvd, int type);
_cDcl(_dllpxlib,_cfunfcc, void)  pxe_eventFaultClose(pxdstate_s*,int unitmap, int rsvd, HANDLE hEvent);
_cDcl(_dllpxlib,_cfunfcc,HANDLE) pxe_eventSerialCreate(pxdstate_s*,int unitmap, int which, int rsvd, int type);
_cDcl(_dllpxlib,_cfunfcc, void)  pxe_eventSerialClose(pxdstate_s*,int unitmap, int which, int rsvd, HANDLE hEvent);

#endif
#if defined(OS_DOS4GW)
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventFieldCreate(int unitmap, pxasyncfunc_t *irqfunc, void *statep);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventFieldClose(int unitmap, pxasyncfunc_t *irqfunc);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventCapturedFieldCreate(int unitmap, pxasyncfunc_t *irqfunc, void *statep);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventCapturedFieldClose(int unitmap, pxasyncfunc_t *irqfunc);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventGPTriggerCreate(int unitmap, int which, int rsvd, pxasyncfunc_t *irqfunc, void *statep);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventGPTriggerClose(int unitmap, int which, int rsvd, pxasyncfunc_t *irqfunc);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventFaultCreate(int unitmap, int rsvd, pxasyncfunc_t *irqfunc, void *statep);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventFaultClose(int unitmap, int rsvd, pxasyncfunc_t *irqfunc);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventSerialCreate(int unitmap, int which, int rsvd, pxasyncfunc_t *irqfunc, void *statep);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventSerialClose(int unitmap, int which, int rsvd, pxasyncfunc_t *irqfunc);

_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventFieldCreate(pxdstate_s*,int unitmap, pxasyncfunc_t *irqfunc, void *statep);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventFieldClose(pxdstate_s*,int unitmap, pxasyncfunc_t *irqfunc);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventCapturedFieldCreate(pxdstate_s*,int unitmap, pxasyncfunc_t *irqfunc, void *statep);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventCapturedFieldClose(pxdstate_s*,int unitmap, pxasyncfunc_t *irqfunc);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventGPTriggerCreate(pxdstate_s*,int unitmap, int which, int rsvd, pxasyncfunc_t *irqfunc, void *statep);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventGPTriggerClose(pxdstate_s*,int unitmap, int which, int rsvd, pxasyncfunc_t *irqfunc);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventFaultCreate(pxdstate_s*,int unitmap, int rsvd, pxasyncfunc_t *irqfunc, void *statep);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventFaultClose(pxdstate_s*,int unitmap, int rsvd, pxasyncfunc_t *irqfunc);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventSerialCreate(pxdstate_s*,int unitmap, int which, int rsvd, pxasyncfunc_t *irqfunc, void *statep);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventSerialClose(pxdstate_s*,int unitmap, int which, int rsvd, pxasyncfunc_t *irqfunc);

#endif
#if defined(OS_LINUX_GNU)
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventFieldCreate(int unitmap, int sig, void *rsvd);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventFieldClose(int unitmap, int sig);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventCapturedFieldCreate(int unitmap, int sig, void *rsvd);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventCapturedFieldClose(int unitmap, int sig);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventGPTriggerCreate(int unitmap, int which, int rsvd, int sig, void *rvsd2);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventGPTriggerClose(int unitmap, int which, int rsvd, int sig);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventFaultCreate(int unitmap, int rsvd, int sig, void *rvsd2);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventFaultClose(int unitmap, int rsvd, int sig);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventSerialCreate(int unitmap, int which, int rsvd, int sig, void *rvsd2);
_cDcl(_dllpxlib,_cfunfcc, int)	pxd_eventSerialClose(int unitmap, int which, int rsvd, int sig);

_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventFieldCreate(pxdstate_s*,int unitmap, int sig, void *rsvd);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventFieldClose(pxdstate_s*,int unitmap, int sig);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventCapturedFieldCreate(pxdstate_s*,int unitmap, int sig, void *rsvd);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventCapturedFieldClose(pxdstate_s*,int unitmap, int sig);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventGPTriggerCreate(pxdstate_s*,int unitmap, int which, int rsvd, int sig, void *rvsd2);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventGPTriggerClose(pxdstate_s*,int unitmap, int which, int rsvd, int sig);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventFaultCreate(pxdstate_s*,int unitmap, int rsvd, int sig, void *rvsd2);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventFaultClose(pxdstate_s*,int unitmap, int rsvd, int sig);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventSerialCreate(pxdstate_s*,int unitmap, int which, int rsvd, int sig, void *rvsd2);
_cDcl(_dllpxlib,_cfunfcc, int)	pxe_eventSerialClose(pxdstate_s*,int unitmap, int which, int rsvd, int sig);

#endif

/*
 * Image corrections
 */
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_setImageDarkBalance(int unitmap, uint referenceRGB[3], uint targetRGB[3], double gamma);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_setImageBrightBalance(int unitmap, uint referenceRGB[3], uint targetRGB[3], double gamma);

_cDcl(_dllpxlib,_cfunfcc,int)	pxe_setImageDarkBalance(pxdstate_s*,int unitmap, uint referenceRGB[3], uint targetRGB[3], double gamma);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_setImageBrightBalance(pxdstate_s*,int unitmap, uint referenceRGB[3], uint targetRGB[3], double gamma);


/*
 * Serial port
 */
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_serialConfigure(int unitmap, int rsvd0, double baud, int bits, int parity, int stopbits, int rsvd1, int rsvd2, int rsvd3);
#if defined(CTOBAS)  // alternate declaration for VB/CTOBAS
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_serialRead(int unitmap, int rsvd0, uchar *buf, int cnt);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_serialWrite(int unitmap, int rsvd0, uchar *buf, int cnt);
#endif
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_serialRead(int unitmap, int rsvd0, char buf[], int cnt);
_cDcl(_dllpxlib,_cfunfcc,int)	pxd_serialWrite(int unitmap, int rsvd0, char buf[], int cnt);

_cDcl(_dllpxlib,_cfunfcc,int)	pxe_serialConfigure(pxdstate_s*,int unitmap, int rsvd0, double baud, int bits, int parity, int stopbits, int rsvd1, int rsvd2, int rsvd3);
#if defined(CTOBAS)  // alternate declaration for VB/CTOBAS
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_serialRead(pxdstate_s*,int unitmap, int rsvd0, uchar *buf, int cnt);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_serialWrite(pxdstate_s*,int unitmap, int rsvd0, uchar *buf, int cnt);
#endif
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_serialRead(pxdstate_s*,int unitmap, int rsvd0, char buf[], int cnt);
_cDcl(_dllpxlib,_cfunfcc,int)	pxe_serialWrite(pxdstate_s*,int unitmap, int rsvd0, char buf[], int cnt);




/*
 * Serial port, CameraLink standard API.
 * (N.B. The clSerial() functions in XCLIB only work
 * with the library already opened. A seperate clserEPX.dll
 * is available for stand-alone use w/out XCLIB).
 *
 * See the Camera Link spec full declarations.
 * This is not the same API as described in clallserial.h and
 * implemented in clallserial.dll.
 */
#if  defined(OS_WIN95)|defined(OS_WIN95_DLL) \
    |defined(OS_WINNT)|defined(OS_WINNT_DLL) \
    |defined(OS_WIN64)|defined(OS_WIN64_DLL)

#if !defined(__clallserial_h__)
_cDcl(_dllpxlib,__cdecl,sint32)     clSerialInit();
_cDcl(_dllpxlib,__cdecl,void)	    clSerialClose();
_cDcl(_dllpxlib,__cdecl,sint32)     clSerialRead();
_cDcl(_dllpxlib,__cdecl,sint32)     clSerialWrite();
_cDcl(_dllpxlib,__cdecl,sint32)     clGetNumBytesAvail();
_cDcl(_dllpxlib,__cdecl,sint32)     clFlushPort();
_cDcl(_dllpxlib,__cdecl,sint32)     clGetErrorText();
_cDcl(_dllpxlib,__cdecl,sint32)     clGetNumSerialPorts();
_cDcl(_dllpxlib,__cdecl,sint32)     clGetSerialPortIdentifier();
_cDcl(_dllpxlib,__cdecl,sint32)     clGetManufacturerInfo();
_cDcl(_dllpxlib,__cdecl,sint32)     clGetSupportedBaudRates();
_cDcl(_dllpxlib,__cdecl,sint32)     clSetBaudRate();
#endif

#endif


/*
 * Camera specific.
 * Deprecated.
 */
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV2112_setExposureAndGain(int unitmap, int rsvd, double exposure, double redgain, double grngain, double blugain);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV2112_setResolutionAndTiming(int unitmap, int rsvd, int decimation, int aoileft, int aoitop, int aoiwidth, int aoiheight,
								  int scandirection, double pixelClkFreq, double rsvd2, int rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV2112_setVideoAndTriggerMode(int unitmap, int rsvd, int videomode, int controlledvideomode, int controlledtrigger);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV2112_setCtrlExposureAndRate(int unitmap, int rsvd, double exposure, double framerate);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV2112_getExposure(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV2112_getGain(int unitmap, int color);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV2112_getDecimation(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV2112_getAoiTop(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV2112_getAoiLeft(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV2112_getPixelClock(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV2112_getScanDirection(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV2112_getVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV2112_getCtrlVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV2112_getCtrlTriggerMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV2112_getCtrlFrameRate(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV2112_getCtrlExposure(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV2112_getMinMaxExposure(int unitmap, double exposure);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV2112_getMinMaxCtrlExposure(int unitmap, double exposure);

_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_setExposureGainOffset(int unitmap, int rsvd, double exposure, double gain, double offset, double rsvd2, double rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_setColorGain(int unitmap, int rsvd, double greenR, double red, double blue, double greenB);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_setResolutionAndTiming(int unitmap, int rsvd, int subsample, int aoileft, int aoitop, int aoiwidth, int aoiheight,
								   int readoutdirection, double pixelClkFreq, double framePeriod, double rsvd2);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_setVideoAndTriggerMode(int unitmap, int rsvd, int videomode, int controlledmode, int controlledtrigger, int strobemode, int rsvd2, int rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_setCtrlRate(int unitmap, int rsvd, double rsvd2, double framerate, double rsvd3);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1310_getExposure(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1310_getGain(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1310_getOffset(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_getSubsample(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_getAoiTop(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_getAoiLeft(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_getReadoutDirection(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1310_getPixelClock(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1310_getFramePeriod(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1310_getCtrlFrameRate(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_getVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_getCtrlVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_getCtrlTriggerMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1310_getStrobeMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1310_getColorGain(int unitmap, int color);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1310_getMinMaxExposure(int unitmap, double exposure);

_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1281_setExposureGainOffset(int unitmap, int rsvd, double exposure,
						double gain, double offset, double rsvd2, double rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1281_setResolutionAndTiming(int unitmap, int rsvd, int rsvd1,
						int aoileft, int aoitop, int aoiwidth, int aoiheight,
						int rsvd4, double pixelClkFreq, double rsvd2, double rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1281_setVideoAndTriggerMode(int unitmap, int rsvd, int videomode, int controlledvideomode, int controlledtrigger, int rsvd1, int rsvd2, int rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1281_setCtrlRate(int unitmap, int rsvd, double rsvd2, double framerate, double rsvd3);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1281_getExposure(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1281_getGain(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1281_getOffset(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1281_getAoiTop(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1281_getAoiLeft(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1281_getPixelClock(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1281_getVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1281_getCtrlVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV1281_getCtrlTriggerMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1281_getCtrlFrameRate(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV1281_getMinMaxExposure(int unitmap, double exposure);

_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_setExposureAndGain(int unitmap, int rsvd, double exposure, double redgain, double grnrgain, double bluegain, double grnbgain);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_setResolutionAndTiming(int unitmap, int rsvd, int subsample, int aoileft, int aoitop, int aoiwidth, int aoiheight, int scandirection, double pixelClkFreq, double framePeriod, double rsvd2);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_setVideoAndTriggerMode(int unitmap, int rsvd, int videomode, int controlledmode, int controlledtrigger, int rsvd4, int rsvd2, int rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_setCtrlRate(int unitmap, int rsvd, double rsvd2, double framerate, double rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_setExposureAndDigitalGain(int unitmap, int rsvd, double exposure, double digitalgain, double rsvd2, double rsvd3, double rsvd4);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV9M001_getExposure(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_getAoiTop(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_getAoiLeft(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV9M001_getGain(int unitmap, int color);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV9M001_getPixelClock(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV9M001_getFramePeriod(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_getVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV9M001_getCtrlFrameRate(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_getCtrlVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_getCtrlTriggerMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_getSubsample(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV9M001_getScanDirection(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV9M001_getDigitalGain(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV9M001_getMinMaxExposure(int unitmap, double exposure);


_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV642_setExposureAndGain(int unitmap, int rsvd1, double exposure, double redgain, double grnrgain, double bluegain, double grnbgain, int gainrange);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV642_setResolutionAndTiming(int unitmap, int rsvd, int rsvd2, int aoileft, int aoitop, int aoiwidth, int aoiheight, int rsvd3, double pixelClkFreq, double framePeriod, double rsvd4);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV642_setVideoAndTriggerMode(int unitmap, int rsvd, int videomode, int controlledmode, int controlledtrigger, int rsvd4, int rsvd2, int rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV642_setCtrlRate(int unitmap, int rsvd, double rsvd2, double framerate, double rsvd3);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV642_getExposure(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV642_getAoiTop(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV642_getAoiLeft(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV642_getGain(int unitmap, int color);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV642_getPixelClock(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV642_getFramePeriod(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV642_getVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV642_getCtrlFrameRate(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV642_getCtrlVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV642_getCtrlTriggerMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV642_getGainRange(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV642_getMinMaxExposure(int unitmap, double exposure);

_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV643_setExposureAndGain(int unitmap, int rsvd1, double exposure, double gain);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV643_setExposureAndGainOffset(int unitmap, int rsvd1, double exposure, double gain, double offset, double rsvd2);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV643_setResolutionAndTiming(int unitmap, int rsvd, int subsample, int aoileft, int aoitop, int aoiwidth, int aoiheight, int rsvd3, double pixelClkFreq, double framePeriod, double rsvd4);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV643_setVideoAndTriggerMode(int unitmap, int rsvd, int videomode, int controlledmode, int controlledtrigger, int rsvd4, int rsvd2, int rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV643_setCtrlExposureAndRate(int unitmap, int rsvd, double exposure, double framerate, double rsvd3);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV643_getExposure(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV643_getAoiTop(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV643_getAoiLeft(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV643_getGain(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV643_getOffset(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV643_getPixelClock(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV643_getFramePeriod(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV643_getVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV643_getCtrlFrameRate(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV643_getCtrlExposure(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV643_getCtrlVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV643_getCtrlTriggerMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	 pxd_SV643_getSubsample(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV643_getMinMaxExposure(int unitmap, double exposure);
_cDcl(_dllpxlib,_cfunfcc,double) pxd_SV643_getMinMaxCtrlExposure(int unitmap, double exposure);


/*
 * SILICON VIDEO camera functions.
 * Replaces deprecated functions, above.
 */
_cDcl(_dllpxlib,_cfunfcc,int) pxd_SILICONVIDEO_setExposureGainOffset(
				    int unitmap,int rsvd,double exposure,double gainA,double offsetA,double gainB,double offsetB);
_cDcl(_dllpxlib,_cfunfcc,int) pxd_SILICONVIDEO_setExposureColorGainOffsets(
				    int unitmap,int rsvd,double exposure,double gainsA[4],double gainsB[4],double offsetsA[4],double offsetsB[4]);
_cDcl(_dllpxlib,_cfunfcc,int) pxd_SILICONVIDEO_setExposure(int unitmap,int rsvd,double exposure);
_cDcl(_dllpxlib,_cfunfcc,int) pxd_SILICONVIDEO_setVideoAndTriggerMode(
				    int unitmap,int rsvd,int videomode,int controlledmode,int controlledtrigger,int rsvd1,int rsvd2,int rsvd3,int rsvd4);
_cDcl(_dllpxlib,_cfunfcc,int) pxd_SILICONVIDEO_setResolutionAndTiming(
				    int unitmap,int rsvd1,int subsample,int aoileft,int aoitop,int aoiwidth,int aoiheight,int scandirection,
				    int bitdepth,int rsvd3,int rsvd4,double pixelClkFreq,double framePeriod,double rsvd5,double rsvd6,double rsvd7);
_cDcl(_dllpxlib,_cfunfcc,int) pxd_SILICONVIDEO_setCtrlRates(
				    int unitmap,int rsvd,double rsvd1,double framerate,double rsvd2,double rsvd3,double rsvd4);
_cDcl(_dllpxlib,_cfunfcc,int) pxd_SILICONVIDEO_setAxC(int unitmap,int rsvd,
				    int agcA,int agcB,int rsvd2,int rsvd3,int aec,int rsvd4,int rsvd5,int rsvd6,int rsvd7);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getExposure(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getAoiTop(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getAoiLeft(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getAoiWidth(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getAoiHeight(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getPixelClock(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getFramePeriod(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getSubsample(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getScanDirection(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getCtrlFrameRate(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getCtrlVideoMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getCtrlTriggerMode(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getGainA(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getGainB(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getGainsA(int unitmap,double gainsA[4]);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getGainsB(int unitmap,double gainsB[4]);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getOffsetsA(int unitmap,double offsetsA[4]);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getOffsetsB(int unitmap,double offsetsB[4]);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getOffsetA(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getOffsetB(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getMinMaxExposure(int unitmap,double exposure);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getMinMaxFramePeriod(int unitmap,double framePeriod);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getMinMaxCtrlFrameRate(int unitmap,double frameRate);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getMinMaxPixelClock(int unitmap,double pixelClkFreq);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getMinMaxGainA(int unitmap,double gain);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getMinMaxGainB(int unitmap,double gain);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getMinMaxOffsetA(int unitmap,double offset);
_cDcl(_dllpxlib,_cfunfcc,double)  pxd_SILICONVIDEO_getMinMaxOffsetB(int unitmap,double offset);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getMinMaxAoiWidth(int unitmap, int width);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getMinMaxAoiHeight(int unitmap, int height);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getAgcA(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getAgcB(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getAec(int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getMinMaxAgcA(int unitmap, int agc);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getMinMaxAgcB(int unitmap, int agc);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxd_SILICONVIDEO_getMinMaxAec(int unitmap, int aec);

_cDcl(_dllpxlib,_cfunfcc,int) pxe_SILICONVIDEO_setExposureGainOffset(pxdstate_s*,
				    int unitmap,int rsvd,double exposure,double gainA,double offsetA,double gainB,double offsetB);
_cDcl(_dllpxlib,_cfunfcc,int) pxe_SILICONVIDEO_setExposureColorGainOffsets(pxdstate_s*,
				    int unitmap,int rsvd,double exposure,double gainsA[4],double gainsB[4],double offsetsA[4],double offsetsB[4]);
_cDcl(_dllpxlib,_cfunfcc,int) pxe_SILICONVIDEO_setExposure(pxdstate_s*,int unitmap,int rsvd,double exposure);
_cDcl(_dllpxlib,_cfunfcc,int) pxe_SILICONVIDEO_setVideoAndTriggerMode(pxdstate_s*,
				    int unitmap,int rsvd,int videomode,int controlledmode,int controlledtrigger,int rsdv1,int rsvd2,int rsvd3,int rsvd4);
_cDcl(_dllpxlib,_cfunfcc,int) pxe_SILICONVIDEO_setResolutionAndTiming(pxdstate_s*,
				    int unitmap,int rsvd1,int subsample,int aoileft,int aoitop,int aoiwidth,int aoiheight,int scandirection,
				    int bitdepth,int rsvd3,int rsvd4,double pixelClkFreq,double framePeriod,double rsvd5,double rsvd6,double rsvd7);
_cDcl(_dllpxlib,_cfunfcc,int) pxe_SILICONVIDEO_setCtrlRates(pxdstate_s*,
				    int unitmap,int rsvd,double rsvd1,double framerate,double rsvd2,double rsvd3,double rsvd4);
_cDcl(_dllpxlib,_cfunfcc,int) pxe_SILICONVIDEO_setAxC(pxdstate_s*,int unitmap,int rsvd,
				    int agcA,int agcB,int rsvd2,int rsvd3,int aec,int rsvd4,int rsvd5,int rsvd6,int rsvd7);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getExposure(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getAoiTop(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getAoiLeft(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getAoiWidth(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getAoiHeight(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getPixelClock(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getFramePeriod(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getSubsample(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getScanDirection(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getVideoMode(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getCtrlFrameRate(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getCtrlVideoMode(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getCtrlTriggerMode(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getGainA(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getGainB(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getGainsA(pxdstate_s*,int unitmap,double gainsA[4]);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getGainsB(pxdstate_s*,int unitmap,double gainsB[4]);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getOffsetsA(pxdstate_s*,int unitmap,double offsetsA[4]);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getOffsetsB(pxdstate_s*,int unitmap,double offsetsB[4]);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getOffsetA(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getOffsetB(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getMinMaxExposure(pxdstate_s*,int unitmap,double exposure);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getMinMaxFramePeriod(pxdstate_s*,int unitmap,double framePeriod);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getMinMaxCtrlFrameRate(pxdstate_s*,int unitmap,double frameRate);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getMinMaxPixelClock(pxdstate_s*,int unitmap,double pixelClkFreq);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getMinMaxGainA(pxdstate_s*,int unitmap,double gain);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getMinMaxGainB(pxdstate_s*,int unitmap,double gain);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getMinMaxOffsetA(pxdstate_s*,int unitmap,double offset);
_cDcl(_dllpxlib,_cfunfcc,double)  pxe_SILICONVIDEO_getMinMaxOffsetB(pxdstate_s*,int unitmap,double offset);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getMinMaxAoiWidth(pxdstate_s*,int unitmap, int width);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getMinMaxAoiHeight(pxdstate_s*,int unitmap, int height);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getAgcA(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getAgcB(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getAec(pxdstate_s*,int unitmap);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getMinMaxAgcA(pxdstate_s*,int unitmap, int agc);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getMinMaxAgcB(pxdstate_s*,int unitmap, int agc);
_cDcl(_dllpxlib,_cfunfcc,int)	  pxe_SILICONVIDEO_getMinMaxAec(pxdstate_s*,int unitmap, int aec);



/*
 * Escape to Structured Interface services.
 * The pxd_xclibEscaped should be called to 'release'
 * the access obtained via pxd_xclibEscape; it causes an update
 * of cached information maintained by the SCF functions and resets
 * the video engine, leaving the state similar to that after a pxd_PIXCIopen.
 */
_cDcl(_dllpxlib,_cfunfcc,xclibs_s *)	pxd_xclibEscape(int rsvd1, int rsvd2, int rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)		pxd_xclibEscaped(int rsvd1, int rsvd2, int rsvd3);

_cDcl(_dllpxlib,_cfunfcc,xclibs_s *)	pxe_xclibEscape(pxdstate_s*,int rsvd1, int rsvd2, int rsvd3);
_cDcl(_dllpxlib,_cfunfcc,int)		pxe_xclibEscaped(pxdstate_s*,int rsvd1, int rsvd2, int rsvd3);


/*
 * Allow compiling a video format configuration
 * into the application, and loading as desired.
 * Use as
 *  {
 *	#include "newformat.fmt"
 *	pxd_videoFormatAsIncluded(0);
 *  }
 * The suggested braces allow multiple uses by limiting the 'C'
 * scope of the names declared within the included file.
 *
 *
 */
static	struct pxvidstate   pxd_videoFormatAsIncluded_vidstate;     /* internal use */
static	int		    pxd_videoFormatAsIncluded_r1;	    /* internal use */
static	int		    pxd_videoFormatAsIncluded_r2;	    /* internal use */
#define pxd_videoFormatAsIncluded(rsvd) \
     (memset(&pxd_videoFormatAsIncluded_vidstate, 0, sizeof(pxd_videoFormatAsIncluded_vidstate)), \
     (pxd_videoFormatAsIncluded_vidstate.ddch.len    = sizeof(pxd_videoFormatAsIncluded_vidstate)),\
     (pxd_videoFormatAsIncluded_vidstate.ddch.mos    = PXMOS_VIDSTATE),\
     (pxd_videoFormatAsIncluded_vidstate.vidformat   = &pxvidformat_561_id31232),\
     (pxd_videoFormatAsIncluded_vidstate.vidres      = &pxvidres_125_id31232),\
     (pxd_videoFormatAsIncluded_vidstate.vidmode     = &pxvidmode_79_id31232),\
     (pxd_videoFormatAsIncluded_vidstate.vidphys     = &pxvidphys_118_id31232),\
     (pxd_videoFormatAsIncluded_vidstate.vidimage    = &pxvidimage_251_id31232),\
     (pxd_videoFormatAsIncluded_vidstate.vidopt      = &pxvidopt_19_id31232),\
     (pxd_videoFormatAsIncluded_vidstate.vidmem      = &pxvidmem_72_id31232),\
     (pxd_videoFormatAsIncluded_vidstate.camcntl     = &pxcamcntl_264_id31232),\
     (pxd_videoFormatAsIncluded_vidstate.xc.sv2format= &xcsv2format_32_id31232),\
     (pxd_videoFormatAsIncluded_vidstate.xc.sv2mode  = &xcsv2mode_21_id31232),\
     (pxd_videoFormatAsIncluded_vidstate.xc.dxxformat= &xcdxxformat_45_id31232),\
     pxd_goAbortLive((1<<pxd_infoUnits())-1),\
     (pxd_videoFormatAsIncluded_r1 = pxd_xclibEscape(0,0,0)->pxlib.defineState(&pxd_xclibEscape(0,0,0)->pxlib, 0, PXMODE_DIGI, &pxd_videoFormatAsIncluded_vidstate)),\
     (pxd_videoFormatAsIncluded_r2 = pxd_xclibEscaped(0,0,0)), \
     ((pxd_videoFormatAsIncluded_r1 < pxd_videoFormatAsIncluded_r2) ? pxd_videoFormatAsIncluded_r1 : pxd_videoFormatAsIncluded_r2))



/*
 * Helpful conversion macros for programs using the XCOBJ API.
 * Assumes one imaging board is in use. Not a complete set of
 * conversions, but will handle the needs of most applications.
 *
 * Most conversions, below, are straightfoward, and can be read
 * as an aid to understanding the differences between XCOBJ and
 * XCLIB. Other conversions are easy in principle, but the macros
 * become complex due to having to convert parameters from one
 * format to another.  Use of pxd_ioc, in particular, is very easy
 * to convert manually, but because XCOBJ used one function to
 * specify an AOI and a second to actually transfer, the macros
 * must save information from one macro invocation to the next.
 * Also, the pxd_ioc conversion only supports the most common modes,
 * of reading or writing pixels left to right and top to bottom.
 *
 * Note that while a different function is used to obtain a struct
 * pximage (for an alternate to using pxd_io frame buffer access); once
 * obtained the struct pximage operates in the same manner as in XCOBJ.
 */
#if defined(XCOBJ_TO_XCLIB_MACROS)
#include <string.h>	    /* for pxd_xcopen */
#define pxd_close()	    pxd_PIXCIclose()
#define pxd_xcopen(F,P)     pxd_PIXCIopen(P, \
				 (F&&!( stricmp(F,"RS-170") &stricmp(F,"NTSC") \
				       &stricmp(F,"NTSC/YC")&stricmp(F,"CCIR") \
				       &stricmp(F,"PAL")    &stricmp(F,"PAL/YC") \
				       &stricmp(F,"PAL(M)") &stricmp(F,"PAL(M)/YC") \
				       &stricmp(F,"PAL(N)") &stricmp(F,"PAL(N)/YC") \
				       &stricmp(F,"SECAM")  &stricmp(F,"SECAM/YC")))?F: (F?NULL:"Default"), \
				 (F&&!( stricmp(F,"RS-170") &stricmp(F,"NTSC") \
				       &stricmp(F,"NTSC/YC")&stricmp(F,"CCIR") \
				       &stricmp(F,"PAL")    &stricmp(F,"PAL/YC") \
				       &stricmp(F,"PAL(M)") &stricmp(F,"PAL(M)/YC") \
				       &stricmp(F,"PAL(N)") &stricmp(F,"PAL(N)/YC") \
				       &stricmp(F,"SECAM")  &stricmp(F,"SECAM/YC")))?NULL:F);

#define pxvid_xbuf(S)			    (pxd_capturedBuffer(1)>0?pxd_capturedBuffer(1):1)
#define pxd_xcmodel()			    pxd_infoModel(1)
#define pxd_xcmodelcamera()		    pxd_infoSubmodel(1)
#define pxd_udim()			    pxd_infoUnits()
#define pxd_xdim()			    pxd_imageXdim()
#define pxd_ydim()			    (pxd_imageYdim()/(pxd_imageIdim()==0?1:pxd_imageIdim()))
#define pxd_cdim()			    pxd_imageCdim()
#define pxd_bdim()			    (pxd_imageCdim()*pxd_imageBdim())
#define pxd_ylace()			    (pxd_imageIdim()-1)
#define pxd_vidmux(m)			    pxd_setVidMux(1,m+1)
#define pxd_imbufs()			    pxd_imageZdim()
#define pxd_imsize()			    (pxd_infoMemsize(1)/1024)
#define pxd_chkfault(R) 		    pxd_mesgFault(1)
#define pxd_chkstack(R)
#define pxerrnomesg(M)			    pxd_mesgErrorCode(M)
#define pxd_defimage(Z,A,B,C,D) 	    pxd_defineImage(1,Z>0?Z:pxvid_xbuf(0),A,B,C,D, pxd_imageCdim()>1?"RGB":"Grey")
#define pxd_defimage3(Y,Z,A,B,C,D)	    pxd_defineImage3(1,Y,Z,A,B,C,D, pxd_imageCdim()>1?"RGB":"Grey")
#define pxd_defimagecolor(Z,A,B,C,D,G)	    pxd_defineImage(1,Z>0?Z:pxvid_xbuf(0),A,B,C,D,G)
#define pxd_defimage3color(Y,Z,A,B,C,D,G)   pxd_defineImage3(1,Y,Z,A,B,C,D,G)
#define pxd_vidtime()			    pxd_getFieldCount(1)
#define pxd_StretchDIBits(A,B,C,D,E,F,G, H,I,J,K,L,M) \
					    pxd_renderStretchDIBits(1,A>0?A:pxvid_xbuf(0),B,C,D,E,0, H,I,J,K,L,0)
#define pxd_snap(A,B,C,D,E)		    pxd_doSnap(1, B>0?B:pxvid_xbuf(0), C)
#define pxd_video(A,B)			    (A=='z'? pxd_goLive(1,B>0?B:pxvid_xbuf(0)): pxd_goUnLive(1))

int	pxd_io1[8][6];	    /* must save pxd_iopen's state for later pxd_ioc! */
char	pxd_io2[8][32];     /* must save pxd_iopen's state for later pxd_ioc! */
#define pxd_iopen(H,B,ULX,ULY,LRX,LRY,MODE) \
		    (((H)<0||(H)>=8||!((MODE)==('r'^'x')||(MODE)==('w'^'x')))? PXERROR: \
			((pxd_io1[H][0]=B,pxd_io1[H][1]=ULX, \
			  pxd_io1[H][2]=ULY,pxd_io1[H][3]=LRX, \
			  pxd_io1[H][4]=LRY,pxd_io1[H][5]=MODE,strncpy(pxd_io2[H],"Grey",32)), 1))

#define pxd_io(h,b,n)	pxd_ioc(h,b,n)
#define pxd_ioc(H,B,N)	(((H)<0||(H)>=8)? PXERROR: \
			pxd_io1[H][5]==('r'^'x') ? pxd_readuchar(1, pxd_io1[H][0], pxd_io1[H][1], pxd_io1[H][2], \
								  pxd_io1[H][3], pxd_io1[H][4], B, N, pxd_io2[H]) : \
			pxd_io1[H][5]==('w'^'x') ? pxd_writeuchar(1, pxd_io1[H][0], pxd_io1[H][1], pxd_io1[H][2], \
								  pxd_io1[H][3], pxd_io1[H][4], B, N, pxd_io2[H]) : \
			PXERROR)
#define pxd_ios(H,B,N)	(((H)<0||(H)>=8)? PXERROR: \
			pxd_io1[H][5]==('r'^'x') ? pxd_readushort(1, pxd_io1[H][0], pxd_io1[H][1], pxd_io1[H][2], \
								  pxd_io1[H][3], pxd_io1[H][4], B, N, pxd_io2[H]) : \
			pxd_io1[H][5]==('w'^'x') ? pxd_writeushort(1, pxd_io1[H][0], pxd_io1[H][1], pxd_io1[H][2], \
								  pxd_io1[H][3], pxd_io1[H][4], B, N, pxd_io2[H]) : \
			PXERROR)

#define pxd_setDalsa01(p,e)	{ pxd_setExsyncPrin(1,e>>16,p>>16); pxd_setExsyncPrincMode(1,e&0xFFFF,p&0xFFFF); }
#define pxd_setKodak01(p,e)	pxd_setDalsa01(p,e)
#define pxd_setHitachi01(p,e)	pxd_setDalsa01(p,e)
#define pxd_setBasler01(p,e)	pxd_setDalsa01(p,e)

#define pxd_extin(w)		pxd_getGPIn(1,0)
#define pxd_extinreset(v,w)	pxd_setGPIn(1,v)
#define pxd_extout(v,w) 	pxd_setGPOut(1,v)

#endif	    /* defined(XCOBJ_TO_XCLIB_MACROS) */

#ifdef  __cplusplus
}
#endif

#include "cext_hpe.h"     
#endif	    /* !defined(__EPIX_XCLIBSC_DEFINED) */
