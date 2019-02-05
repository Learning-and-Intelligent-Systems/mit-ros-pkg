/*
 *
 *	xclibex5.cpp	External	13-Nov-2010
 *
 *	Copyright (C)  1998-2010  EPIX, Inc.  All rights reserved.
 *
 *	Example program for the XCLIB 'C' Library.
 *	Example assumes Windows 95/98/ME/NT/2000/XP/Vista/7/XP(x64)/Vista(x64)/7(x64).
 *
 *	Demonstrates XCLIB and/or PXIPL functions for capture and
 *	display from multiple PIXCI(R) imaging boards where
 *	the boards are not identical or each board is operating
 *	at a different resolution or a different camera.
 *	This Windows program must, of course, also make use of
 *	various Windows GDI API functions; however, this is not
 *	intended to serve as a Windows tutorial.
 *
 *	Many parts similar to the XCLIBEX2.CPP example; that
 *	example provides PIXCI SV2, SV3, SV4, SV5, SV5A, SV5B, SV5L, SV6, and SV7
 *	specific controls, and demonstrates more display options including
 *	graphic overlays. It also demonstrates access to numeric
 *	pixel values and use of Events rather than a timer.
 *	It also demonstrates use of multiple boards and cameras
 *	of the same type and operated at the same resolution.
 *	For simplicity this example concentrates on operating
 *	multiple boards at different resolutions and does not illustrate
 *	as many display options nor duplicate the other features
 *	demonstrated in that example. This example also
 *	demonstrates use of the non pxd_ style functions.
 *
 */


/*
 *  INSTRUCTIONS:
 *
 *
 *  1.1) Choose whether we are operating the identical boards
 *  but with different resolutions and/or cameras.
 *  Or whether we are operating different  boards.
 *  The former can be done with one open instance of the library.
 *  and allows performing some actions on all boards with one function call.
 *  The latter requires opening multiple instances of the library,
 *  one for each different type of board.
 *
 *  Only one of these choices should have value 1, the others should be 0.
 */
#if !defined(ONELIB_TWOUNIT) && !defined(TWOLIB_ONEUNIT) && !defined(PXDLIB_ENHANCED) && !defined(PXELIB_ENHANCED)
    #define ONELIB_TWOUNIT	0   // open one instance which controls multiple, identical, boards ..
				    // .. using structured functions
    #define TWOLIB_ONEUNIT	0   // open multiple instances, each controlling one board, identical or not, ..
				    // .. using structured functions
    #define PXDLIB_ENHANCED	0   // open one instance which controls multiple, identical, boards ..
				    // .. using new enhanced features of pxd_ functions
				    // .. Only FORMATFILE_LOAD option, below, is suppported
    #define PXELIB_ENHANCED	1   // open multiple instances, each controlling one board, identical or not, ..
				    // .. using new pxe_ functions, which are enhanced variations of pxd_ functions
#endif


/*
 *
 *  1.2)  Set 'define' options below according to the intended camera
 *	and video format(s).
 *
 *	For PIXCI(R) SV2, SV3, SV4, SV5, SV5A, SV5B, SV6L, and SV6 imaging boards
 *	common choices are RS-170, NSTC, NTSC/YC, CCIR, PAL, or PAL/YC.
 *	(The SV5A and SV5B do not support NTSC/YC or PAL/YC).
 *	For PIXCI(R) SV7 imaging boards
 *	common choices are RS-170, NSTC, CCIR, or PAL.
 *
 *	For PIXCI(R) A, CL1, CL2, CL3SD, D, D24, D32, D2X, D3X, E1, E1DB, E4, E4DB,
 *	EB1, EB1POCL, EC1, ECB1, ECB1-34, ECB2, EL1, EL1DB, ELS2, SI, SI1, SI2, and SI4
 *	imaging boards, use "default" to select the default format for
 *	the camera for which the PIXCI(R) imaging board is intended.
 *	For non default formats, use XCAP to save the video setup to
 *	a file, and set FORMATFILE_LOAD to the saved file's path name.
 *	For camera's with RS-232 control, note that the saved
 *	video setup only resets the PIXCI(R) imaging board's
 *	settings, but XCLIB does not reset the camera's settings.
 *	For selected Camera Link cameras w. serial controls,
 *	the video setup file may include serial commands which are
 *	automatically sent by XCLIB to the camera.
 *
 *	Alternately, this could be modified to use getenv("PIXCI"),
 *	GetPrivateProfileString(...), RegQueryValueEx(...), or any
 *	other convention chosen by the programmer to allow run time
 *	selection of the video format and resolution.
 *
 */

#if !defined(FORMAT_UNIT0) && !defined(FORMAT_UNIT1) \
 && !defined(FORMATFILE_LOAD) && !defined(FORMATFILE_COMP)
					// For PIXCI(R) SV2, SV3, SV4, SV5, SV5A, SV5B, SV5L, SV6
    //#define FORMAT_UNIT0  "RS-170"	// RS-170 on input 2
    //#define FORMAT_UNIT0  "NTSC"	// NTSC on input 2
    //#define FORMAT_UNIT0  "NTSC/YC"	// NSTC S-Video on input 1		(N/A on SV5A,SV5B)
    //#define FORMAT_UNIT0  "CCIR"	// CCIR on input 2
    //#define FORMAT_UNIT0  "PAL"	// PAL (B,D,G,H,I) on input 2
    //#define FORMAT_UNIT0  "PAL/YC"	// PAL (B,D,G,H,I) S-Video on input 1	(N/A on SV5A,SV5B)
    //#define FORMAT_UNIT1  "RS-170"	// RS-170 on input 2
    //#define FORMAT_UNIT1  "NTSC"	// NTSC on input 2
    //#define FORMAT_UNIT1  "NTSC/YC"	// NSTC S-Video on input 1		(N/A on SV5A,SV5B)
    //#define FORMAT_UNIT1  "CCIR"	// CCIR on input 2
    //#define FORMAT_UNIT1  "PAL"	// PAL (B,D,G,H,I) on input 2
    //#define FORMAT_UNIT1  "PAL/YC"	// PAL (B,D,G,H,I) S-Video on input 1	(N/A on SV5A,SV5B)

					// For PIXCI(R) SV7
    //#define FORMAT_UNIT0   "RS-170"	// RS-170
    //#define FORMAT_UNIT0   "NTSC"	// NTSC
    //#define FORMAT_UNIT0   "CCIR"	// CCIR
    //#define FORMAT_UNIT0   "PAL"	// PAL
    //#define FORMAT_UNIT1   "RS-170"	// RS-170
    //#define FORMAT_UNIT1   "NTSC"	// NTSC
    //#define FORMAT_UNIT1   "CCIR"	// CCIR
    //#define FORMAT_UNIT1   "PAL"	// PAL


					// For PIXCI(R) A, CL1, CL2, CL3SD, D, D24, D32,
					// D2X, D3X, E1, E1DB, E4, E4DB, EB1, EB1POCL,
					// EC1, ECB1, ECB1-34, ECB2, EL1, EL1DB, ELS2,
					// SI, SI1, SI2, SI4
    //#define FORMAT_UNIT0  "default"	// as per board's intended camera
    //#define FORMAT_UNIT1  "default"	// as per board's intended camera

					// For any PIXCI(R) imaging board
					// using a format file saved by XCAP

    #define FORMATFILE_LOAD   "xcvidset.fmt"	// loaded from file during execution
						// for FORMATFILE_LOAD and ONELIB_TWOUNIT

  //#define FORMATFILE_LOAD0	"xcvidset0.fmt" // loaded from file during execution
  //#define FORMATFILE_LOAD1	"xcvidset1.fmt" // loaded from file during execution
						// for TWOLIB_ONEUNIT
#endif
#if PXDLIB_ENHANCED
    #if !defined(FORMATFILE_LOAD)
	#error The PXDLIB_ENHANCED option requires FORMATFILE_LOAD.
    #endif
#endif
#if PXELIB_ENHANCED
#endif
#if TWOLIB_ONEUNIT
    #if defined(FORMATFILE_LOAD)
	#error The TWOLIB_ONEUNIT option does not use FORMATFILE_LOAD.
    #endif
#endif
#if ONELIB_TWOUNIT
    #if defined(FORMATFILE_LOAD0) || defined(FORMATFILE_LOAD1)
	#error The ONELIB_TWOUNIT option does not use FORMATFILE_LOAD0 or FORMATFILE_LOAD1.
    #endif
#endif


/*
 *  2.1) Number of expected PIXCI(R) image boards.
 *  Currently, this example's code only supports 1 or 2;
 *  although there are partial 'hooks' to support 1 through 4.
 *
 *  For PIXCI(R) imaging boards with multiple, functional units,
 *  the XCLIB presents the two halves of the
 *  PIXCI\*(Rg\ E1DB, E4DB, ECB2, EL1DB, ELS2, SI2, or SV7 imaging boards
 *  or the four quarters of the PIXCI\*(Rg\ SI4 imaging board
 *  as two or four independent PIXCI\*(Rg\ imaging boards, respectively.
 *
 */
#if !defined(UNITS)
    #define UNITS	2
#endif
#if !defined(UNITSMAP)
    #define UNITSMAP	((1<<UNITS)-1)	/* shorthand - bitmap of all units */
#endif



/*
 *  2.3) Optionally, set driver configuration parameters.
 *  These are normally left to the default, "".
 *  The actual driver configuration parameters include the
 *  desired PIXCI(R) imaging boards, but to make configuation easier,
 *  code, below, will automatically add board selection to this.
 */
#if !defined(DRIVERPARMS)
    //#define DRIVERPARMS "-QU 0"   // don't use interrupts
      #define DRIVERPARMS ""	    // default
#endif


/*
 *  3.1) Choose which form of image display is to be demonstrated.
 *  Some of these  options expect that the optional PXIPL library is present.
 *  Others may expect that the Windows DirectDraw SDK is present
 *  (available from Microsoft) and that the S/VGA supports DirectDraw.
 *
 *  Only one of these choices should have value 1, the others should be 0.
 */
#if !defined(SHOWIM_STRETCHDIBITS)  && !defined(SHOWIG_STRETCHDIBITS)  && !defined(SHOWIM_STRETCHDIBITS2) \
 && !defined(SHOWIG_STRETCHDIBITS2) && !defined(SHOWIG_STRETCHDIBITS3) && !defined(SHOWIM_DRAWDIBDRAW)	  \
 && !defined(SHOWIM_DRAWDIBDISPLAY) && !defined(SHOWIM_GDIDISPLAY)     && !defined(SHOWIM_GDIDISPLAY2)	  \
 && !defined(SHOWIG_GDIDISPLAY)     && !defined(SHOWIM_DIRECTXDISPLAY) && !defined(SHOWIG_DIRECTVIDEO)	  \
 && !defined(SHOWIM_DIRECTVIDEO)

    #define SHOWIM_STRETCHDIBITS    1	// use XCLIB or XCLIB-Lite and GDI
    #define SHOWIM_DRAWDIBDRAW	    0	// use XCLIB or XCLIB-Lite and Video for Windows
    #define SHOWIM_DRAWDIBDISPLAY   0	// use XCLIB and PXIPL and Video for Windows
    #define SHOWIM_GDIDISPLAY	    0	// use XCLIB and PXIPL
    #define SHOWIM_DIRECTXDISPLAY   0	// use XCLIB and PXIPL and DirectDraw
#endif


/*
 *  4)	Compile with Microsoft Visual Studio VS2010 as Win32 Platform
 *	application using command line tools:
 *
 *	    cl -I "\Program Files\EPIX\XCLIB" -c xclibex5.cpp
 *	    rc.exe /l 0x409 /fo tt.res	"\Program Files\EPIX\XCLIB\xclibex2.rc"
 *	    echo -subsystem:windows,4.0 -entry:WinMainCRTStartup comdlg32.lib ole32.lib >t1
 *	    echo /NODEFAULTLIB libc.lib oldnames.lib user32.lib gdi32.lib kernel32.lib vfw32.lib largeint.lib winmm.lib >t2
 *
 *	    Either (XCLIB w/out PXIPL for Windows NT/2000/XP/Vista/7)
 *		link xclibex5.obj "\Program Files\EPIX\XCLIB\XCLIBWNT.LIB" tt.res @t1 @t2
 *	    or	(XCLIB+PXIPL for Windows NT/2000/XP/Vista/7)
 *		link xclibex5.obj "\Program Files\EPIX\XCLIB\XCLIBWNT.LIB" "\Program Files\EPIX\XCLIB\PXIPLWNT.LIB" tt.res @t1 @t2
 *	    or (XCLIB-Lite for Windows NT/2000/XP/Vista/7)
 *		link xclibex5.obj "\Program Files\EPIX\XCLIB\XCLYBWNT.LIB" tt.res @t1 @t2
 *
 *	    del t1
 *	    del t2
 *	    del tt.res
 *	    del xclibex5.obj
 *
 *	Or, compile with Microsoft Visual Studio VS2010 as x64 Platform
 *	application using command line tools:
 *
 *	    cl -I "\Program Files\EPIX\XCLIB" -c xclibex5.cpp
 *	    rc.exe /l 0x409 /fo tt.res "\Program Files\EPIX\XCLIB\xclibex2.rc"
 *	    echo -subsystem:windows -machine:X64 -entry:WinMainCRTStartup comdlg32.lib ole32.lib >t1
 *	    echo /NODEFAULTLIB libcmt.lib oldnames.lib user32.lib gdi32.lib kernel32.lib vfw32.lib winmm.lib >t2
 *
 *	Either (XCLIB w/out PXIPL for Windows XP(x64)/Vista(x64)/7(x64)
 *	    link xclibex5.obj "\Program Files\EPIX\XCLIB\XCLIBW64.LIB" tt.res @t1 @t2
 *	or  (XCLIB+PXIPL for Windows XP(x64)/Vista(x64)/7(x64)
 *	    link xclibex5.obj "\Program Files\EPIX\XCLIB\XCLIBW64.LIB" "\Program Files\EPIX\XCLIB\PXIPLW64.LIB" tt.res @t1 @t2
 *
 *	    del t1
 *	    del t2
 *	    del tt.res
 *	    del xclibex5.obj
 *
 *    Or use the Microsoft Visual Studio VS2010 IDE, compiling xclibex5.cpp
 *    as a Windows NT/2000/XP/Vista/7 Win32 Platform or x64 Platform app,
 *    using the XCLIBEX2.RC resource, linking to library(s):
 *
 *	  for XCLIB:	    XCLIBWNT.LIB (Win32) or XCLIBW64.LIB (x64)
 *        for XCLIB+PXIPL:  XCLIBWNT.LIB PXIPLWNT.LIB (Win32) or XCLIBW64.LIB PXIPLW64.LIB (x64)
 *	  for XCLIB-Lite:   XCLYBWNT.LIB (Win32) or XCLYBW64.LIB (x64)
 *
 *    For example, in the IDE for Visual Studio 2010:
 *         1. Launch Microsoft Visual Studio 2010.
 *         2. Select 'File' -> 'New' -> 'Project'.
 *         3. Under 'Project types:', Select 'Visual C++->'Win32' and 'Win32 Project' under 'Templates',
 *            do NOT select 'Win32 Console Application'.
 *         4. Enter your Project Name and press the 'OK' button, and then the 'Finish' button.
 *         6. A window with generic example code will be displayed, replace with code from this file.
 *	   7. Copy XCLIBEX2.RC to your project directory.
 *	   8. Select 'Project' -> 'Add Existing Item' and select XCLIBEX2.RC.
 *         9. Select 'Project' and '"Project Name" Properties' or 'Properties'.
 *        10. Select 'Configuration Properties', Change 'Configuration:'
 *            selection from 'Active(debug)' to 'All Configurations' and Platform to 'All Platforms'.
 *        11. Under 'Configuration Properties', 'General' select 'Character Set' and specify 'Not Set'.
 *        12. Under 'Configuration Properties', 'VC++ Directories' select 'Additional Include Directories' and specify '\Program Files\EPIX\XCLIB'.
 *        13. Under C/C++-> General select 'Additional Include Directories' and specify '\Program Files\EPIX\XCLIB'.
 *        14. Under Resources->General select Additional Include Directories specify '\Program Files\EPIX\XCLIB'.
 *        15. Under C/C++-> Precompiled Headers select 'Create/Use Precompiled Header' and
 *	      specify 'Not Using Precompiled Headers'.
 *        16. Under Linker-> General select 'Additional Library Directories' and specify '\Program Files\EPIX\XCLIB'.
 *        17. For Win32 target platforms, Select 'Configuration Properties', Change 'Configuration:'
 *            selection to 'All Configurations' and Platform to 'Win32'.
 *	      Under Linker-> Input-> Additional Dependencies add
 *		for XCLIB:	  XCLIBWNT.LIB
 *              for XCLIB+PXIPL:  XCLIBWNT.LIB PXIPLWNT.LIB
 *              for XCLIB-Lite:   XCLYBWNT.LIB XCLYBWNT.LIB
 *            to the list of modules already there. Be certain to enter
 *            a space between your entry and the adjacent name.
 *        18. For x64 target platforms, Select 'Configuration Properties', Change 'Configuration:'
 *            selection to 'All Configurations' and Platform to 'x64'. If x64 is not selectable,
 *	      use Configuration Manager to create a new 'Active solution platform'
 *	      and select x64 as the target and set 'Copy settings from:' to 'Win32'.
 *	      Under Linker-> Input-> Additional Dependencies add
 *		for XCLIB:	  XCLIBW64.LIB
 *              for XCLIB+PXIPL:  XCLIBW64.LIB PXIPLW64.LIB
 *              for XCLIB-Lite:   XCLYBW64.LIB
 *            to the list of modules already there and remove any XCLIB Win32 entries. Be certain to enter
 *            a space between your entry and the adjacent name.
 *        19. Click on OK.
 *        20. Copy the file(s)
 *	      for XCLIB:	XCLIBWNT.DLL or XCLIBW64.DLL
 *	      for XCLIB+PXIPL:	XCLIBWNT.DLL PXIPLWNT.DLL or XCLIBW64.DLL PXIPLW64.DLL
 *	      for XCLIB-Lite:	XCLYBWNT.DLL or XCLYBW64.DLL
 *	      to this project's directory.
 *	  21. Press F5 to build and run program.
 *
 *
 *  For those using with Borland, Watcom, Delphi, or older Microsoft compilers:
 *  The XCLIBEX2.CPP example provides additional compilation instructions
 *  and hints which can be also applied to this example, replacing
 *  'XCLIBEX2.CPP' and XCLIBEX2.RC with 'XCLIBEX5.CPP' and 'XCLIBEX2.RC'.
 *
 */





/*
 *  NECESSARY INCLUDES:
 */
#include <windows.h>
#include <windowsx.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#if SHOWIM_DRAWDIBDRAW || SHOWIM_DRAWDIBDISPLAY
    #include <vfw.h>
#endif
#if SHOWIM_DIRECTXDISPLAY
    #include <compobj.h>
    #include <ddraw.h>
#endif

extern "C" {
    #include "xcliball.h"
    #include "xclibex2.h"
    #if SHOWIM_GDIDISPLAY || SHOWIM_DIRECTXDISPLAY || SHOWIM_DRAWDIBDISPLAY || USE_PXIPL
	#include "pxipl.h"         
	#include "pximages.h"           
    #endif
}

/*
 * Global variables.
 */
static	HWND	hWnd;	    /* the main window */
static	HWND	hDlg;	    /* the main dialog */
#if SHOWIM_DRAWDIBDRAW || SHOWIM_DRAWDIBDISPLAY
    static HDRAWDIB	hDrawDib = NULL;    /* VFW handle */
#endif
#if SHOWIM_DIRECTXDISPLAY
    static LPDIRECTDRAW lpDD = NULL;
    static HINSTANCE	hDDLibrary = NULL;  /* DDraw handles */
#endif

/*
 * Utility
 */
void mesg(const char *format, ...)
{
    va_list argp;
    char    *buf;
    #define BUFSIZE 1024

    if (!(buf = (char *)malloc(BUFSIZE))) {
	MessageBox(NULL, "No memory!", "XCLIBEX5", MB_OK|MB_TASKMODAL);
	return;
    }
    va_start(argp, format);
    _vsnprintf(buf, BUFSIZE, format, argp);
    buf[BUFSIZE-1] = 0; // this & snprintf: overly conservative - avoids warning messages
    va_end(argp);
    MessageBox(NULL, buf, "XCLIBEX5", MB_OK|MB_TASKMODAL);
    free(buf);
}

/*
 * Display specified buffer from specified unit,
 * in specified AOI of specified HWND,
 * using a compile-time selected method.
 */
#if ONELIB_TWOUNIT || TWOLIB_ONEUNIT
void DisplayBuffer(xclibs_s *xclib, int stateid, int unit, pxbuffer_t buf, HWND hWndImage, struct pxywindow windImage)
{
    HDC     hDC;
    int     err = 0;

    hDC = GetDC(hWndImage);

    //
    // Show image using pxd_renderStretchDIBits.
    //
    #if SHOWIM_STRETCHDIBITS
	SetStretchBltMode(hDC, STRETCH_DELETESCANS);
	err = xclib->pxaux.StretchDIBits(&xclib->pxaux, 0, 1<<unit, stateid, buf, 0, 0, -1, -1, 0,
			 hDC, windImage.nw.x, windImage.nw.y,
			 windImage.se.x-windImage.nw.x,
			 windImage.se.y-windImage.nw.y);
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxaux.StretchDIBits", MB_OK|MB_TASKMODAL);
    #endif

    //
    // Show image using PXIPL pxio8_GDIDisplay.
    // Altho not used in this example, pxio8_GDIDisplay also allows drawing
    // a full screen cross hair cursor over the image.
    //
    #if SHOWIM_GDIDISPLAY
    {
	pximage_s   image[PXMAX_FIRBGF];
	xclib->pxlib.initFilteredPximage(&xclib->pxlib,1<<unit,image,PXMAX_FIRBGF,PXHINTBGR,0,stateid,buf, 0, 0x7);
	err = pxio8_GDIDisplay(NULL, image,
			 NULL, 0, 'n', 0, 0,hDC,&windImage, NULL, NULL);
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_GDIDisplay", MB_OK|MB_TASKMODAL);
    }
    #endif

    //
    // Show image using DirectDraw.
    // Note that this only supports S/VGA in
    // 24 or 32 bits per pixel mode.
    // This might be slightly quicker if the primary surface
    // were acquired once, and kept till the program exits.
    //
    // This can be extended to draw graphics in
    // the same manner as for pxio8_GDIDisplay, above.
    //
    // The pixel format of the S/VGA may not be acceptable!
    // Error reporting should be added!
    //
    #if SHOWIM_DIRECTXDISPLAY
    {
	pximage_s   image[PXMAX_FIRBGF];
	DDSURFACEDESC	surfacedesc;
	LPDIRECTDRAWSURFACE ddrs = NULL;
	HRESULT     h;
	POINT	    pt;
	if (lpDD) {
	    //
	    // Get primary surface of full screen.
	    //
	    surfacedesc.dwFlags = DDSD_CAPS;
	    surfacedesc.ddsCaps.dwCaps = DDSCAPS_PRIMARYSURFACE|DDSCAPS_VIDEOMEMORY;
	    surfacedesc.dwSize = sizeof(surfacedesc);
	    h = lpDD->CreateSurface(&surfacedesc, &ddrs, NULL);
	    if (ddrs) {
		//
		// Adjust for position of dialog on screen.
		//
		pt.x = pt.y = 0;
		ClientToScreen(hWndImage, &pt);
		windImage.nw.x += pt.x;
		windImage.nw.y += pt.y;
		windImage.se.x += pt.x;
		windImage.se.y += pt.y;
		xclib->pxlib.initFilteredPximage(&xclib->pxlib,1<<unit,image,PXMAX_FIRBGF,PXHINTBGR,0,stateid,buf, 0, 0x7);
		err = pxio8_DirectXDisplay(NULL, image,
				     NULL, 0, 'n', 0, 0, ddrs, &windImage, NULL, hDC, NULL, NULL);
		windImage.nw.x -= pt.x;
		windImage.nw.y -= pt.y;
		windImage.se.x -= pt.x;
		windImage.se.y -= pt.y;
		((LPDIRECTDRAWSURFACE)ddrs)->Release();
	    }
	}
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_DirectXDisplay", MB_OK|MB_TASKMODAL);
    }
    #endif

    //
    // Show image using pxd_renderDIBCreate to create a
    // standard Windows DIB, and display with Video for Windows.
    //
    #if SHOWIM_DRAWDIBDRAW
    {
	BITMAPINFOHEADER FAR *dib;
	HGLOBAL hDIB = NULL;

	xclib->pxaux.makeDIB(&xclib->pxaux, 0, 1<<unit, stateid, buf, 0, 0, -1, -1, &hDIB);
	if (hDIB) {
	    if (dib = (BITMAPINFOHEADER FAR *)GlobalLock(hDIB)) {
		DrawDibDraw(hDrawDib, hDC, windImage.nw.x, windImage.nw.y,
					   windImage.se.x-windImage.nw.x, windImage.se.y-windImage.nw.y,
			    (BITMAPINFOHEADER *)dib,
			    (uchar FAR*)dib+dib->biSize+dib->biClrUsed*sizeof(RGBQUAD),
			    0, 0, pxd_imageXdim(), pxd_imageYdim(), 0);
		GlobalUnlock(hDIB);
	    }
	    xclib->pxaux.freeDIB(&xclib->pxaux, 0, 0, stateid, 0, 0, 0, 0, 0, &hDIB);
	} else
	    MessageBox(NULL, "Error", "pxaux.makeDIB", MB_OK|MB_TASKMODAL);
    }
    #endif

    //
    // Show image using PXIPL and Video for Windows.
    // Altho not used in this example, pxio8_DrawDibDisplay also allows drawing
    // a full screen cross hair cursor over the image.
    //
    #if SHOWIM_DRAWDIBDISPLAY
    {
	pximage_s   image[PXMAX_FIRBGF];
	xclib->pxlib.initFilteredPximage(&xclib->pxlib,1<<unit,image,PXMAX_FIRBGF,PXHINTBGR,0,stateid,buf, 0, 0x7);
	err = pxio8_DrawDibDisplay(NULL, image,
			 NULL, 0, 'n', 0, 0,hDrawDib,hDC,&windImage, NULL, NULL);
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_DrawDibDisplay", MB_OK|MB_TASKMODAL);
    }
    #endif

    ReleaseDC(hWndImage, hDC);
}
#endif

#if PXDLIB_ENHANCED
void DisplayBuffer(int unit, pxbuffer_t buf, HWND hWndImage, struct pxywindow windImage[])
{
    HDC     hDC;
    int     err = 0;

    hDC = GetDC(hWndImage);

    //
    // Show image using pxd_renderStretchDIBits.
    //
    #if SHOWIM_STRETCHDIBITS
	SetStretchBltMode(hDC, STRETCH_DELETESCANS);
	err = pxd_renderStretchDIBits(1<<unit, buf, 0, 0, -1, -1, 0,
			 hDC, windImage[unit].nw.x, windImage[unit].nw.y,
			 windImage[unit].se.x-windImage[unit].nw.x,
			 windImage[unit].se.y-windImage[unit].nw.y, 0);
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_renderStretchDIBits", MB_OK|MB_TASKMODAL);
    #endif

    //
    // Show image using PXIPL pxio8_GDIDisplay.
    // Altho not used in this example, pxio8_GDIDisplay also allows drawing
    // a full screen cross hair cursor over the image.
    //
    #if SHOWIM_GDIDISPLAY
	err = pxio8_GDIDisplay(NULL, pxd_defineImage(1<<unit, buf, 0, 0, -1, -1, "Display"),
			 NULL, 0, 'n', 0, 0,hDC,&windImage[unit], NULL, NULL);
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_GDIDisplay", MB_OK|MB_TASKMODAL);
    #endif

    //
    // Show image using DirectDraw.
    // Note that this only supports S/VGA in
    // 24 or 32 bits per pixel mode.
    // This might be slightly quicker if the primary surface
    // were acquired once, and kept till the program exits.
    //
    // This can be extended to draw graphics in
    // the same manner as for pxio8_GDIDisplay, above.
    //
    // The pixel format of the S/VGA may not be acceptable!
    // Error reporting should be added!
    //
    #if SHOWIM_DIRECTXDISPLAY
    {
	DDSURFACEDESC	surfacedesc;
	LPDIRECTDRAWSURFACE ddrs = NULL;
	HRESULT     h;
	POINT	    pt;
	if (lpDD) {
	    //
	    // Get primary surface of full screen.
	    //
	    surfacedesc.dwFlags = DDSD_CAPS;
	    surfacedesc.ddsCaps.dwCaps = DDSCAPS_PRIMARYSURFACE|DDSCAPS_VIDEOMEMORY;
	    surfacedesc.dwSize = sizeof(surfacedesc);
	    h = lpDD->CreateSurface(&surfacedesc, &ddrs, NULL);
	    if (ddrs) {
		//
		// Adjust for position of dialog on screen.
		//
		pt.x = pt.y = 0;
		ClientToScreen(hWndImage, &pt);
		windImage[unit].nw.x += pt.x;
		windImage[unit].nw.y += pt.y;
		windImage[unit].se.x += pt.x;
		windImage[unit].se.y += pt.y;
		err = pxio8_DirectXDisplay(NULL, pxd_defineImage(1<<unit, buf, 0, 0, -1, -1, "Display"),
				     NULL, 0, 'n', 0, 0, ddrs, &windImage[unit], NULL, hDC, NULL, NULL);
		windImage[unit].nw.x -= pt.x;
		windImage[unit].nw.y -= pt.y;
		windImage[unit].se.x -= pt.x;
		windImage[unit].se.y -= pt.y;
		((LPDIRECTDRAWSURFACE)ddrs)->Release();
	    }
	}
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_DirectXDisplay", MB_OK|MB_TASKMODAL);
    }
    #endif

    //
    // Show image using pxd_renderDIBCreate to create a
    // standard Windows DIB, and display with Video for Windows.
    //
    #if SHOWIM_DRAWDIBDRAW
	BITMAPINFOHEADER FAR *dib;
	HGLOBAL hDIB;

	hDIB = pxd_renderDIBCreate(1<<unit, buf, 0, 0, -1, -1, 0, 0);
	if (hDIB) {
	    if (dib = (BITMAPINFOHEADER FAR *)GlobalLock(hDIB)) {
		DrawDibDraw(hDrawDib, hDC, windImage[unit].nw.x, windImage[unit].nw.y,
					   windImage[unit].se.x-windImage[unit].nw.x, windImage[unit].se.y-windImage[unit].nw.y,
			    (BITMAPINFOHEADER *)dib,
			    (uchar FAR*)dib+dib->biSize+dib->biClrUsed*sizeof(RGBQUAD),
			    0, 0, pxd_imageXdim(), pxd_imageYdim(), 0);
		GlobalUnlock(hDIB);
	    }
	    pxd_renderDIBFree(hDIB);
	} else
	    MessageBox(NULL, "Error", "pxd_renderDIBCreate", MB_OK|MB_TASKMODAL);

    #endif

    //
    // Show image using PXIPL and Video for Windows.
    // Altho not used in this example, pxio8_DrawDibDisplay also allows drawing
    // a full screen cross hair cursor over the image.
    //
    #if SHOWIM_DRAWDIBDISPLAY
	err = pxio8_DrawDibDisplay(NULL, pxd_defineImage(1<<unit, buf, 0, 0, -1, -1, "Display"),
			 NULL, 0, 'n', 0, 0,hDrawDib,hDC,&windImage[unit], NULL, NULL);
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_DrawDibDisplay", MB_OK|MB_TASKMODAL);
    #endif

    ReleaseDC(hWndImage, hDC);
}
#endif

#if PXELIB_ENHANCED
void DisplayBuffer(pxdstate_s *pxdstatep, int unit, pxbuffer_t buf, HWND hWndImage, struct pxywindow windImage[])
{
    HDC     hDC;
    int     err = 0;

    hDC = GetDC(hWndImage);

    //
    // Show image using pxe_renderStretchDIBits.
    //
    #if SHOWIM_STRETCHDIBITS
	SetStretchBltMode(hDC, STRETCH_DELETESCANS);
	err = pxe_renderStretchDIBits(pxdstatep, 1<<unit, buf, 0, 0, -1, -1, 0,
			 hDC, windImage[unit].nw.x, windImage[unit].nw.y,
			 windImage[unit].se.x-windImage[unit].nw.x,
			 windImage[unit].se.y-windImage[unit].nw.y, 0);
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxe_renderStretchDIBits", MB_OK|MB_TASKMODAL);
    #endif

    //
    // Show image using PXIPL pxio8_GDIDisplay.
    // Altho not used in this example, pxio8_GDIDisplay also allows drawing
    // a full screen cross hair cursor over the image.
    //
    #if SHOWIM_GDIDISPLAY
	err = pxio8_GDIDisplay(NULL, pxe_defineImage(pxdstatep, 1<<unit, buf, 0, 0, -1, -1, "Display"),
			 NULL, 0, 'n', 0, 0,hDC,&windImage[unit], NULL, NULL);
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_GDIDisplay", MB_OK|MB_TASKMODAL);
    #endif

    //
    // Show image using DirectDraw.
    // Note that this only supports S/VGA in
    // 24 or 32 bits per pixel mode.
    // This might be slightly quicker if the primary surface
    // were acquired once, and kept till the program exits.
    //
    // This can be extended to draw graphics in
    // the same manner as for pxio8_GDIDisplay, above.
    //
    // The pixel format of the S/VGA may not be acceptable!
    // Error reporting should be added!
    //
    #if SHOWIM_DIRECTXDISPLAY
    {
	DDSURFACEDESC	surfacedesc;
	LPDIRECTDRAWSURFACE ddrs = NULL;
	HRESULT     h;
	POINT	    pt;
	if (lpDD) {
	    //
	    // Get primary surface of full screen.
	    //
	    surfacedesc.dwFlags = DDSD_CAPS;
	    surfacedesc.ddsCaps.dwCaps = DDSCAPS_PRIMARYSURFACE|DDSCAPS_VIDEOMEMORY;
	    surfacedesc.dwSize = sizeof(surfacedesc);
	    h = lpDD->CreateSurface(&surfacedesc, &ddrs, NULL);
	    if (ddrs) {
		//
		// Adjust for position of dialog on screen.
		//
		pt.x = pt.y = 0;
		ClientToScreen(hWndImage, &pt);
		windImage[unit].nw.x += pt.x;
		windImage[unit].nw.y += pt.y;
		windImage[unit].se.x += pt.x;
		windImage[unit].se.y += pt.y;
		pxio8_DirectXDisplay(NULL, pxe_defineImage(pxdstatep, 1<<unit, buf, 0, 0, -1, -1, "Display"),
				     NULL, 0, 'n', 0, 0, ddrs, &windImage[unit], NULL, hDC, NULL, NULL);
		windImage[unit].nw.x -= pt.x;
		windImage[unit].nw.y -= pt.y;
		windImage[unit].se.x -= pt.x;
		windImage[unit].se.y -= pt.y;
		((LPDIRECTDRAWSURFACE)ddrs)->Release();
	    }
	}
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_DirectXDisplay", MB_OK|MB_TASKMODAL);
    }
    #endif

    //
    // Show image using pxe_renderDIBCreate to create a
    // standard Windows DIB, and display with Video for Windows.
    //
    #if SHOWIM_DRAWDIBDRAW
	BITMAPINFOHEADER FAR *dib;
	HGLOBAL hDIB;

	hDIB = pxe_renderDIBCreate(pxdstatep, 1<<unit, buf, 0, 0, -1, -1, 0, 0);
	if (hDIB) {
	    if (dib = (BITMAPINFOHEADER FAR *)GlobalLock(hDIB)) {
		DrawDibDraw(hDrawDib, hDC, windImage[unit].nw.x, windImage[unit].nw.y,
					   windImage[unit].se.x-windImage[unit].nw.x, windImage[unit].se.y-windImage[unit].nw.y,
			    (BITMAPINFOHEADER *)dib,
			    (uchar FAR*)dib+dib->biSize+dib->biClrUsed*sizeof(RGBQUAD),
			    0, 0, pxe_imageXdim(pxdstatep), pxe_imageYdim(pxdstatep), 0);
		GlobalUnlock(hDIB);
	    }
	    pxe_renderDIBFree(pxdstatep, hDIB);
	} else
	    MessageBox(NULL, "Error", "pxe_renderDIBCreate", MB_OK|MB_TASKMODAL);

    #endif

    //
    // Show image using PXIPL and Video for Windows.
    // Altho not used in this example, pxio8_DrawDibDisplay also allows drawing
    // a full screen cross hair cursor over the image.
    //
    #if SHOWIM_DRAWDIBDISPLAY
	pxio8_DrawDibDisplay(NULL, pxe_defineImage(pxdstatep, 1<<unit, buf, 0, 0, -1, -1, "Display"),
			 NULL, 0, 'n', 0, 0,hDrawDib,hDC,&windImage[unit], NULL, NULL);
	if (err < 0)
	    MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_DrawDibDisplay", MB_OK|MB_TASKMODAL);
    #endif

    ReleaseDC(hWndImage, hDC);
}
#endif



/*
 * The Dialog
 */
BOOL CALLBACK
PIXCIDialogProc(HWND hDlg, UINT wMsg, WPARAM wParam, LPARAM lParam)
{
    #if ONELIB_TWOUNIT || TWOLIB_ONEUNIT
	static	xclibs_s    xclib[max(4,UNITS)] = { {0},{0},{0},{0} };
							    // library instances
    #endif

    static  UINT	svgaBits;			    // pixel format of S/VGA
    static  int 	liveon = 0;
    static  int 	isopen = 0;
    static  pxvbtime_t	lastcapttime[UNITS] = {0};	    // when was image last captured
    static  struct	pxywindow windImage[max(4,UNITS)];  // subwindow of child window for image display
    static  struct	pxy	  dimImage[max(4,UNITS)];   // image dimensions
    static		double	  aspectImage[max(4,UNITS)];// pixel aspect ratio
    static		int	  pixiesImage[max(4,UNITS)];//
    static		int	  bitsImage[max(4,UNITS)];
    static  HWND	hWndImage;			    // child window of dialog for image display
    static		pxdstate_s *pxdstates[max(4,UNITS)];
	    int 	u, r = 0;
	    int 	err = 0;

    //
    // To cleanly convert code between ONELIB_TWOUNIT and TWOLIB_ONEUNIT modes.
    // In the former, there is one instance of xclib_s, and a 'unitmap'
    // parameter to various API calls selects a unit.
    // In the latter, there are multiple instances of xclib_s, and the
    // 'unitmap' parameter is always 0x1.
    //
    // N.B. The -DM parameter to xclib_open is different than the 'unitmap'
    // API parameter. The former selects one or more physical boards, the
    // latter selects one or more 'logical' boards relative to those opened
    // via that xclib_s instance.
    //
    #if ONELIB_TWOUNIT || TWOLIB_ONEUNIT
	static	int	    unitselect	 [max(4,UNITS)];
	static	xclibs_s    *xclibselect [max(4,UNITS)];
	static	int	    stateidselect[max(4,UNITS)];
    #endif


    switch (wMsg) {
      case WM_INITDIALOG:
      {
	RECT	rectImage;

	//
	// Open the PIXCI(R) imaging boards.
	//
	// Either FORMAT_UNIT? or FORMATFILE_LOAD
	// should have been selected above.
	//
	#if ONELIB_TWOUNIT
	    memset(&xclib[0], 0, sizeof(xclib[0]));
	    xclib[0].ddch.len = sizeof(xclib[0]);
	    xclib[0].ddch.mos = XCMOS_LIBS;
	    char driverparms[80];
	    driverparms[sizeof(driverparms)-1] = 0; // this & snprintf: overly conservative - avoids warning messages
	    _snprintf(driverparms, sizeof(driverparms)-1, "-DM 0x%x %s", (1<<UNITS)-1, DRIVERPARMS);
	    #if defined(FORMAT_UNIT0)
		if ((r = xclib_open(&xclib[0], NULL, driverparms, FORMAT_UNIT0, NULL)) >= 0)
		    isopen++;
		else if (xclib[0].pxaux.faultMessageBox)
		    xclib[0].pxaux.faultMessageBox(&xclib[0].pxaux, 0, (1<<UNITS)-1, NULL, NULL, NULL, 0);
		else
		    mesg("xclib_open: Error=%d", r);
		if (!isopen)
		    return(TRUE);
		//
		// The xclib_open will have set the PXMODE_DIGI+0 as per FORMAT_UNIT0,
		// we now must create a PXMODE_DIGI+1 as per FORMAT_UNIT1.
		//
		#if UNITS > 1
		    pxvidstate_s *statep = NULL;
		    pxdevinfo_s  info;
		    memset(&info, 0, sizeof(info));
		    info.ddch.len  = sizeof(info);
		    info.ddch.mos = PXMOS_DEVINFO;

		    if ((r = xclib[0].pxdev.getDevInfo(&xclib[0].pxdev, 1<<1, 0, &info)) < 0
		     || (r = xclib[0].pxlib.allocStateCopy(&xclib[0].pxlib, 0, 0, &statep)) < 0
		     || (r = xclib[0].pxlib.initStateCopy(&xclib[0].pxlib, 0,0,statep,&info,FORMAT_UNIT1,PXMODE_DIGI)) < 0
		     || (r = xclib[0].pxlib.defineState(&xclib[0].pxlib,0,PXMODE_DIGI+1,statep)) < 0) {
			mesg("initStateCopy: Unit=%d Error=%d", 1, r);
		    }
		    xclib[0].pxlib.freeStateCopy(&xclib[0].pxlib, 0, 0, &statep);
		#endif

	    #endif
	    #if defined(FORMATFILE_LOAD)
		//
		// This is expected to load a format file saved by XCAP
		// which was set for individual controls for each of the
		// units. Thus, the format file defines PXMODE_DIGI+0,
		//  PXMODE_DIGI+1, etc.
		//
		if ((r = xclib_open(&xclib[0], NULL, driverparms, NULL, FORMATFILE_LOAD)) >= 0)
		    isopen++;
		else if (xclib[0].pxaux.faultMessageBox)
		    xclib[0].pxaux.faultMessageBox(&xclib[0].pxaux, 0, (1<<UNITS)-1, NULL, NULL, NULL, 0);
		else
		    mesg("xclib_open: Error=%d", r);
	    #endif
	    if (!isopen)
		return(TRUE);

	    //
	    // Init boards' configuration as per video states.
	    //
	    for (u = 0; u < UNITS; u++) {
		if ((r = xclib[0].xcdev.setVideoConfig(&xclib[0].xcdev, 1<<u, 0, PXMODE_DIGI+u, NULL, NULL)) < 0)
		    mesg("setVideoConfig: Unit=%d Error=%d", u, r);
		if ((r = xclib[0].xcdev.setCameraConfig(&xclib[0].xcdev, 1<<u, 1, PXMODE_DIGI+u, NULL, NULL)) < 0)
		    if (r !=  PXERNOFEATURE)
			mesg("setCameraConfig: Unit=%d Error=%d", u, r);
	    }

	    //
	    // Setup for cleaner common code able to work in both
	    // ONELIB_TWOUNIT and TWOLIB_ONEUNIT modes.
	    //
	    xclibselect[0] = xclibselect[1] =
	    xclibselect[2] = xclibselect[3] = &xclib[0];
	    unitselect[0] = 0;
	    unitselect[1] = 1;
	    unitselect[2] = 2;
	    unitselect[3] = 3;
	    stateidselect[0] = PXMODE_DIGI+0;
	    stateidselect[1] = PXMODE_DIGI+1;
	    stateidselect[2] = PXMODE_DIGI+2;
	    stateidselect[3] = PXMODE_DIGI+3;
	#endif

	#if TWOLIB_ONEUNIT
	    for (u = 0; u < UNITS; u++) {
		//
		// Open
		// This example doesn't currently demonstrate FORMATFILE_LOAD
		// under the TWOLIB_ONEUNIT option.
		//
		memset(&xclib[u], 0, sizeof(xclib[u]));
		xclib[u].ddch.len = sizeof(xclib[u]);
		xclib[u].ddch.mos = XCMOS_LIBS;
		char driverparms[80];
		driverparms[sizeof(driverparms)-1] = 0; // this & snprintf: overly conservative - avoids warning messages
		_snprintf(driverparms, sizeof(driverparms)-1, "-DM 0x%x %s", 1<<u, DRIVERPARMS);
		#if defined(FORMAT_UNIT0)
		    r = xclib_open(&xclib[u], NULL, driverparms, u==0? FORMAT_UNIT0: FORMAT_UNIT1, NULL);
		#else
		    r = xclib_open(&xclib[u], NULL, driverparms, NULL, u==0? FORMATFILE_LOAD0: FORMATFILE_LOAD1);
		#endif
		if (r >= 0)
		    isopen++;
		else if (xclib[u].pxaux.faultMessageBox) {
		    xclib[u].pxaux.faultMessageBox(&xclib[u].pxaux, 0, 0x1, NULL, NULL, NULL, 0);
		    return(TRUE);
		} else {
		    mesg("xclib_open: Error=%d", r);
		    return(TRUE);
		}
		//
		// Init boards' configuration as per video states.
		//
		if ((r = xclib[u].xcdev.setVideoConfig(&xclib[u].xcdev, 0x1, 0, PXMODE_DIGI, NULL, NULL)) < 0)
		    mesg("setVideoConfig: Unit=%d Error=%d", u, r);
		if ((r = xclib[u].xcdev.setCameraConfig(&xclib[u].xcdev, 0x1, 1, PXMODE_DIGI, NULL, NULL)) < 0)
		    if (r != PXERNOFEATURE)
			mesg("setCameraConfig: Unit=%d Error=%d", u, r);
	    }

	    //
	    // Setup for cleaner, common code able to work in both
	    // ONELIB_TWOUNIT and TWOLIB_ONEUNIT modes.
	    //
	    xclibselect[0] = &xclib[0];
	    xclibselect[1] = &xclib[1];
	    xclibselect[2] = &xclib[2];
	    xclibselect[3] = &xclib[3];
	    unitselect[0] = unitselect[1] =
	    unitselect[2] = unitselect[3] = 0;
	    stateidselect[0] = stateidselect[1] =
	    stateidselect[2] = stateidselect[3] = PXMODE_DIGI+0;
	#endif

	#if PXDLIB_ENHANCED
	    //
	    // This example currently demonstrate only FORMATFILE_LOAD
	    // under the PXDLIB_ENHANCED option.  FORMAT_UNIT? options
	    // are not applicable; only with FORMATFILE_LOAD
	    // using a format file saved by XCAP w. its
	    // "Use Individual Formats & Controls" option
	    // would the enhanced SCF functions recognize
	    // that N different resolutions are in effect.
	    //
	    char driverparms[80];
	    driverparms[sizeof(driverparms)-1] = 0; // this & snprintf: overly conservative - avoids warning messages
	    _snprintf(driverparms, sizeof(driverparms)-1, "-DM 0x%x %s", UNITSMAP, DRIVERPARMS);
	    if (pxd_PIXCIopen(driverparms, "", FORMATFILE_LOAD) < 0)
		pxd_mesgFault(UNITSMAP);
	#endif

	#if PXELIB_ENHANCED
	    for (u = 0; u < UNITS; u++) {
		char driverparms[80];
		char *formatname = "";
		char *formatfile = "";
		#if defined(FORMATFILE_LOAD)
		    formatfile = FORMATFILE_LOAD;
		#endif
		#if defined(FORMATFILE_LOAD0)
		    if (u == 0)
			formatfile = FORMATFILE_LOAD0;
		#endif
		#if defined(FORMATFILE_LOAD1)
		    if (u == 1)
			formatfile = FORMATFILE_LOAD1;
		#endif
		#if defined(FORMAT_UNIT0)
		    if (u == 0)
			formatname = FORMAT_UNIT0;
		#endif
		#if defined(FORMAT_UNIT1)
		    if (u == 1)
			formatname = FORMAT_UNIT1;
		#endif
		driverparms[sizeof(driverparms)-1] = 0; // this & snprintf: overly conservative - avoids warning messages
		_snprintf(driverparms, sizeof(driverparms)-1, "-DM 0x%x %s", 1<<u, DRIVERPARMS);
		pxdstates[u] = pxe_XCLIBinstantiate();
		if (pxe_PIXCIopen(pxdstates[u], driverparms, formatname, formatfile) < 0)
		    pxe_mesgFault(pxdstates[u], UNITSMAP);
	    }
	#endif



	//
	// Set our title.
	//
	SetWindowText(hDlg, "EPIX(R) PIXCI(R) Imaging Board Example");

	//
	// Enable timer, for live video updates, checking for faults,
	// and timed display fo sequences.
	// See xclibex2.cpp for an alternate, using an Event
	// instead of a timer.
	//
	SetTimer(hDlg, 1, 5, NULL);

	//
	// Get handle to image display area of dialog,
	// then get its device context and size.
	//
	hWndImage = GetDlgItem(hDlg, IDIMAGE);
	{
	    HDC  hDC = GetDC(hWndImage);
	    GetClientRect(hWndImage, &rectImage);
	    svgaBits = GetDeviceCaps(hDC, PLANES) * GetDeviceCaps(hDC, BITSPIXEL);
	    ReleaseDC(hWndImage, hDC);
	}

	//
	// Partition display area for the units.
	//
	windImage[0].nw.x = windImage[0].nw.y = 0;
	windImage[0].se.x = rectImage.right+1;	     // inclusive->exclusive
	windImage[0].se.y = rectImage.bottom+1;      // inclusive->exclusive
	if (UNITS == 1) {
	    ;
	} else if (UNITS == 2) {
	    windImage[1] = windImage[0];
	    windImage[0].se.y /= 2;			// top half
	    windImage[1].nw.y = windImage[1].se.y/2;	// bottom half
	} else {
	    windImage[1] = windImage[0];
	    windImage[2] = windImage[0];
	    windImage[3] = windImage[0];
	    windImage[0].se.x /= 2;
	    windImage[0].se.y /= 2;			// upper left quadrant
	    windImage[1].nw.x = windImage[1].se.x/2;
	    windImage[1].se.y /= 2;			// upper right quadrant
	    windImage[2].se.x /= 2;
	    windImage[2].nw.y = windImage[2].se.y/2;	// lower left quadrant
	    windImage[3].nw.x = windImage[3].se.x/2;
	    windImage[3].nw.y = windImage[3].se.y/2;	// lower right quadrant
	}

	//
	// Determine displayed size.
	// We could simply fill up the hWndImage, but
	// much rather adjust the displayed image for
	// correct aspect ratio.
	//
	for (u = 0; u < UNITS; u++) {
	    #if ONELIB_TWOUNIT || TWOLIB_ONEUNIT
		pximage_s   image[PXMAX_IRBGF];
		if ((r = xclibselect[u]->pxlib.initPximage(&xclibselect[u]->pxlib, 1, image, PXMAX_IRBGF, PXHINTNONE, 0, stateidselect[u], 1, 0)) < 0)
		    mesg("initPximage: Error=%d", r);
		else {
		    if (image[0].h.pixelwidth != 0)
			aspectImage[u] = image[0].h.pixelheight/image[0].h.pixelwidth;
		    else
			aspectImage[u] = 1.0;
		    dimImage[u].x  = image[0].imdim.se.x;
		    dimImage[u].y  = image[0].imdim.se.y;
		    bitsImage[u]   = image[0].d.u.i.bitsused;
		    pixiesImage[u] = image[0].d.pixies;
		}
	    #endif
	    #if PXDLIB_ENHANCED
		aspectImage[u] = pxd_imageAspectRatios(1<<u)==0? 1.0: pxd_imageAspectRatios(1<<u);
		dimImage[u].x  = pxd_imageXdims(1<<u);
		dimImage[u].y  = pxd_imageYdims(1<<u);
		bitsImage[u]   = pxd_imageBdims(1<<u);
		pixiesImage[u] = pxd_imageCdims(1<<u);
	    #endif
	    #if PXELIB_ENHANCED
		aspectImage[u] = pxe_imageAspectRatios(pxdstates[u], 1)==0? 1.0: pxe_imageAspectRatios(pxdstates[u], 1);
		dimImage[u].x  = pxe_imageXdims(pxdstates[u], 1);
		dimImage[u].y  = pxe_imageYdims(pxdstates[u], 1);
		bitsImage[u]   = pxe_imageBdims(pxdstates[u], 1);
		pixiesImage[u] = pxe_imageCdims(pxdstates[u], 1);
	    #endif
	    {
		double	scalex, scaley;
		scalex = (windImage[u].se.x-windImage[u].nw.x)/(double)dimImage[u].x;
		scaley = (windImage[u].se.y-windImage[u].nw.y)/((double)dimImage[u].y*aspectImage[u]);
		scalex = min(scalex, scaley);
		windImage[u].se.x = windImage[u].nw.x + (int)(dimImage[u].x * scalex);
		windImage[u].se.y = windImage[u].nw.y + (int)(dimImage[u].y * scalex * aspectImage[u]);
		//
		// As long as we have image dimensions, display them
		// so as to show the user that the boards are operating
		// at the desired, and perhaps different, resolution.
		//
		mesg("Unit %d: xdim=%d ydim=%d bitdepth=%d pixies=%d", u, dimImage[u].x, dimImage[u].y, bitsImage[u], pixiesImage[u]);
	    }
	}

	//
	// If StrecthDIBits is to be used, some VGA card drivers
	// abhore horizontal dimensions which are not a multiple of 4.
	// This isn't needed for other rendering methods, but doesn't hurt.
	//
	for (u = 0; u < UNITS; u++) {
	    windImage[u].se.x = windImage[u].nw.x + ((windImage[u].se.x-windImage[u].nw.x)&~3);
	}

	//
	// If using DirectDraw, initialize access to it.
	//
	// DirectDraw may not be available!
	// Error reporting should be added!
	//
	#if SHOWIM_DIRECTXDISPLAY
	{
	    HRESULT	    h;
	    hDDLibrary = LoadLibrary("DDRAW");
	    if (hDDLibrary) {
		typedef HRESULT (WINAPI* OPEN)(void FAR*,LPDIRECTDRAW FAR*, void FAR*);
		OPEN	lpfnDM;
		lpfnDM = (OPEN)GetProcAddress(hDDLibrary, "DirectDrawCreate");
		if (lpfnDM) {
		    h = (*lpfnDM)(NULL, &lpDD, NULL);
		    if (lpDD) {
			h = lpDD->SetCooperativeLevel((HWND)hWnd, DDSCL_NORMAL);
		    }
		}
	    }
	}
	#endif

	//
	// If using Video for Windows, initialize access to it.
	//
	#if SHOWIM_DRAWDIBDRAW || SHOWIM_DRAWDIBDISPLAY
	    hDrawDib = DrawDibOpen();
	#endif

	return(TRUE);
      }

      case WM_COMMAND:
	switch (LOWORD(wParam)) {

	  case IDSNAP:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    #if ONELIB_TWOUNIT || TWOLIB_ONEUNIT
		if (liveon) {
		    for (u = 0; u < UNITS; u++)
			xclibselect[u]->xcdev.setUnLive(&xclibselect[u]->xcdev, 1<<unitselect[u], 0,stateidselect[u],NULL, NULL);
		} else {
		    for (u = 0; u < UNITS; u++) {
			err = xclibselect[u]->xcdev.setSnapBuf(&xclibselect[u]->xcdev, 1<<unitselect[u], 0, stateidselect[u],NULL,NULL,1, 1);
			if (err < 0)
			    MessageBox(NULL, pxd_mesgErrorCode(err), "setSnapBuf", MB_OK|MB_TASKMODAL);
		}
	    #endif
	    #if PXDLIB_ENHANCED
		if (liveon) {
		    pxd_goUnLive(UNITSMAP);
		} else {
		    err = pxd_goSnap(UNITSMAP, 1);
		    if (err < 0)
			MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_goSnap", MB_OK|MB_TASKMODAL);
		}
	    #endif
	    #if PXELIB_ENHANCED
		if (liveon) {
		    for (u = 0; u < UNITS; u++)
			pxe_goUnLive(pxdstates[u], 1);
		} else {
		    for (u = 0; u < UNITS; u++) {
			pxe_goSnap(pxdstates[u], 1, 1);
			if (err < 0)
			    MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_goSnap", MB_OK|MB_TASKMODAL);
		    }
		}
	    #endif
	    liveon = FALSE;
	    return(TRUE);

	  case IDLIVE:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    #if ONELIB_TWOUNIT || TWOLIB_ONEUNIT
		for (u = 0; u < UNITS; u++)
		    xclibselect[u]->xcdev.setLiveBuf(&xclibselect[u]->xcdev, 1<<unitselect[u], 0,stateidselect[u],NULL,NULL, 1, 1);
	    #endif
	    #if PXDLIB_ENHANCED
		pxd_goLive(UNITSMAP, 1L);
	    #endif
	    #if PXELIB_ENHANCED
		for (u = 0; u < UNITS; u++)
		    pxe_goLive(pxdstates[u], 1, 1L);
	    #endif
	    liveon = TRUE;
	    return(TRUE);

	  case IDUNLIVE:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    #if ONELIB_TWOUNIT || TWOLIB_ONEUNIT
		for (u = 0; u < UNITS; u++)
		    xclibselect[u]->xcdev.setUnLive(&xclibselect[u]->xcdev, 1<<unitselect[u], 0,stateidselect[u],NULL, NULL);
	    #endif
	    #if PXDLIB_ENHANCED
		pxd_goUnLive(UNITSMAP);
	    #endif
	    #if PXELIB_ENHANCED
		for (u = 0; u < UNITS; u++)
		    pxe_goUnLive(pxdstates[u], 1);
	    #endif
	    liveon = FALSE;
	    return(TRUE);

	case IDSAVE:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    for (u = 0; u < UNITS; u++) {
		OPENFILENAME ofn;
		char	title[80] = "Save";
		char	pathname[_MAX_PATH] = "";
		int	r;
		memset(&ofn, 0, sizeof(ofn));
		ofn.lStructSize = sizeof(ofn);
		ofn.hwndOwner	= hWnd;
		ofn.lpstrFilter = "TIFF Files (*.tif)\0*.tif\0\0";
		ofn.lpstrFile	= pathname;
		ofn.nMaxFile	= _MAX_PATH;
		if (UNITS > 1) {
		    _snprintf(title, sizeof(title), "Save Unit %d", u);
		    title[sizeof(title)-1] = 0;
		}
		ofn.lpstrTitle	= title;
		ofn.Flags   |= OFN_EXPLORER|OFN_OVERWRITEPROMPT|OFN_HIDEREADONLY;
		r = GetSaveFileName(&ofn);
		if (r != 0) {
		    #if ONELIB_TWOUNIT || TWOLIB_ONEUNIT
			err = xclibselect[u]->pxaux.imageSaveTiff(&xclibselect[u]->pxaux, 0, 1<<unitselect[u],stateidselect[u],1, 0,0,-1,-1,pathname,0);
		    #endif
		    #if PXDLIB_ENHANCED
			err = pxd_saveTiff(1<<u, pathname, 1, 0, 0, -1, -1, 0, 0);
		    #endif
		    #if PXELIB_ENHANCED
			err = pxe_saveTiff(pxdstates[u], 1, pathname, 1, 0, 0, -1, -1, 0, 0);
		    #endif
		    if (err < 0)
			MessageBox(NULL, pxd_mesgErrorCode(err), "imageSaveTiff/pxd_saveTiff/pxe_saveTiff", MB_OK|MB_TASKMODAL);
		}
	    }
	    return(TRUE);
	}
	break;

      case WM_CLOSE:
	isopen = 0;
	#if ONELIB_TWOUNIT
	    xclib_close(&xclib[0]);
	#endif
	#if TWOLIB_ONEUNIT
	    for (u = 0; u < UNITS; u++)
		xclib_close(&xclib[u]);
	#endif
	#if PXDLIB_ENHANCED
	    pxd_PIXCIclose();
	#endif
	#if PXELIB_ENHANCED
	    for (u = 0; u < UNITS; u++)
		pxe_PIXCIclose(pxdstates[u]);
	    pxe_XCLIBuninstantiate(pxdstates[u]);
	    pxdstates[u] = NULL;
	#endif
	#if SHOWIM_DIRECTXDISPLAY
	    if (lpDD)
		lpDD->Release();
	    if (hDDLibrary)
		FreeLibrary(hDDLibrary);
	    lpDD = NULL;
	    hDDLibrary = NULL;
	#endif
	#if SHOWIM_DRAWDIBDRAW || SHOWIM_DRAWDIBDISPLAY
	    if (hDrawDib)
		DrawDibClose(hDrawDib);
	    hDrawDib = NULL;
	#endif

	DestroyWindow(hWnd);
	EndDialog(hDlg, 0);
	return(TRUE);

      case WM_TIMER:
	//
	// Monitor for asynchronous faults, such as video
	// being disconnected while capturing. These faults
	// can't be reported by functions such as pxd_goLive()
	// which initiate capture and return immediately.
	//
	// Should there be a fault pxd_mesgFault() pop up a dialog,
	// the Windows TIMER will continue in a new thread. Thus the
	// 'faulting' variable and logic to limit to one dialog at a time.
	//
	if (isopen) {	// implies whether library is open
	    static int faulting = 0;
	    if (!faulting) {
		faulting++;
		#if ONELIB_TWOUNIT
		    xclib[0].pxaux.faultMessageBox(&xclib[0].pxaux, 0, (1<<UNITS)-1, NULL, NULL, NULL, 0);
		#endif
		#if TWOLIB_ONEUNIT
		    for (u = 0; u < UNITS; u++)
			xclib[u].pxaux.faultMessageBox(&xclib[u].pxaux, 0, 0x1, NULL, NULL, NULL, 0);
		#endif
		#if PXDLIB_ENHANCED
		    pxd_mesgFault(UNITSMAP);
		#endif
		faulting--;
	    }
	}

	//
	// Has a new field or frame been captured
	// since the last time we checked?
	//
	pxbuffer_t  buf = 1;
	for (u = 0; u < UNITS; u++) {
	    #if ONELIB_TWOUNIT || TWOLIB_ONEUNIT
		pxvbtime_t lasttime = xclibselect[u]->xcdev.getLiveStatus(&xclibselect[u]->xcdev, 1<<unitselect[u], 0, PXVIST_DONE | PXVIST_VCNT);
		if (lastcapttime[u] == lasttime)
		    continue;
		lastcapttime[u] = lasttime;
		DisplayBuffer(xclibselect[u], stateidselect[u], unitselect[u], buf, hWndImage, windImage[u]);
	    #endif
	    #if PXDLIB_ENHANCED
		pxvbtime_t lasttime = pxd_capturedFieldCount(1<<u);
		if (lastcapttime[u] == lasttime)
		    continue;
		lastcapttime[u] = lasttime;
		DisplayBuffer(u, buf, hWndImage, windImage);
	    #endif
	    #if PXELIB_ENHANCED
		pxvbtime_t lasttime = pxe_capturedFieldCount(pxdstates[u], 1);
		if (lastcapttime[u] == lasttime)
		    continue;
		lastcapttime[u] = lasttime;
		DisplayBuffer(pxdstates[u], 0, buf, hWndImage, windImage);
	    #endif
	}

	return(TRUE);

    }
    return(FALSE);
}

LRESULT CALLBACK MainWndProc(
    HWND	hWnd,
    unsigned	message,
    WPARAM	wParam,
    LPARAM	lParam
){
    switch (message) {
	case WM_CLOSE:
	    DestroyWindow(hWnd);
	    return(0);
	case WM_DESTROY:
	    PostQuitMessage(0);
	    return(0);
    }
    return(DefWindowProc(hWnd, message, wParam, lParam));
}

/*
 * The Main
 */
int APIENTRY WinMain(
    HINSTANCE  hInstance,
    HINSTANCE  hPrevInstance,
    LPSTR      lpCmdLine,
    int        nCmdShow
){
    MSG       msg;
    WNDCLASS  wc;

    wc.style	     = CS_BYTEALIGNWINDOW;
    wc.lpfnWndProc   = MainWndProc;
    wc.cbClsExtra    = 0;
    wc.cbWndExtra    = 0;
    wc.hInstance     = hInstance;
    wc.hIcon	     = 0;
    wc.hCursor	     = LoadCursor(0, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
    wc.lpszMenuName  =	NULL;
    wc.lpszClassName = "PXlibWClass";

    if (!RegisterClass(&wc))
	return (FALSE);

    if (hPrevInstance)
	return(FALSE);

    hWnd = CreateWindow("PXlibWClass",
			"XCLIBEX5 Windows Example",
			WS_OVERLAPPEDWINDOW,
			CW_USEDEFAULT, CW_USEDEFAULT,
			CW_USEDEFAULT, CW_USEDEFAULT,
			0, 0, hInstance, NULL );
    if (!hWnd)
	return (FALSE);

    //
    // This main window of this example does nothing useful.
    // Don't bother showing it.
    //
    //ShowWindow(hWnd, nCmdShow);
    //UpdateWindow(hWnd);
    hDlg = CreateDialogParam(hInstance, "PIXCIDIALOG", NULL, (DLGPROC)PIXCIDialogProc, NULL);
    if (!hDlg) {
	MessageBox(NULL, "Missing Dialog Resource - Compilation or Link Error!", "XCLIBEX5", MB_OK|MB_TASKMODAL);
	return(FALSE);
    }

    while (GetMessage(&msg, 0, 0, 0)) {
	if (!hDlg || !IsDialogMessage(hDlg, &msg)) {
	    TranslateMessage(&msg);
	    DispatchMessage(&msg);
	}
    }
    return (msg.wParam);
}
