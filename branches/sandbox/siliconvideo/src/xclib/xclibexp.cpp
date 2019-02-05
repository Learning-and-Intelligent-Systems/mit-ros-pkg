/*
 *
 *	xclibexp.cpp	External	27-Dec-2010
 *
 *	Copyright (C)  1998-2010  EPIX, Inc.  All rights reserved.
 *
 *	Example program for the XCLIB 'C' Library.
 *	Example assumes Windows 95/98/ME/NT/2000/XP/Vista/7/XP(x64)/Vista(x64)/7(x64).
 *
 *	Demonstrates XCLIB and/or PXIPL functions for capture,
 *	post processing and display.
 *	This Windows program must, of course, also make use of
 *	various Windows GDI API functions; however, this is not
 *	intended to serve as a Windows tutorial.
 *
 *	Many parts similar to the XCLIBEX2.CPP example; that
 *	example provides PIXCI SV2, SV3, SV4, SV5, SV5A, SV5B, SV5L, SV6, and SV7
 *	specific controls, and demonstrates more display options including
 *	graphic overlays. It also demonstrates access to numeric
 *	pixel values and use of Events rather than a timer.
 *	For simplicity this example concentrates on demonstrating
 *	post capture processing and display,  and does not illustrate
 *	as many display options nor duplicate the other features
 *	demonstrated in that example.
 *
 */


/*
 *  INSTRUCTIONS:
 *
 *  1)	Set 'define' options below according to the intended camera
 *	and video format(s).
 *
 *	For PIXCI(R) SV2, SV3, SV4, SV5, SV5A, SV5B, SV5L, and SV6 imaging boards
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


#if !defined(FORMAT) && !defined(FORMATFILE_LOAD) && !defined(FORMATFILE_COMP)
				  // For PIXCI(R) SV2, SV3, SV4, SV5, SV5A, SV5B, SV5L, SV6
  //#define FORMAT  "RS-170"	  // RS-170 on input 2
  //#define FORMAT  "NTSC"	  // NTSC on input 2
  //#define FORMAT  "NTSC/YC"	  // NSTC S-Video on input 1		  (N/A on SV5A,SV5B)
  //#define FORMAT  "CCIR"	  // CCIR on input 2
  //#define FORMAT  "PAL"	  // PAL (B,D,G,H,I) on input 2
  //#define FORMAT  "PAL/YC"	  // PAL (B,D,G,H,I) S-Video on input 1   (N/A on SV5A,SV5B)
  //#define FORMAT  "default"	  // NSTC S-Video on input 1

				  // For PIXCI(R) SV7
  //#define FORMAT  "RS-170"	  // RS-170
  //#define FORMAT  "NTSC"	  // NTSC
  //#define FORMAT  "CCIR"	  // CCIR
  //#define FORMAT  "PAL"	  // PAL
  //#define FORMAT  "default"	  // NSTC

				  // For PIXCI(R) A, CL1, CL2, CL3SD, D, D24, D32,
				  // D2X, D3X, E1, E1DB, E4, E4DB, EB1, EB1POCL,
				  // EC1, ECB1, ECB1-34, ECB2, EL1, EL1DB,
				  // ELS2, SI, SI1, SI2, SI4
    #define FORMAT  "default"	  // as per board's intended camera

				  // For any PIXCI(R) imaging board
				  // using a format file saved by XCAP
				  // which can either be:
  //#define FORMATFILE_LOAD   "xcvidset.fmt"  // loaded from file during execution
  //#define FORMATFILE_COMP   "xcvidset.fmt"  // or compiled into this application
#endif


/*
 *  2.1) Set number of expected PIXCI(R) image boards, from 1 to 4.
 *  The XCLIB Simple 'C' Functions expect that the boards are
 *  identical and operated at the same resolution.
 *
 *  For PIXCI(R) imaging boards with multiple, functional units,
 *  the XCLIB presents the two halves of the
 *  PIXCI\*(Rg\ E1DB, E4DB, ECB2, EL1DB, ELS2, SI2, or SV7 imaging boards
 *  or the four quarters of the PIXCI\*(Rg\ SI4 imaging board
 *  as two or four independent PIXCI\*(Rg\ imaging boards, respectively.
 *
 */
#if !defined(UNITS)
    #define UNITS	1
#endif
#define UNITSMAP    ((1<<UNITS)-1)  /* shorthand - bitmap of all units */


/*
 *  2.3) Optionally, set driver configuration parameters.
 *  These are normally left to the default, "".
 *  The actual driver configuration parameters include the
 *  desired PIXCI(R) imaging boards, but to make configuation easier,
 *  code, below, will automatically add board selection to this.
 */
#if !defined(DRIVERPARMS)
  //#define DRIVERPARMS "-QU 0"   // don't use interrupts
    #define DRIVERPARMS ""	  // default
#endif


/*
 *  3.1) Choose which form of image display is to be demonstrated.
 *  Some of these options expect that the Windows DirectDraw SDK is present
 *  (available from Microsoft) and that the S/VGA supports DirectDraw.
 *
 *  Only one of these choices should have value 1, the others should be 0.
 */
#if !defined(SHOWIM_DRAWDIBDISPLAY) && !defined(SHOWIM_GDIDISPLAY) && !defined(SHOWIM_DIRECTXDISPLAY)
    #define SHOWIM_DRAWDIBDISPLAY   0	// use XCLIB and PXIPL and Video for Windows
    #define SHOWIM_GDIDISPLAY	    1	// use XCLIB and PXIPL
    #define SHOWIM_DIRECTXDISPLAY   0	// use XCLIB and PXIPL and DirectDraw
#endif

/*
 *  3.2) Select how 'Live' mode should be demonstrated.
 *  Only one of these choices should have value 1, the others should be 0.
 */
#if !defined(LIVE_LIVE) && !defined(LIVE_SNAP) && !defined(LIVE_LIVE2)
    #define LIVE_LIVE	    1	// Continuous capture into frame buffer,
				// display from same frame buffer while
				// next frame is being captured.
    #define LIVE_SNAP	    0	// Snap single frame into frame buffer,
				// display, repeat.
    #define LIVE_LIVE2	    0	// Continuous capture into alternate buffers,
				// display from frame buffer which is not being
				// captured into. Requires sufficient frame buffer
				// memory for at least two frame buffers.
#endif


/*
 *  4)	Compile with Microsoft Visual Studio VS2010 as Win32 Platform
 *	application using command line tools:
 *
 *	    cl -I "\Program Files\EPIX\XCLIB" -c xclibexp.cpp
 *	    rc.exe /l 0x409 /fo tt.res	"\Program Files\EPIX\XCLIB\xclibex2.rc"
 *	    echo -subsystem:windows,4.0 -entry:WinMainCRTStartup comdlg32.lib ole32.lib >t1
 *	    echo /NODEFAULTLIB libc.lib oldnames.lib user32.lib gdi32.lib kernel32.lib vfw32.lib largeint.lib winmm.lib >t2
 *
 *	    Either (XCLIB w/out PXIPL for Windows NT/2000/XP/Vista/7)
 *		link xclibexp.obj "\Program Files\EPIX\XCLIB\XCLIBWNT.LIB" tt.res @t1 @t2
 *	    or	(XCLIB+PXIPL for Windows NT/2000/XP/Vista/7)
 *		link xclibexp.obj "\Program Files\EPIX\XCLIB\XCLIBWNT.LIB" "\Program Files\EPIX\XCLIB\PXIPLWNT.LIB" tt.res @t1 @t2
 *	    or (XCLIB-Lite for Windows NT/2000/XP/Vista/7)
 *		link xclibexp.obj "\Program Files\EPIX\XCLIB\XCLYBWNT.LIB" tt.res @t1 @t2
 *
 *	    del t1
 *	    del t2
 *	    del tt.res
 *	    del xclibexp.obj
 *
 *	Or, compile with Microsoft Visual Studio VS2010 as x64 Platform
 *	application using command line tools:
 *
 *	    cl -I "\Program Files\EPIX\XCLIB" -c xclibexp.cpp
 *	    rc.exe /l 0x409 /fo tt.res "\Program Files\EPIX\XCLIB\xclibex2.rc"
 *	    echo -subsystem:windows -machine:X64 -entry:WinMainCRTStartup comdlg32.lib ole32.lib >t1
 *	    echo /NODEFAULTLIB libcmt.lib oldnames.lib user32.lib gdi32.lib kernel32.lib vfw32.lib winmm.lib >t2
 *
 *	Either (XCLIB w/out PXIPL for Windows XP(x64)/Vista(x64)/7(x64)
 *	    link xclibexp.obj "\Program Files\EPIX\XCLIB\XCLIBW64.LIB" tt.res @t1 @t2
 *	or  (XCLIB+PXIPL for Windows XP(x64)/Vista(x64)/7(x64)
 *	    link xclibexp.obj "\Program Files\EPIX\XCLIB\XCLIBW64.LIB" "\Program Files\EPIX\XCLIB\PXIPLW64.LIB" tt.res @t1 @t2
 *
 *	    del t1
 *	    del t2
 *	    del tt.res
 *	    del xclibexp.obj
 *
 *    Or use the Microsoft Visual Studio VS2010 IDE, compiling xclibexp.cpp
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
 *  'XCLIBEX2.CPP' and XCLIBEX2.RC with 'XCLIBEXP.CPP' and 'XCLIBEX2.RC'.
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
#if SHOWIM_DRAWDIBDISPLAY
    #include <vfw.h>
#endif
#if SHOWIM_DIRECTXDISPLAY
    #include <compobj.h>
    #include <ddraw.h>
#endif

extern "C" {
    #include "xcliball.h"
    #include "xclibex2.h"
    #include "pxipl.h"         
    #include "pximages.h"           
}

/*
 * Global variables.
 */
static	HWND	hWnd;	    /* the main window */
static	HWND	hDlg;	    /* the main dialog */
#if SHOWIM_DRAWDIBDISPLAY
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
	MessageBox(NULL, "No memory!", "XCLIBEXP", MB_OK|MB_TASKMODAL);
	return;
    }
    va_start(argp, format);
    _vsnprintf(buf, BUFSIZE, format, argp);
    buf[BUFSIZE-1] = 0; // this & snprintf: overly conservative - avoids warning messages
    va_end(argp);
    MessageBox(NULL, buf, "XCLIBEXP", MB_OK|MB_TASKMODAL);
    free(buf);
}

/*
 * Process and Display specified buffer from specified unit,
 * in specified AOI of specified HWND,
 * using a compile-time selected method.
 */
void DisplayBuffer(int unit, pxbuffer_t buf, HWND hWndImage, struct pxywindow windImage[])
{

    struct pximage  *procimage = NULL;
    struct pximage  *tempimage = NULL;

    //
    // Perform image processing.
    //
    // In LIVE_SNAP mode, we can process the frame buffer in place,
    // as capture of a new image isn't initiated until after display.
    // In LIVE_LIVE and LIVE_LIVE2 modes, we would run the risk
    // of modifying an image buffer while the same buffer is overwritten
    // with a new image; but we must check that there is a spare frame buffer.
    //
    pxbuffer_t procbuf = 0;
    #if LIVE_SNAP
	procbuf = buf;
    #elif LIVE_LIVE
	if (pxd_imageZdim() > 1)
	    procbuf = pxd_imageZdim();
    #elif LIVE_LIVE2
	if (pxd_imageZdim() > 2)
	    procbuf = pxd_imageZdim();
    #endif

    //
    // If there are insufficient frame buffers,
    // or if the camera outputs raw Bayer (see the
    // Processing Images from a Bayer Output Camera
    // with the PIXCI\*(Rg D or CL Series application note)
    // use a malloc'ed image buffer as temporary.
    //
    // There is no easy way to determine if the color camera
    // is Bayer output. Here we assume that any color camera is.
    // Even for monochrome cameras, using a malloc'ed image buffer
    // is safe. But as to whether it is slower or faster
    // depends on many variables.
    //
    // Admiteddly, some of this code would be simpler
    // if we always used a malloc'ed image buffer as temporary.
    // Which would work, but is not as educational.
    //
    if (procbuf == 0
     || pxd_imageCdim() > 1) {
	tempimage = pximage_memmalloc2(NULL,
				pxd_imageBdim()<=8? PXDATUCHAR: PXDATUSHORT,
				pxd_imageBdim(),
				pxd_imageCdim(),
				pxd_imageCdim()==1? PXHINTGREY: PXHINTRGB,
				pxd_imageXdim(),
				pxd_imageYdim(),
				0.f, 0.f, 0);
	if (tempimage == NULL) {
	    mesg("Can't malloc temporary image buffer");
	    return;
	}
	procimage = tempimage;
    } else {
	procimage = pxd_defineImage(1<<unit, procbuf, 0, 0, -1, -1, pxd_imageCdim()==1? "Grey": "RGB");
    }

    //
    // Process.
    // Copy and complement entire image.
    //
    pxip8_pixneg(NULL,
	    pxd_defineImage(1<<unit, buf, 0, 0, -1, -1, pxd_imageCdim()==1? "Grey": "RGB"),
	    procimage);


    //
    // Display the procimage
    //
    HDC  hDC = GetDC(hWndImage);

    //
    // Show image using PXIPL pxio8_GDIDisplay.
    // Altho not used in this example, pxio8_GDIDisplay also allows drawing
    // a full screen cross hair cursor over the image.
    //
    #if SHOWIM_GDIDISPLAY
	pxio8_GDIDisplay(NULL, procimage,
			 NULL, 0, 'n', 0, 0,hDC,&windImage[unit], NULL, NULL);
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
		pxio8_DirectXDisplay(NULL, procimage,
				     NULL, 0, 'n', 0, 0, ddrs, &windImage[unit], NULL, hDC, NULL, NULL);
		windImage[unit].nw.x -= pt.x;
		windImage[unit].nw.y -= pt.y;
		windImage[unit].se.x -= pt.x;
		windImage[unit].se.y -= pt.y;
		((LPDIRECTDRAWSURFACE)ddrs)->Release();
	    }
	}
    }
    #endif


    //
    // Show image using PXIPL and Video for Windows.
    // Altho not used in this example, pxio8_DrawDibDisplay also allows drawing
    // a full screen cross hair cursor over the image.
    //
    #if SHOWIM_DRAWDIBDISPLAY
	pxio8_DrawDibDisplay(NULL, procimage,
			 NULL, 0, 'n', 0, 0,hDrawDib,hDC,&windImage[unit], NULL, NULL);
    #endif

    ReleaseDC(hWndImage, hDC);

    //
    // If a malloc'ed image buffer was used, free it.
    //
    if (tempimage)
	pximage_memfree2(tempimage, NULL);
}

/*
 * The Dialog
 */
BOOL CALLBACK
PIXCIDialogProc(HWND hDlg, UINT wMsg, WPARAM wParam, LPARAM lParam)
{
    static  HWND	hWndImage;			    // child window of dialog for image display
    static  UINT	svgaBits;			    // pixel format of S/VGA
    static  int 	liveon = 0;
    static  pxvbtime_t	lastcapttime[UNITS] = {0};	    // when was image last captured
    static  struct	pxywindow windImage[max(4,UNITS)];  // subwindow of child window for image display
	    int 	err = 0;

    #if SHOWIM_DIRECTXDISPLAY
	static LPDIRECTDRAW lpDD = NULL;
	static HINSTANCE    hDDLibrary = NULL;
    #endif
    #if SHOWIM_DRAWDIBDISPLAY
	static HDRAWDIB     hDrawDib = NULL;
    #endif


    switch (wMsg) {
    case WM_INITDIALOG:
    {
	RECT	rectImage;

	//
	// Open the PIXCI(R) imaging board.
	// If this program were to only support a single PIXCI(R)
	// imaging board, the first parameter could be simplified to:
	//
	//	if (pxd_PIXCIopen("", FORMAT, NULL) < 0)
	//	    pxd__mesgFault(1);
	//
	// But, for the sake of multiple PIXCI(R) imaging boards
	// specify which units are to be used.
	//
	char driverparms[80];
	driverparms[sizeof(driverparms)-1] = 0; // this & snprintf: overly conservative - avoids warning messages
	_snprintf(driverparms, sizeof(driverparms)-1, "-DM 0x%x %s", UNITSMAP, DRIVERPARMS);
	//
	// Either FORMAT or FORMATFILE_LOAD or FORMATFILE_COMP
	// should have been selected above.
	//
	#if defined(FORMAT)
	    if (pxd_PIXCIopen(driverparms, FORMAT, "") < 0)
		pxd_mesgFault(UNITSMAP);
	#elif defined(FORMATFILE_LOAD)
	    //
	    // The FORMATFILE can be read and loaded
	    // during the pxd_PIXCIopen(), for convenience
	    // of changing the format file without recompiling.
	    //
	    if (pxd_PIXCIopen(driverparms, "", FORMATFILE_LOAD) < 0)
		pxd_mesgFault(UNITSMAP);
	#elif defined(FORMATFILE_COMP)
	    //
	    // Or the FORMATFILE can be compiled into this application,
	    // reducing the number of files that must be distributed, or
	    // possibly lost.
	    //
	    // Note: On MSVC 6.0, if the precompiled header option is used,
	    // the compiler objects to this code (error C2006) when
	    // FORMATFILE_COMP is not defined and thus not intended to be used,
	    // even though this code shouldn't be compiled when
	    // FORMATFILE_COMP is not defined.
	    // Either turn off the 'Use Precompiled Headers' option,
	    // remove this code, or choose to use the FORMATFILE_COMP option.
	    //
	    if (pxd_PIXCIopen(driverparms, "Default", "") < 0)
		pxd_mesgFault(UNITSMAP);
	    {
		#include FORMATFILE_COMP
		pxd_videoFormatAsIncluded(0);
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
	// Determine displayed size.
	// We could simply fill up the hWndImage, but
	// much rather adjust the displayed image for
	// correct aspect ratio.
	//
	windImage[0].nw.x = windImage[0].nw.y = 0;
	windImage[0].se.x = rectImage.right+1;		 // inclusive->exclusive
	windImage[0].se.y = rectImage.bottom+1; 	 // inclusive->exclusive
	{
	    double  scalex, scaley, aspect;
	    aspect = pxd_imageAspectRatio();
	    if (aspect == 0.0)
		aspect = 1.0;
	    scalex = windImage[0].se.x/(double)pxd_imageXdim();
	    scaley = windImage[0].se.y/((double)pxd_imageYdim()*aspect);
	    scalex = min(scalex, scaley);
	    windImage[0].se.x = (int)(pxd_imageXdim() * scalex);
	    windImage[0].se.y = (int)(pxd_imageYdim() * scalex * aspect);
	}

	//
	// If StrecthDIBits is to be used, some VGA card drivers
	// abhor horizontal dimensions which are not a multiple of 4.
	// This isn't needed for other rendering methods, but doesn't hurt.
	//
	windImage[0].se.x &= ~3;

	//
	// For multiple units, display each of four units
	// in quadrant of display area.
	// Or, even for one unit, with the optional graph enabled.
	//
	if (UNITS > 1) {
	    windImage[0].se.x &= ~0xF;	 // See above StretchDIBits comment above
	    windImage[1] = windImage[0];
	    windImage[2] = windImage[0];
	    windImage[3] = windImage[0];
	    windImage[0].se.x /= 2;
	    windImage[0].se.y /= 2;
	    windImage[1].nw.x = windImage[1].se.x/2;
	    windImage[1].se.y /= 2;
	    windImage[2].se.x /= 2;
	    windImage[2].nw.y = windImage[2].se.y/2;
	    windImage[3].nw.x = windImage[3].se.x/2;
	    windImage[3].nw.y = windImage[3].se.y/2;
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
	#if SHOWIM_DRAWDIBDISPLAY
	    hDrawDib = DrawDibOpen();
	#endif

	//
	// Warn if LIVE_LIVE2 was selected and there is only one buffer.
	//
	#if LIVE_LIVE2
	    if (pxd_imageZdim() < 2)
		MessageBox(NULL, "Only 1 image frame buffer available!", "XCLIBEXP", MB_OK|MB_TASKMODAL);
	#endif

	return(TRUE);
    }

    case WM_COMMAND:
	switch (LOWORD(wParam)) {

	case IDSNAP:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    if (liveon) {
		pxd_goUnLive(UNITSMAP);
		liveon = FALSE;
		return(TRUE);
	    }

	    //
	    // As the WM_TIMER code monitors
	    // completed capture buffers, there is no need to wait
	    // for the snap to be done.
	    //
	    err = pxd_goSnap(UNITSMAP, 1);
	    if (err < 0)
		MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_goSnap", MB_OK|MB_TASKMODAL);
	    return(TRUE);

	case IDLIVE:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    liveon = TRUE;
	    #if LIVE_LIVE
		err = pxd_goLive(UNITSMAP, 1L);
		if (err < 0)
		    MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_goLive", MB_OK|MB_TASKMODAL);
	    #elif LIVE_LIVE2
		err = pxd_goLivePair(UNITSMAP, 1L, 2L);
		if (err < 0)
		    MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_goLivePair", MB_OK|MB_TASKMODAL);
	    #elif LIVE_SNAP
		err = pxd_goSnap(UNITSMAP, 1);
		if (err < 0)
		    MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_goSnap", MB_OK|MB_TASKMODAL);
	    #endif
	    return(TRUE);

	case IDUNLIVE:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    #if LIVE_LIVE
		pxd_goUnLive(UNITSMAP);
		liveon = FALSE;
	    #elif LIVE_LIVE2
		pxd_goUnLive(UNITSMAP);
		liveon = FALSE;
	    #elif LIVE_SNAP
		liveon = FALSE;
	    #endif
	    return(TRUE);

	case IDSAVE:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    for (int u = 0; u < UNITS; u++) {
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
		    err = pxd_saveTiff(1<<u, pathname, 1, 0, 0, -1, -1, 0, 0);
		    if (err < 0)
			MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_saveTiff", MB_OK|MB_TASKMODAL);
		}
	    }
	    return(TRUE);
	}
	break;

    case WM_CLOSE:
	pxd_PIXCIclose();
	//DestroyWindow(GetParent(hDlg));
	#if SHOWIM_DIRECTXDISPLAY
	    if (lpDD)
		lpDD->Release();
	    if (hDDLibrary)
		FreeLibrary(hDDLibrary);
	    lpDD = NULL;
	    hDDLibrary = NULL;
	#endif
	#if SHOWIM_DRAWDIBDISPLAY
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
	if (pxd_infoUnits()) {	 // implies whether library is open
	    static int faulting = 0;
	    if (!faulting) {
		faulting++;
		pxd_mesgFault(UNITSMAP);
		faulting--;
	    }
	}

	//
	// Has a new field or frame been captured
	// since the last time we checked?
	//
	for (int u = 0; u < UNITS; u++) {
	    pxvbtime_t lasttime = pxd_capturedFieldCount(1<<u);
	    if (lastcapttime[u] == lasttime)
		continue;
	    lastcapttime[u] = lasttime;

	    //
	    // In LIVE_LIVE2 mode the captured buffer
	    // rotates between 1 and 2.
	    // In other modes, the code above always
	    // does snap or live into buffer 1.
	    //
	    pxbuffer_t	buf = 1;
	    #if LIVE_LIVE2
		buf = pxd_capturedBuffer(1<<u);
	    #endif

	    DisplayBuffer(u, buf, hWndImage, windImage);
	    //
	    // After display, if in live mode using LIVE_SNAP,
	    // having displayed one image start snapping the next
	    //
	    #if LIVE_SNAP
		if (liveon) {
		    err = pxd_goSnap(1<<u, 1);
		    if (err < 0)
			MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_goSnap", MB_OK|MB_TASKMODAL);
		}
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
			"XCLIBEXP Windows Example",
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
	MessageBox(NULL, "Missing Dialog Resource - Compilation or Link Error!", "XCLIBEXP", MB_OK|MB_TASKMODAL);
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
