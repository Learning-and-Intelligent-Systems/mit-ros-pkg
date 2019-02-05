/*
 *
 *	xclibexd.cpp	External	27-Dec-2010
 *
 *	Copyright (C)  2001-2010  EPIX, Inc.  All rights reserved.
 *
 *	Example program for the XCLIB 'C' Library with
 *	   SILICON VIDEO(R) 1281M, 1281C
 *	   SILICON VIDEO(R) 1310,  1310C
 *	   SILICON VIDEO(R) 2112,  2112C
 *	   SILICON VIDEO(R) 5C10,  5M10
 *	   SILICON VIDEO(R) 642M,  642C
 *	   SILICON VIDEO(R) 643M,  643C
 *	   SILICON VIDEO(R) 9C10
 *	   SILICON VIDEO(R) 9M001, 9M001C
 *	   SILICON VIDEO(R) 9T001C
 *	   SILICON VIDEO(R) WGA-M, WGA-C
 *	cameras. Example assumes Windows 95/98/ME/NT/2000/XP/Vista/7/XP(x64)/Vista(x64)/7(x64).
 *
 *	Demonstrates XCLIB and/or PXIPL functions for capture,
 *	display of imagery, and adjustment of camera.
 *	This Windows program must, of course, also make use of
 *	various Windows GDI API functions; however, this is not
 *	intended to serve as a Windows tutorial.
 *
 *	Many parts similar to the XCLIBEX2.CPP example; that
 *	example will also operate the PIXCI(R) and the SILICON VIDEO(R)
 *	cameras, demonstrates more display options including
 *	graphic overlays, demonstrates simultaneous use of
 *	multiple PIXCI(R) imaging boards and cameras,
 *	and demonstrates access to numeric pixel values and
 *	saving of images; but does not have camera specific controls.
 *	This example demonstrates camera specific controls, but
 *	for simplicity does not illustrate as many display options.
 *	Either example can be read and used to demonstrate
 *	the desired features.
 *
 */

/*
 *  INSTRUCTIONS:
 *
 *  1.1) Set 'define' options below according to the intended camera
 *	and video format.
 *
 *	For PIXCI(R) D2X imaging boards with SV2112, SV2112C, SV1310,
 *	SV1310C, SV1281M, SV1281C. For PIXCI(R) SI, SI1, SI2, or SI4
 *	imaging boards with SV5C10, SV5M10, SV642M, SV642C, SV643M,
 *	SV643C, SV9C10, SV9M001, SV9M001C, SV9T001C, SV-WGA-C, SV-WGA-M
 *	use "default" to select the default format for the camera for which
 *	the PIXCI(R) imaging board is intended. For non default formats,
 *	use XCAP to save the video set-up to a file, and set FORMATFILE
 *	to the saved file's path name.
 *
 *	Alternately, this could be modified to use getenv("PIXCI"),
 *	GetPrivateProfileString(...), RegQueryValueEx(...), or any
 *	other convention chosen by the programmer to allow run time
 *	selection of the video format and resolution.
 *
 */

#if !defined(FORMAT) && !defined(FORMATFILE)
					// For PIXCI(R) D2X w.
					//	SV2112, SV2112C
					//	SV1310, SV1310C
					//	SV1281M, SV1281C
					// For PIXCI(R) SI, SI1, SI2, or SI4 w.
					//	SV9M001, SV9M001C
					//	SV9T001C
					//	SV642M, SV642C
					//	SV5C10, SV5M10
					//	SV9C10
					//	SV643M, SV643C
					//	SV-WGA-M, SV-WGA-C
    #define FORMAT  "default"		// as per board's intended camera
  //#define FORMATFILE	"xcvidset.fmt"	// using format file saved by XCAP
#endif


/*
 *  2.1) Set number of expected PIXCI(R) image boards and cameras,
 *  from 1 to 4. The XCLIB Simple 'C' Functions expect that the
 *  boards are identical and operated at the same resolution.
 *
 *  For PIXCI(R) imaging boards with multiple, functional units,
 *  the XCLIB presents the two halves of the
 *  PIXCI\*(Rg\ SI2 imaging board or the four quarters of the
 *  PIXCI\*(Rg\ SI4 imaging board as two or four, respectively,
 *  independent PIXCI\*(Rg\ SI imaging boards.
 */
#if !defined(UNITS)
    #define UNITS	1
#endif
#define UNITSMAP    ((1<<UNITS)-1)  /* shorthand - bitmap of all units */
#if !defined(UNITSOPENMAP)
    #define UNITSOPENMAP UNITSMAP
#endif



/*
 *  2.2) Optionally, set driver configuration parameters.
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
 *  3.1) Choose wich form of image display (SHOWIM_), or image and
 *  graphics overlay (SHOWIG_), is to be demonstrated. Some of these
 *  options expect that the optional PXIPL library is present.
 *  Others may expect that the Windows DirectDraw SDK is present
 *  (available from Microsoft) and that the S/VGA supports DirectDraw.
 *
 *  Only one of these choices should have value 1, the others should be 0.
 *
 *  See the XCLIBEX2.CPP example for additional choices and options.
 */
#if !defined(SHOWIM_STRETCHDIBITS) && !defined(SHOWIG_STRETCHDIBITS2) \
 && !defined(SHOWIG_STRETCHDIBITS3) \
 && !defined(SHOWIM_GDIDISPLAY)    && !defined(SHOWIM_DIRECTXDISPLAY)
    #define SHOWIM_STRETCHDIBITS    1	// use XCLIB or XCLIB-Lite and GDI
    #define SHOWIG_STRETCHDIBITS2   0	// use XCLIB or XCLIB-Lite and GDI
    #define SHOWIG_STRETCHDIBITS3   0	// use XCLIB, PXIPL and GDI
    #define SHOWIM_GDIDISPLAY	    0	// use XCLIB and PXIPL
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
 *  3.3) Select how display updates should be triggered.
 *  Only one of these choices should have value 1, the others should be 0.
 */
#if !defined(UPDATE_TIMER) && !defined(UPDATE_EVENT)
    #define UPDATE_TIMER    1	// Use timer to periodically poll library
				// and test for a new image to be displayed.
				// Simpler for those not familiar with threads
				// and events.
    #define UPDATE_EVENT    0	// Use an event to trigger display of new image.
				// Slightly more efficient, but requires use of
				// threads and events.
#endif


/*
 *  3.4) Select whether software AGC/AEC should be enabled.
 */
#if !defined(SOFT_AGC)
    #define SOFT_AGC	    0  // 1: yes, 0: no
#endif


/*
 *  4)	Compile with Microsoft Visual Studio VS2010 as Win32 Platform
 *	application using command line tools:
 *
 *	    cl -I "\Program Files\EPIX\XCLIB" -c xclibexd.cpp
 *	    rc.exe /l 0x409 /fo tt.res	"\Program Files\EPIX\XCLIB\xclibexd.rc"
 *	    echo -subsystem:windows,4.0 -entry:WinMainCRTStartup comdlg32.lib ole32.lib >t1
 *	    echo /NODEFAULTLIB libc.lib oldnames.lib user32.lib gdi32.lib kernel32.lib vfw32.lib largeint.lib winmm.lib >t2
 *
 *	    Either (XCLIB w/out PXIPL for Windows NT/2000/XP/Vista/7)
 *		link xclibexd.obj "\Program Files\EPIX\XCLIB\XCLIBWNT.LIB" tt.res @t1 @t2
 *	    or	(XCLIB+PXIPL for Windows NT/2000/XP/Vista/7)
 *		link xclibexd.obj "\Program Files\EPIX\XCLIB\XCLIBWNT.LIB" "\Program Files\EPIX\XCLIB\PXIPLWNT.LIB" tt.res @t1 @t2
 *	    or (XCLIB-Lite for Windows NT/2000/XP/Vista/7)
 *		link xclibexd.obj "\Program Files\EPIX\XCLIB\XCLYBWNT.LIB" tt.res @t1 @t2
 *
 *	    del t1
 *	    del t2
 *	    del tt.res
 *	    del xclibexd.obj
 *
 *	Or, compile with Microsoft Visual Studio VS2010 as x64 Platform
 *	application using command line tools:
 *
 *	    cl -I "\Program Files\EPIX\XCLIB" -c xclibexd.cpp
 *	    rc.exe /l 0x409 /fo tt.res "\Program Files\EPIX\XCLIB\xclibexd.rc"
 *	    echo -subsystem:windows -machine:X64 -entry:WinMainCRTStartup comdlg32.lib ole32.lib >t1
 *	    echo /NODEFAULTLIB libcmt.lib oldnames.lib user32.lib gdi32.lib kernel32.lib vfw32.lib winmm.lib >t2
 *
 *	Either (XCLIB w/out PXIPL for Windows XP(x64)/Vista(x64)/7(x64)
 *	    link xclibexd.obj "\Program Files\EPIX\XCLIB\XCLIBW64.LIB" tt.res @t1 @t2
 *	or  (XCLIB+PXIPL for Windows XP(x64)/Vista(x64)/7(x64)
 *	    link xclibexd.obj "\Program Files\EPIX\XCLIB\XCLIBW64.LIB" "\Program Files\EPIX\XCLIB\PXIPLW64.LIB" tt.res @t1 @t2
 *
 *	    del t1
 *	    del t2
 *	    del tt.res
 *	    del xclibexd.obj
 *
 *    Or use the Microsoft Visual Studio VS2010 IDE, compiling xclibexd.cpp
 *    as a Windows NT/2000/XP/Vista/7 Win32 Platform or x64 Platform app,
 *    using the XCLIBEXD.RC resource, linking to library(s):
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
 *	   7. Copy XCLIBEXD.RC to your project directory.
 *	   8. Select 'Project' -> 'Add Existing Item' and select XCLIBEXD.RC.
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
 *  'XCLIBEX2.CPP' and XCLIBEX2.RC with 'XCLIBEXD.CPP' and 'XCLIBEXD.RC'.
 *
 */


/*
 *  NECESSARY INCLUDES:
 */
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
#if SHOWIM_DIRECTXDISPLAY
    #include <compobj.h>
    #include <ddraw.h>
#endif

extern "C" {
    #include "xcliball.h"
    #include "xclibex2.h"
    #if SHOWIM_GDIDISPLAY || SHOWIM_DIRECTXDISPLAY || SHOWIG_STRETCHDIBITS3
	#include "pxipl.h"         
	#include "pximages.h"           
    #endif
}

/*
 * Global variables.
 */
static	HWND	    hWnd;				// the main window
static	HWND	    hDlg;				// the main dialog
static	int	    liveon = 0; 			// whether live video was requested
static	HWND	    hWndImage;				// child window of dialog for image display
static	pxvbtime_t  lastcapttime[UNITS] = {0};		// when was image last captured
static	struct	    pxywindow windImage[max(4,UNITS)];	// subwindow of child window for image display
static	ulong	    displaycount = 0;			// display rate statistics
static	CRITICAL_SECTION critsect;			// for use with Events
static	UINT	    svgaBits;				// pixel format of S/VGA

#define NSUBSAMPLES 100
static	int	    subsamples[NSUBSAMPLES];		// map LISTBOX index to valid subsample values, such as 0x0101

#define NSCANDIRECTIONS 4
static	int	    scandirections[NSCANDIRECTIONS];	// map LISTBOX index to valid scandirection values, such as CC('R','B')



/*
 * To improve readability when translating camera parameters,
 * such as gain, pixel clock, etc., to/from scrollbar controls.
 * Especially useful as this universal example operates with any
 * SILICON VIDEO camera, so we can't easily hard code the mapping
 * based on the range of a specific camera's specific parameter.
 */
#define ToScroll(Value, LoV, HiV)   (int)(((Value)-(LoV))*32767./((HiV)-(LoV)))
#define UnScroll(Scroll, LoV, HiV)  ((LoV)+(double)(Scroll)*((HiV)-(LoV))/32767.)


/*
 * Shorthand.
 */
#if !defined(CC)
  #define CC(a,b)   (((a)<<8) | (b))
#endif

/*
 * Subroutine to display image.
 * Could be in-line, once, if this were hard-coded
 * for either UPDATE_TIMER vs UPDATE_EVENT.
 * Since we have those options, a subroutine avoids code
 * duplication.
 */
void displayImage()
{
    int err = 0;
    //
    // Has a new field or frame been captured
    // since the last time we checked?
    //
    for (int u = 0; u < UNITS; u++) {
	HDC  hDC;
	pxvbtime_t lasttime = pxd_capturedFieldCount(1<<u);
	if (lastcapttime[u] == lasttime)
	    continue;
	lastcapttime[u] = lasttime;

	hDC = GetDC(hWndImage);

	//
	// In LIVE_LIVE2 mode the captured buffer
	// rotates between 1 and 2.
	// In other modes, the code above always
	// does snap or live into buffer 1.
	//
	pxbuffer_t  buf = 1;
	#if LIVE_LIVE2
	    buf = pxd_capturedBuffer(1<<u);
	#endif

	//
	// Show image using pxd_renderStretchDIBits.
	//
	#if SHOWIM_STRETCHDIBITS
	    SetStretchBltMode(hDC, STRETCH_DELETESCANS);
	    err = pxd_renderStretchDIBits(1<<u, buf, 0, 0, -1, -1, 0,
			     hDC, windImage[u].nw.x, windImage[u].nw.y,
			     windImage[u].se.x-windImage[u].nw.x,
			     windImage[u].se.y-windImage[u].nw.y, 0);
	    if (err < 0)
		MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_renderStretchDIBits", MB_OK|MB_TASKMODAL);
	#endif

	//
	// Show image using pxd_renderDIBCreate and graphics using GDI.
	// Uses an off-screen buffer so the displayed
	// graphics do not flicker. Note that the graphics
	// are drawn in (a copy of) the image before resizing,
	// thus the graphics are resized.
	// It also end up performing one extra copying of the image,
	// from the DIB to the DDB.
	//
	#if SHOWIG_STRETCHDIBITS2
	{
	    HGLOBAL hDIB = pxd_renderDIBCreate(1<<u, buf, 0, 0, -1, -1, 0, 0);
	    if (hDIB) {
		BITMAPINFOHEADER FAR *dib = (BITMAPINFOHEADER FAR *)GlobalLock(hDIB);
		if (dib) {
		    HBITMAP hDIB2 = CreateDIBitmap(hDC, dib, CBM_INIT,
				(uchar FAR*)dib+dib->biSize+dib->biClrUsed*sizeof(RGBQUAD),
				(BITMAPINFO*)dib, DIB_RGB_COLORS);
		    if (hDIB2) {
			HDC hdcDIB = CreateCompatibleDC(hDC);
			if (hdcDIB) {
			    HGDIOBJ  ohDIB = SelectObject(hdcDIB, hDIB2);
			    TextOut(hdcDIB, 20, 20, "This is text", 12);
			    SetStretchBltMode(hDC, STRETCH_DELETESCANS);
			    StretchBlt(hDC, windImage[u].nw.x, windImage[u].nw.y,
					    windImage[u].se.x-windImage[u].nw.x,
					    windImage[u].se.y-windImage[u].nw.y,
				    hdcDIB, 0, 0, pxd_imageXdim(), pxd_imageYdim(), SRCCOPY);
			    SelectObject(hdcDIB, ohDIB);
			    DeleteDC(hdcDIB);
			}
			DeleteObject(hDIB2);
		    }
		    GlobalUnlock(hDIB);
		}
		pxd_renderDIBFree(hDIB);
	    } else
		MessageBox(NULL, "Error", "pxd_renderDIBCreate", MB_OK|MB_TASKMODAL);
	}
	#endif

	//
	// Show image using pxio8_GDIDisplay and graphics using GDI.
	// Uses an off-screen buffer so the displayed
	// graphics do not flicker. Note that the graphics
	// are drawn in (a copy of) the image after resizing,
	// thus the graphics are not resized, yielding crisper graphics.
	//
	#if SHOWIG_STRETCHDIBITS3
	{
	    HBITMAP hDIB = NULL;
	    HGDIOBJ hODIB;
	    HDC     hDCDIB;
	    RECT    rect;
	    struct  pxywindow	windDIB;

	    //
	    // Create HBITMAP as off-screen buffer.
	    // And create DC for it.
	    //
	    hDIB = CreateCompatibleBitmap(hDC, windImage[u].se.x-windImage[u].nw.x,
					       windImage[u].se.y-windImage[u].nw.y);
	    hDCDIB = CreateCompatibleDC(hDC);
	    hODIB = SelectObject(hDCDIB, hDIB);

	    //
	    // Draw image into it, resizing.
	    // The "Display" selection is equivalent to using
	    //	   pxd_imageCdim()==1?"GREY":"BGR"
	    //
	    windDIB.nw.x = 0;
	    windDIB.nw.y = 0;
	    windDIB.se.x = windImage[u].se.x-windImage[u].nw.x;
	    windDIB.se.y = windImage[u].se.y-windImage[u].nw.y;
	    err = pxio8_GDIDisplay(NULL, pxd_defineImage(1<<u, buf, 0, 0, -1, -1, "Display"),
			    NULL, 0, 'n', 0, 0, hDCDIB, &windDIB, // not &windImage[u]!
			    NULL, NULL);
	    if (err < 0)
		MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_GDIDisplay", MB_OK|MB_TASKMODAL);

	    //
	    // Use GDI to draw graphics over image.
	    //
	    SetRect(&rect, windImage[u].nw.x+(windImage[u].se.x-windImage[u].nw.x)/4,
			   windImage[u].nw.y+(windImage[u].se.y-windImage[u].nw.y)/4,
			   windImage[u].nw.x+(windImage[u].se.x-windImage[u].nw.x)*3/4,
			   windImage[u].nw.y+(windImage[u].se.y-windImage[u].nw.y)*3/4);
	    DrawFocusRect(hDCDIB, &rect);
	    DrawText(hDCDIB, "This is text", 12, &rect, 0);

	    //
	    // Display image & graphics.
	    //
	    BitBlt(hDC, windImage[u].nw.x, windImage[u].nw.y,
			windImage[u].se.x-windImage[u].nw.x,
			windImage[u].se.y-windImage[u].nw.y,
		   hDCDIB, 0, 0, SRCCOPY);

	    SelectObject(hDCDIB, hODIB);
	    DeleteDC(hDCDIB);
	    DeleteObject(hDIB);
	}
	#endif




	//
	// Show image using PXIPL pxio8_GDIDisplay.
	// Altho not used in this example, pxio8_GDIDisplay also allows drawing
	// a full screen cross hair cursor over the image.
	//
	#if SHOWIM_GDIDISPLAY
	    err = pxio8_GDIDisplay(NULL, pxd_defineImage(1<<u, buf, 0, 0, -1, -1, "Display"),
			     NULL, 0, 'n', 0, 0,hDC,&windImage[u], NULL, NULL);
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
	    DDSURFACEDESC   surfacedesc;
	    LPDIRECTDRAWSURFACE ddrs = NULL;
	    HRESULT	h;
	    POINT	pt;
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
		    windImage[u].nw.x += pt.x;
		    windImage[u].nw.y += pt.y;
		    windImage[u].se.x += pt.x;
		    windImage[u].se.y += pt.y;
		    err = pxio8_DirectXDisplay(NULL, pxd_defineImage(1<<u, buf, 0, 0, -1, -1, "Display"),
					 NULL, 0, 'n', 0, 0, ddrs, &windImage[u], NULL, hDC, NULL, NULL);
		    windImage[u].nw.x -= pt.x;
		    windImage[u].nw.y -= pt.y;
		    windImage[u].se.x -= pt.x;
		    windImage[u].se.y -= pt.y;
		    ((LPDIRECTDRAWSURFACE)ddrs)->Release();
		}
	    }
	    if (err < 0)
		MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_DirectXDisplay", MB_OK|MB_TASKMODAL);
	}
	#endif

	ReleaseDC(hWndImage, hDC);

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

	//
	// For display rate statistics.
	//
	if (u == 0)
	    displaycount++;
    }
}


/*
 * Thread to watch for field events, and advise
 * when the image should be redrawn.
 * To simplify this example's synchronization with the main thread,
 * a message is posted to the main thread instead of directly
 * drawing the image,
 */
#if UPDATE_EVENT
DWORD WINAPI ServiceThread(PVOID hEventR3)
{
    for (;;) {
	//
	// Wait for signal.
	//
	WaitForSingleObject((HANDLE)hEventR3, INFINITE);
	//
	// Update image display
	//
	EnterCriticalSection(&critsect);
	displayImage();
	LeaveCriticalSection(&critsect);
    }
    return(0);
}
#endif


/*
 * Init or re-init dialog controls
 *
 * The allowable range of exposure is a function
 * of other camera parameters, such as decimation/subsample, pixel clock
 * and frame rate. The range may also change after switching from free-run
 * to Trigger/Controlled mode.
 * Thus, when decimation/subsample, pixel clock, or frmae rate is
 * changed the exposure slider will, and should, change position!
 *
 * Similarly, allowable frame rates are dependent on
 * decimation/subsample and pixel clock. Frame rate and exposure
 * control may only be available in free-run mode.
 */
void initControls(HWND hDlg, int firsttime)
{
    if (firsttime) {
	//
	double	minclk = pxd_SILICONVIDEO_getMinMaxPixelClock(UNITSMAP,0.0);
	double	maxclk = pxd_SILICONVIDEO_getMinMaxPixelClock(UNITSMAP,9E99);
	SetScrollRange(GetDlgItem(hDlg, IDCLKSCROLL),  SB_CTL, ToScroll(minclk,minclk,maxclk), ToScroll(maxclk,minclk,maxclk), TRUE);
	SetScrollPos  (GetDlgItem(hDlg, IDCLKSCROLL),  SB_CTL, ToScroll(pxd_SILICONVIDEO_getPixelClock(UNITSMAP),minclk,maxclk), TRUE);
	EnableWindow  (GetDlgItem(hDlg, IDCLKSCROLL),  pxd_SILICONVIDEO_getMinMaxPixelClock(UNITSMAP,0.0) != pxd_SILICONVIDEO_getMinMaxPixelClock(UNITSMAP,9E99));
	//
	// For some cameras, in color mode, the gains help implement white balancing,
	// and should be ganged together - adjusted in unison
	// and relative differences maintained. Not needed for mono cameras,
	// but neither is it detrimental.
	//
	double gainsA[4];
	double	mingainA = pxd_SILICONVIDEO_getMinMaxGainA(UNITSMAP,-9E99);
	double	maxgainA = pxd_SILICONVIDEO_getMinMaxGainA(UNITSMAP,9E99);
	pxd_SILICONVIDEO_getGainsA(UNITSMAP, gainsA);
	SetScrollRange(GetDlgItem(hDlg, IDGAINSCROLL), SB_CTL, ToScroll(mingainA,mingainA,maxgainA), ToScroll(maxgainA,mingainA,maxgainA), TRUE);
	SetScrollPos  (GetDlgItem(hDlg, IDGAINSCROLL), SB_CTL, ToScroll(min(gainsA[0],min(gainsA[1],min(gainsA[2],gainsA[3]))),mingainA,maxgainA), TRUE);
	//
	double offsetsA[4];
	double	minoffsetA = pxd_SILICONVIDEO_getMinMaxOffsetA(UNITSMAP,-9E99);
	double	maxoffsetA = pxd_SILICONVIDEO_getMinMaxOffsetA(UNITSMAP,9E99);
	pxd_SILICONVIDEO_getOffsetsA(UNITSMAP, offsetsA);
	SetScrollRange(GetDlgItem(hDlg, IDOFFSETSCROLL), SB_CTL, ToScroll(minoffsetA,minoffsetA,maxoffsetA), ToScroll(maxoffsetA,minoffsetA,maxoffsetA), TRUE);
	SetScrollPos  (GetDlgItem(hDlg, IDOFFSETSCROLL), SB_CTL, ToScroll(min(offsetsA[0],min(offsetsA[1],min(offsetsA[2],offsetsA[3]))),minoffsetA,maxoffsetA), TRUE);
	EnableWindow  (GetDlgItem(hDlg, IDOFFSETSCROLL), minoffsetA != maxoffsetA);
	//
	double gainsB[4];
	double	mingainB = pxd_SILICONVIDEO_getMinMaxGainB(UNITSMAP,-9E99);
	double	maxgainB = pxd_SILICONVIDEO_getMinMaxGainB(UNITSMAP,9E99);
	pxd_SILICONVIDEO_getGainsB(UNITSMAP, gainsB);
	SetScrollRange(GetDlgItem(hDlg, IDGAIN2SCROLL), SB_CTL, ToScroll(mingainB,mingainB,maxgainB), ToScroll(maxgainB,mingainB,maxgainB), TRUE);
	SetScrollPos  (GetDlgItem(hDlg, IDGAIN2SCROLL), SB_CTL, ToScroll(min(gainsB[0],min(gainsB[1],min(gainsB[2],gainsB[3]))),mingainB,maxgainB), TRUE);
	EnableWindow  (GetDlgItem(hDlg, IDGAIN2SCROLL), mingainB != maxgainB);
	//
	// Determine valid subsample/decimage options. Build list
	// and mapping from the GUI control's index into the subsample code
	// such as 0x0101 or 0x0202. This could be simplified in
	// camera specific code to a hardcoded list.
	//
	{
	    int originalSubsample = pxd_SILICONVIDEO_getSubsample(UNITSMAP);
	    int guiIndex = 0;
	    subsamples[guiIndex++] = 0x0101;	 // do special case first
	    SendMessage (GetDlgItem(hDlg, IDLISTDECIMATE),  LB_ADDSTRING, 0, (LPARAM)"Off");
	    for (int x = 0; x < 4; x++) {
		for (int y = 0; y < 4; y++) {
		    int xsub = 1<<x;
		    int ysub = 1<<y;
		    int subsample = (xsub<<8) | ysub;
		    if (subsample == 0x0101)
			continue;
		    pxd_SILICONVIDEO_setResolutionAndTiming(UNITSMAP, 0, subsample,
					pxd_SILICONVIDEO_getAoiLeft(UNITSMAP), pxd_SILICONVIDEO_getAoiTop(UNITSMAP),
					pxd_SILICONVIDEO_getAoiWidth(UNITSMAP), pxd_SILICONVIDEO_getAoiHeight(UNITSMAP),
					pxd_SILICONVIDEO_getScanDirection(UNITSMAP),
					pxd_imageBdim(), 0, 0,
					pxd_SILICONVIDEO_getPixelClock(UNITSMAP),
					pxd_SILICONVIDEO_getFramePeriod(UNITSMAP), 0, 0, 0);
		    if (pxd_SILICONVIDEO_getSubsample(UNITSMAP) == subsample
		     && guiIndex < NSUBSAMPLES) {
			subsamples[guiIndex++] = subsample;
			char buf[100];
			buf[sizeof(buf)-1] = 0; // this & snprintf: overly conservative - avoids warning messages
			_snprintf(buf, sizeof(buf)-1, "Sample %dx%d", xsub, ysub);
			SendMessage (GetDlgItem(hDlg, IDLISTDECIMATE),	LB_ADDSTRING, 0, (LPARAM)buf);
		    }
		}
	    }
	    for (int i = 0; ; i++) {
		if (i >= guiIndex) {
		    // was the initial subsample setting not covered by iterations?
		    originalSubsample = 0x0101;
		    SendMessage(GetDlgItem(hDlg, IDLISTDECIMATE), LB_SETCURSEL, 0, 0);
		    break;
		}
		if (originalSubsample == subsamples[i]) {
		    SendMessage(GetDlgItem(hDlg, IDLISTDECIMATE), LB_SETCURSEL, i, 0);
		    break;
		}
	    }
	    pxd_SILICONVIDEO_setResolutionAndTiming(UNITSMAP, 0, originalSubsample,
				pxd_SILICONVIDEO_getAoiLeft(UNITSMAP), pxd_SILICONVIDEO_getAoiTop(UNITSMAP),
				pxd_SILICONVIDEO_getAoiWidth(UNITSMAP), pxd_SILICONVIDEO_getAoiHeight(UNITSMAP),
				pxd_SILICONVIDEO_getScanDirection(UNITSMAP),
				pxd_imageBdim(), 0, 0,
				pxd_SILICONVIDEO_getPixelClock(UNITSMAP),
				pxd_SILICONVIDEO_getFramePeriod(UNITSMAP), 0, 0, 0);
	    EnableWindow(GetDlgItem(hDlg, IDLISTDECIMATE), guiIndex > 1);
	    //
	    // Check for hardware supports for AGC & AEC.
	    //
	    if (pxd_SILICONVIDEO_getMinMaxAgcA(UNITSMAP, 99999) == 0)
		ShowWindow(GetDlgItem(hDlg, IDAGCCHECK), SW_HIDE);
	    else
		ShowWindow(GetDlgItem(hDlg, IDAGCCHECK), SW_SHOW);
	    if (pxd_SILICONVIDEO_getMinMaxAec(UNITSMAP, 99999) == 0)
		ShowWindow(GetDlgItem(hDlg, IDAECCHECK), SW_HIDE);
	    else
		ShowWindow(GetDlgItem(hDlg, IDAECCHECK), SW_SHOW);
	}
	//
	// Determine valid scan direction options. Build list
	// and mapping from the GUI control's index into the scan direction code
	// such as CC('L','T'). This could be simplified in
	// camera specific code to a hardcoded list.
	//
	if (NSCANDIRECTIONS >= 4) {	// just being safe
	    int originalScan = pxd_SILICONVIDEO_getScanDirection(UNITSMAP);
	    int guiIndex = 0;
	    int scancodes[]	 = { CC('L','T'), CC('L','B'), CC('R','T'), CC('R','B') };
	    char * scanstrings[] = { "L-R/T-B",   "L-R/B-T",   "R-L/T-B",   "R-L/B-T"	};
	    for (int s = 0; s < 4; s++) {
		pxd_SILICONVIDEO_setResolutionAndTiming(UNITSMAP, 0,
				    pxd_SILICONVIDEO_getSubsample(UNITSMAP),
				    pxd_SILICONVIDEO_getAoiLeft(UNITSMAP), pxd_SILICONVIDEO_getAoiTop(UNITSMAP),
				    pxd_SILICONVIDEO_getAoiWidth(UNITSMAP), pxd_SILICONVIDEO_getAoiHeight(UNITSMAP),
				    scancodes[s],
				    pxd_imageBdim(), 0, 0,
				    pxd_SILICONVIDEO_getPixelClock(UNITSMAP),
				    pxd_SILICONVIDEO_getFramePeriod(UNITSMAP), 0, 0, 0);
		if (pxd_SILICONVIDEO_getScanDirection(UNITSMAP) == scancodes[s]) {
		    SendMessage (GetDlgItem(hDlg, IDLISTSCAN),	LB_ADDSTRING, 0, (LPARAM)scanstrings[s]);
		    scandirections[guiIndex] = scancodes[s];
		    if (scancodes[s] == originalScan)
			SendMessage(GetDlgItem(hDlg, IDLISTSCAN), LB_SETCURSEL, guiIndex, 0);
		    guiIndex++;
		}
	    }
	    pxd_SILICONVIDEO_setResolutionAndTiming(UNITSMAP, 0,
				pxd_SILICONVIDEO_getSubsample(UNITSMAP),
				pxd_SILICONVIDEO_getAoiLeft(UNITSMAP), pxd_SILICONVIDEO_getAoiTop(UNITSMAP),
				pxd_SILICONVIDEO_getAoiWidth(UNITSMAP), pxd_SILICONVIDEO_getAoiHeight(UNITSMAP),
				originalScan,
				pxd_imageBdim(), 0, 0,
				pxd_SILICONVIDEO_getPixelClock(UNITSMAP),
				pxd_SILICONVIDEO_getFramePeriod(UNITSMAP), 0, 0, 0);
	    EnableWindow(GetDlgItem(hDlg, IDLISTSCAN), guiIndex > 1);
	}

	//
	// Video Modes.
	// Continuous live & unlive buttons aren't applicable
	// in single shot camera modes
	//
	SendMessage   (GetDlgItem(hDlg, IDVIDEOMODE), LB_ADDSTRING, 0, (LPARAM)"Free-Run");
	SendMessage   (GetDlgItem(hDlg, IDVIDEOMODE), LB_ADDSTRING, 0, (LPARAM)"Ext. Trigger");
	SendMessage   (GetDlgItem(hDlg, IDVIDEOMODE), LB_ADDSTRING, 0, (LPARAM)"'Snap' Trigger");

	//
	// White balance is only for color cameras.
	//
	EnableWindow(GetDlgItem(hDlg, IDWHITEBAL), pxd_imageCdim()==3);

	//
	// The AOI can be adjusted in much finer steps, as well as moved in position.
	// To keep the GUI simple, only a few options are implemented in this example.
	//
	int maxwidth = pxd_SILICONVIDEO_getMinMaxAoiWidth(UNITSMAP, INT_MAX);
	int maxheight = pxd_SILICONVIDEO_getMinMaxAoiHeight(UNITSMAP, INT_MAX);
	char buf[100];
	for (int i = 1; i <= 16; i *= 2) {
	    if (i!=1)
		if ((maxwidth/i)*i != maxwidth || (maxheight/i)*i != maxheight)
		    break;
	    buf[sizeof(buf)-1] = 0; // this & snprintf: overly conservative - avoids warning messages
	    _snprintf(buf, sizeof(buf)-1, "%dx%d", maxwidth/i, maxheight/i);
	    SendMessage   (GetDlgItem(hDlg, IDLISTAOI),  LB_ADDSTRING, 0, (LPARAM)buf);
	}
	int xdim = pxd_imageXdim();
	if (xdim == maxwidth)
	    SendMessage(GetDlgItem(hDlg, IDLISTAOI), LB_SETCURSEL, 0, 0);
	else if (xdim >= maxwidth/2)
	    SendMessage(GetDlgItem(hDlg, IDLISTAOI), LB_SETCURSEL, 1, 0);
	else if (xdim >= maxwidth/4)
	    SendMessage(GetDlgItem(hDlg, IDLISTAOI), LB_SETCURSEL, 2, 0);
	else
	    SendMessage(GetDlgItem(hDlg, IDLISTAOI), LB_SETCURSEL, 3, 0);
	EnableWindow(GetDlgItem(hDlg, IDLISTAOI), TRUE);
    }
    //
    double minfp = pxd_SILICONVIDEO_getMinMaxFramePeriod(UNITSMAP,0.0);
    double maxfp = pxd_SILICONVIDEO_getMinMaxFramePeriod(UNITSMAP,9E99);
    SetScrollRange(GetDlgItem(hDlg, IDFPSSCROLL), SB_CTL, ToScroll(1000.0/maxfp,1000.0/maxfp,1000.0/minfp), ToScroll(1000.0/minfp,1000.0/maxfp,1000.0/minfp), TRUE);
    SetScrollPos  (GetDlgItem(hDlg, IDFPSSCROLL), SB_CTL, ToScroll(1000.0/pxd_SILICONVIDEO_getFramePeriod(UNITSMAP),1000.0/maxfp,1000.0/minfp), TRUE);
    if ((pxd_SILICONVIDEO_getVideoMode(UNITSMAP)&0xFF) == 'c'
     && (pxd_SILICONVIDEO_getCtrlVideoMode(UNITSMAP)) == 's') {
	EnableWindow(GetDlgItem(hDlg, IDLIVE),	 FALSE);
	EnableWindow(GetDlgItem(hDlg, IDUNLIVE), FALSE);
	EnableWindow(GetDlgItem(hDlg, IDFPSSCROLL), FALSE && minfp != maxfp);
	if (pxd_SILICONVIDEO_getCtrlTriggerMode(UNITSMAP) == 'b')
	   SendMessage(GetDlgItem(hDlg, IDVIDEOMODE), LB_SETCURSEL, 2, 0);
	else
	   SendMessage(GetDlgItem(hDlg, IDVIDEOMODE), LB_SETCURSEL, 1, 0);
    } else {
	EnableWindow(GetDlgItem(hDlg, IDLIVE),	 TRUE);
	EnableWindow(GetDlgItem(hDlg, IDUNLIVE), TRUE);
	EnableWindow(GetDlgItem(hDlg, IDFPSSCROLL), TRUE && minfp != maxfp);
	SendMessage(GetDlgItem(hDlg, IDVIDEOMODE), LB_SETCURSEL, 0, 0);
    }
    //
    double minexp = pxd_SILICONVIDEO_getMinMaxExposure(UNITSMAP,0.0);
    double maxexp = pxd_SILICONVIDEO_getMinMaxExposure(UNITSMAP,9E99);
    SetScrollRange(GetDlgItem(hDlg, IDEXPSCROLL), SB_CTL, ToScroll(minexp,minexp,maxexp), ToScroll(maxexp,minexp,maxexp), TRUE);
    SetScrollPos  (GetDlgItem(hDlg, IDEXPSCROLL), SB_CTL, ToScroll(pxd_SILICONVIDEO_getExposure(UNITSMAP),minexp,maxexp), TRUE);
    EnableWindow (GetDlgItem(hDlg,  IDEXPSCROLL), minexp != maxexp && pxd_SILICONVIDEO_getAec(UNITSMAP) == 0);
    //
    double  mingainA = pxd_SILICONVIDEO_getMinMaxGainA(UNITSMAP,-9E99);
    double  maxgainA = pxd_SILICONVIDEO_getMinMaxGainA(UNITSMAP,9E99);
    EnableWindow  (GetDlgItem(hDlg, IDGAINSCROLL), mingainA != maxgainA && pxd_SILICONVIDEO_getAgcA(UNITSMAP) == 0);
    //
    SendMessage(GetDlgItem(hDlg, IDAGCCHECK), BM_SETCHECK, pxd_SILICONVIDEO_getAgcA(UNITSMAP)==0? BST_UNCHECKED: BST_CHECKED, 0);
    SendMessage(GetDlgItem(hDlg, IDAECCHECK), BM_SETCHECK, pxd_SILICONVIDEO_getAec(UNITSMAP)==0? BST_UNCHECKED: BST_CHECKED, 0);
}


/*
 * Process gain scroll
 */
BOOL doGainScroll(WPARAM wParam)
{
    int     err = 0;
    double  gains[4];
    double  mingain = pxd_SILICONVIDEO_getMinMaxGainA(UNITSMAP,-9E99);
    double  maxgain = pxd_SILICONVIDEO_getMinMaxGainA(UNITSMAP,9E99);
    pxd_SILICONVIDEO_getGainsA(UNITSMAP, gains);
    double gang = min(gains[0], min(gains[1], min(gains[2], gains[3])));
    double g = gang;
    switch (LOWORD(wParam)) {
	case SB_PAGEDOWN:	g += (maxgain-mingain)/10;   break;
	case SB_LINEDOWN:	g += (maxgain-mingain)/50;   break;
	case SB_PAGEUP: 	g -= (maxgain-mingain)/10;   break;
	case SB_LINEUP: 	g -= (maxgain-mingain)/50;   break;
	case SB_TOP:		g = mingain;		     break;
	case SB_BOTTOM: 	g = maxgain;		     break;
	case SB_THUMBPOSITION:
	case SB_THUMBTRACK:	g = UnScroll(HIWORD(wParam),mingain,maxgain); break;
	default:		return(FALSE);
    }
    for (int i = 0; i < 4; i++)
	gains[i] += g-gang;
    gang = min(gains[0], min(gains[1], min(gains[2], gains[3])));
    if (gang < mingain)
	for (int i = 0; i < 4; i++)
	    gains[i] += mingain-gang;
    gang = max(gains[0], max(gains[1], max(gains[2], gains[3])));
    if (gang > maxgain)
	for (int i = 0; i < 4; i++)
	    gains[i] -= gang-maxgain;
    err = pxd_SILICONVIDEO_setExposureColorGainOffsets(UNITSMAP, 0, pxd_SILICONVIDEO_getExposure(UNITSMAP), gains, NULL, NULL, NULL);
    pxd_SILICONVIDEO_getGainsA(UNITSMAP, gains);
    SetScrollPos  (GetDlgItem(hDlg, IDGAINSCROLL), SB_CTL, ToScroll(min(gains[0],min(gains[1],min(gains[2],gains[3]))),mingain,maxgain), TRUE);
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setExposureColorGainOffsets", MB_OK|MB_TASKMODAL);
    return(TRUE);
}

BOOL doGainBScroll(WPARAM wParam)
{
    int     err = 0;
    double  gains[4];
    double  mingain = pxd_SILICONVIDEO_getMinMaxGainB(UNITSMAP,-9E99);
    double  maxgain = pxd_SILICONVIDEO_getMinMaxGainB(UNITSMAP,9E99);
    pxd_SILICONVIDEO_getGainsB(UNITSMAP, gains);
    double gang = min(gains[0], min(gains[1], min(gains[2], gains[3])));
    double g = gang;
    switch (LOWORD(wParam)) {
	case SB_PAGEDOWN:	g += (maxgain-mingain)/10;   break;
	case SB_LINEDOWN:	g += (maxgain-mingain)/50;   break;
	case SB_PAGEUP: 	g -= (maxgain-mingain)/10;   break;
	case SB_LINEUP: 	g -= (maxgain-mingain)/50;   break;
	case SB_TOP:		g = mingain;		     break;
	case SB_BOTTOM: 	g = maxgain;		     break;
	case SB_THUMBPOSITION:
	case SB_THUMBTRACK:	g = UnScroll(HIWORD(wParam),mingain,maxgain); break;
	default:		return(FALSE);
    }
    for (int i = 0; i < 4; i++)
	gains[i] += g-gang;
    gang = min(gains[0], min(gains[1], min(gains[2], gains[3])));
    if (gang < mingain)
	for (int i = 0; i < 4; i++)
	    gains[i] += mingain-gang;
    gang = max(gains[0], max(gains[1], max(gains[2], gains[3])));
    if (gang > maxgain)
	for (int i = 0; i < 4; i++)
	    gains[i] -= gang-maxgain;
    err = pxd_SILICONVIDEO_setExposureColorGainOffsets(UNITSMAP, 0, pxd_SILICONVIDEO_getExposure(UNITSMAP), NULL, gains, NULL, NULL);
    pxd_SILICONVIDEO_getGainsB(UNITSMAP, gains);
    SetScrollPos  (GetDlgItem(hDlg, IDGAIN2SCROLL), SB_CTL, ToScroll(min(gains[0],min(gains[1],min(gains[2],gains[3]))),mingain,maxgain), TRUE);
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setExposureColorGainOffsets", MB_OK|MB_TASKMODAL);
    return(TRUE);
}


/*
 * Process offset (black level) scroll
 */
BOOL doOffsetScroll(WPARAM wParam)
{
    int     err = 0;
    double  offsets[4];
    double  minoffset = pxd_SILICONVIDEO_getMinMaxOffsetA(UNITSMAP,-9E99);
    double  maxoffset = pxd_SILICONVIDEO_getMinMaxOffsetA(UNITSMAP,9E99);
    pxd_SILICONVIDEO_getOffsetsA(UNITSMAP, offsets);
    double gang = min(offsets[0], min(offsets[1], min(offsets[2], offsets[3])));
    double g = gang;
    switch (LOWORD(wParam)) {
	case SB_PAGEDOWN:	g += (maxoffset-minoffset)/10;	 break;
	case SB_LINEDOWN:	g += (maxoffset-minoffset)/50;	 break;
	case SB_PAGEUP: 	g -= (maxoffset-minoffset)/10;	 break;
	case SB_LINEUP: 	g -= (maxoffset-minoffset)/50;	 break;
	case SB_TOP:		g = minoffset;	break;
	case SB_BOTTOM: 	g = maxoffset;	break;
	case SB_THUMBPOSITION:
	case SB_THUMBTRACK:	g = UnScroll(HIWORD(wParam),minoffset,maxoffset);    break;
	default:		return(FALSE);
    }
    for (int i = 0; i < 4; i++)
	offsets[i] += g-gang;
    gang = min(offsets[0], min(offsets[1], min(offsets[2], offsets[3])));
    if (gang < minoffset)
	for (int i = 0; i < 4; i++)
	    offsets[i] += minoffset-gang;
    gang = max(offsets[0], max(offsets[1], max(offsets[2], offsets[3])));
    if (gang > maxoffset)
	for (int i = 0; i < 4; i++)
	    offsets[i] -= gang-maxoffset;
    err = pxd_SILICONVIDEO_setExposureColorGainOffsets(UNITSMAP, 0, pxd_SILICONVIDEO_getExposure(UNITSMAP), NULL, NULL, offsets, NULL);
    pxd_SILICONVIDEO_getOffsetsA(UNITSMAP, offsets);
    SetScrollPos  (GetDlgItem(hDlg, IDOFFSETSCROLL), SB_CTL, ToScroll(min(offsets[0],min(offsets[1],min(offsets[2],offsets[3]))),minoffset,maxoffset), TRUE);
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setExposureColorGainOffsets", MB_OK|MB_TASKMODAL);
    return(TRUE);
}

/*
 * Process clock scroll
 */
BOOL doClockScroll(WPARAM wParam)
{
    int     err = 0;
    double  minclk = pxd_SILICONVIDEO_getMinMaxPixelClock(UNITSMAP,0.0);
    double  maxclk = pxd_SILICONVIDEO_getMinMaxPixelClock(UNITSMAP,9E99);
    double  clk    = pxd_SILICONVIDEO_getPixelClock(UNITSMAP);

    switch (LOWORD(wParam)) {
	case SB_PAGEDOWN:	clk += 2.0;   break;
	case SB_LINEDOWN:	clk += .5;    break;
	case SB_PAGEUP: 	clk -= 2.0;   break;
	case SB_LINEUP: 	clk -= .5;    break;
	case SB_TOP:		clk = minclk; break;
	case SB_BOTTOM: 	clk = maxclk; break;
	case SB_THUMBPOSITION:
	case SB_THUMBTRACK:	clk = UnScroll(HIWORD(wParam),minclk,maxclk); break;
	default:		return(FALSE);
    }
    err = pxd_SILICONVIDEO_setResolutionAndTiming(UNITSMAP, 0,
			pxd_SILICONVIDEO_getSubsample(UNITSMAP),
			pxd_SILICONVIDEO_getAoiLeft(UNITSMAP), pxd_SILICONVIDEO_getAoiTop(UNITSMAP),
			pxd_SILICONVIDEO_getAoiWidth(UNITSMAP), pxd_SILICONVIDEO_getAoiHeight(UNITSMAP),
			pxd_SILICONVIDEO_getScanDirection(UNITSMAP),
			pxd_imageBdim(), 0, 0,
			clk,
			pxd_SILICONVIDEO_getFramePeriod(UNITSMAP), 0, 0, 0);
    SetScrollPos(GetDlgItem(hDlg, IDCLKSCROLL), SB_CTL, ToScroll(pxd_SILICONVIDEO_getPixelClock(UNITSMAP),minclk,maxclk), TRUE);
    initControls(hDlg, 0);	// update allowable frame rate & exposure
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setResolutionAndTiming", MB_OK|MB_TASKMODAL);
    return(TRUE);
}

/*
 * Process exposure scroll
 */
BOOL doExposureScroll(WPARAM wParam)
{
    int     err = 0;
    double  exp    = pxd_SILICONVIDEO_getExposure(UNITSMAP);
    double  minexp = pxd_SILICONVIDEO_getMinMaxExposure(UNITSMAP, 0.0);
    double  maxexp = pxd_SILICONVIDEO_getMinMaxExposure(UNITSMAP, 9E99);
    switch (LOWORD(wParam)) {
	case SB_PAGEDOWN:	exp += min(8.3, (maxexp-minexp)/10);	break;
	case SB_LINEDOWN:	exp += min(1.0, (maxexp-minexp)/50);	break;
	case SB_PAGEUP: 	exp -= min(8.3, (maxexp-minexp)/10);	break;
	case SB_LINEUP: 	exp -= min(1.0, (maxexp-minexp)/50);	break;
	case SB_TOP:		exp = minexp;  break;
	case SB_BOTTOM: 	exp = maxexp;  break;
	case SB_THUMBPOSITION:
	case SB_THUMBTRACK:	exp = UnScroll(HIWORD(wParam),minexp,maxexp); break;
	default:		return(FALSE);
    }
    err = pxd_SILICONVIDEO_setExposureColorGainOffsets(UNITSMAP, 0, exp, NULL, NULL, NULL, NULL);
    SetScrollPos(GetDlgItem(hDlg, IDEXPSCROLL), SB_CTL, ToScroll(pxd_SILICONVIDEO_getExposure(UNITSMAP),minexp,maxexp), TRUE);
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setExposureColorGainOffsets", MB_OK|MB_TASKMODAL);
    return(TRUE);
}

/*
 * Process frame rate scroll
 */
BOOL doFpsScroll(WPARAM wParam)
{
    int     err = 0;
    double  fps = 1000.0/pxd_SILICONVIDEO_getFramePeriod(UNITSMAP);
    double  lofps = 1000.0/pxd_SILICONVIDEO_getMinMaxFramePeriod(UNITSMAP,9E99);
    double  hifps = 1000.0/pxd_SILICONVIDEO_getMinMaxFramePeriod(UNITSMAP,0.0);
    switch (LOWORD(wParam)) {
	case SB_PAGEDOWN:	fps += (hifps-lofps)/10.; break;
	case SB_LINEDOWN:	fps += (hifps-lofps)/50.; break;
	case SB_PAGEUP: 	fps -= (hifps-lofps)/10.; break;
	case SB_LINEUP: 	fps -= (hifps-lofps)/50.; break;
	case SB_TOP:		fps = hifps;  break;
	case SB_BOTTOM: 	fps = lofps;  break;
	case SB_THUMBPOSITION:
	case SB_THUMBTRACK:	fps = UnScroll(HIWORD(wParam),lofps,hifps); break;
	default:		return(FALSE);
    }
    err = pxd_SILICONVIDEO_setResolutionAndTiming(UNITSMAP, 0,
			pxd_SILICONVIDEO_getSubsample(UNITSMAP),
			pxd_SILICONVIDEO_getAoiLeft(UNITSMAP), pxd_SILICONVIDEO_getAoiTop(UNITSMAP),
			pxd_SILICONVIDEO_getAoiWidth(UNITSMAP), pxd_SILICONVIDEO_getAoiHeight(UNITSMAP),
			pxd_SILICONVIDEO_getScanDirection(UNITSMAP),
			pxd_imageBdim(), 0, 0,
			pxd_SILICONVIDEO_getPixelClock(UNITSMAP),
			1000.0/fps, 0, 0, 0);
    SetScrollPos(GetDlgItem(hDlg, IDFPSSCROLL), SB_CTL, ToScroll(1000.0/pxd_SILICONVIDEO_getFramePeriod(UNITSMAP),lofps,hifps), TRUE);
    initControls(hDlg, 0);	// update allowable exposure
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setResolutionAndTiming", MB_OK|MB_TASKMODAL);
    return(TRUE);
}


/*
 * Process scan direction list.
 */
BOOL doScanList()
{
    int err = 0;
    int scancursel = SendMessage(GetDlgItem(hDlg, IDLISTSCAN), LB_GETCURSEL, 0, 0);
    int scancode = 0;
    if (scancursel >= 0 && scancursel < NSCANDIRECTIONS)
	scancode = scandirections[scancursel];	    // lookup our scan code
    if (scancode != 0) {			    // just being safe
	err = pxd_SILICONVIDEO_setResolutionAndTiming(UNITSMAP, 0,
			    pxd_SILICONVIDEO_getSubsample(UNITSMAP),
			    pxd_SILICONVIDEO_getAoiLeft(UNITSMAP), pxd_SILICONVIDEO_getAoiTop(UNITSMAP),
			    pxd_SILICONVIDEO_getAoiWidth(UNITSMAP), pxd_SILICONVIDEO_getAoiHeight(UNITSMAP),
			    scancode,
			    pxd_imageBdim(), 0, 0,
			    pxd_SILICONVIDEO_getPixelClock(UNITSMAP),
			    pxd_SILICONVIDEO_getFramePeriod(UNITSMAP), 0, 0, 0);
	initControls(hDlg, 0);	    // update ?
    }
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setResolutionAndTiming", MB_OK|MB_TASKMODAL);
    return(TRUE);
}


/*
 * Process decimation/subsample & aoi list.
 * To minimize GUI options and complexity,
 * this example simply positions the AOI in the center.
 */
BOOL doDecimateList()
{
    int err = 0;
    int aoicursel = SendMessage(GetDlgItem(hDlg, IDLISTAOI), LB_GETCURSEL, 0, 0);
    int aoifactor = 1<<aoicursel;
    int maxwidth  = pxd_SILICONVIDEO_getMinMaxAoiWidth(UNITSMAP, INT_MAX);
    int maxheight = pxd_SILICONVIDEO_getMinMaxAoiHeight(UNITSMAP, INT_MAX);
    //
    int subcursel = SendMessage(GetDlgItem(hDlg, IDLISTDECIMATE), LB_GETCURSEL, 0, 0);
    int subsample = 0;
    if (subcursel >= 0 && subcursel < NSUBSAMPLES)
	subsample = subsamples[subcursel];	    // lookup our subsample code
    if (subsample != 0) {			    // just being safe
	err = pxd_SILICONVIDEO_setResolutionAndTiming(UNITSMAP, 0,
			    subsample,
			    (maxwidth-(maxwidth/aoifactor))/2,	    // center!
			    (maxheight-(maxheight/aoifactor))/2,    // center!
			    maxwidth/aoifactor,
			    maxheight/aoifactor,
			    pxd_SILICONVIDEO_getScanDirection(UNITSMAP),
			    pxd_imageBdim(), 0, 0,
			    pxd_SILICONVIDEO_getPixelClock(UNITSMAP),
			    pxd_SILICONVIDEO_getFramePeriod(UNITSMAP), 0, 0, 0);
	initControls(hDlg, 0);		    // update allowable frame rate & exposure
    }
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setResolutionAndTiming", MB_OK|MB_TASKMODAL);
    return(TRUE);
}

/*
 * Process video mode.
 * In this simple example, only the most popular video modes
 * are demonstrated.
 */
BOOL doVideoMode()
{
    int err = 0;
    int videocursel = SendMessage(GetDlgItem(hDlg, IDVIDEOMODE), LB_GETCURSEL, 0, 0);
    switch (videocursel) {
      case 0:	// free run
	err = pxd_SILICONVIDEO_setVideoAndTriggerMode(UNITSMAP, 0, 'f', 0, 0, 0, 0, 0, 0);
	break;
      case 1:	// external trigger
	err = pxd_SILICONVIDEO_setVideoAndTriggerMode(UNITSMAP, 0, 'c', 's', '+', 0, 0, 0, 0);
	break;
      case 2:	// 'snap' trigger
	err = pxd_SILICONVIDEO_setVideoAndTriggerMode(UNITSMAP, 0, 'c', 's', 'b', 0, 0, 0, 0);
	break;
    }
    initControls(hDlg, 0);
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setVideoAndTriggerMode", MB_OK|MB_TASKMODAL);
    return(TRUE);
}

/*
 * Process hardware AGC option
 */
BOOL doAGC(WPARAM wParam)
{
    int err = 0;
    int agc = SendMessage(GetDlgItem(hDlg, IDAGCCHECK), BM_GETCHECK, 0, 0) == BST_UNCHECKED;
    err = pxd_SILICONVIDEO_setAxC(UNITSMAP, 0, agc==0? 0: 9999, -1,-1,-1,-1,-1,-1,-1,-1);
    initControls(hDlg, 0);
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setAxC", MB_OK|MB_TASKMODAL);
    return(TRUE);
}

/*
 * Process hardware AEC option
 */
BOOL doAEC(WPARAM wParam)
{
    int err = 0;
    int aec = SendMessage(GetDlgItem(hDlg, IDAECCHECK), BM_GETCHECK, 0, 0) == BST_UNCHECKED;
    err = pxd_SILICONVIDEO_setAxC(UNITSMAP, 0, -1,-1,-1,-1, aec==0? 0: 9999, -1,-1,-1,-1);
    initControls(hDlg, 0);
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setAxC", MB_OK|MB_TASKMODAL);
    return(TRUE);
}

/*
 * Process white balance button.
 *
 * Some cameras can implement coarse white balance
 * via color specific gain settings.  This function only
 * adjusts the postcapture white balance corrections.
 * Typically, a video setup file exported from XCAP
 * is used to set the coarse settings via gain controls.
 */
BOOL doWhiteButton()
{
    unsigned int reference[3] = {0,0,0};
    unsigned int target[3]    = {0,0,0};
    double  masses[] = {0, 0, 0};
    ushort  pixels[9*9];
    int     midx, midy, i, err;
    //
    // This follows the white balance application note so explanatory
    // comments are minimized.
    // Assumes an image of a white target has already been captured.
    //
    pxd_setImageBrightBalance(UNITSMAP, reference, target, 0.00);
    midx = pxd_imageXdim()/2;
    midy = pxd_imageYdim()/2;
    pxd_readushort(1,1,midx-9/2,midy-9/2,midx+1+9/2,midy+1+9/2,pixels,9*9,"RofRGB");
    for (i = 0; i < 9*9; i++)
	masses[0] += pixels[i];
    pxd_readushort(1,1,midx-9/2,midy-9/2,midx+1+9/2,midy+1+9/2,pixels,9*9,"GofRGB");
    for (i = 0; i < 9*9; i++)
	masses[1] += pixels[i];
    pxd_readushort(1,1,midx-9/2,midy-9/2,midx+1+9/2,midy+1+9/2,pixels,9*9,"BofRGB");
    for (i = 0; i < 9*9; i++)
	masses[2] += pixels[i];
    reference[0] = (unsigned int)(masses[0]/(9*9));
    reference[1] = (unsigned int)(masses[1]/(9*9));
    reference[2] = (unsigned int)(masses[2]/(9*9));
    target[0] = target[1] = target[2] = max(max(reference[0], reference[1]), reference[2]);
    err = pxd_setImageBrightBalance(UNITSMAP, reference, target, 1.00);
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_setImageBrightBalance", MB_OK|MB_TASKMODAL);
    return(TRUE);
}


/*
 * Software AGC/AEC Example.
 * The current image
 * intensity is compared to a desired target, and the
 * camera's gain and/or exposure are adjusted.
 * There are many possible variations, such as whether
 * gain or exposure are adjusted, or both, how fast they
 * are adjusted, etc. This demonstrates the basics and
 * outlines the possible variations.
 */
void doSoftwareAxC()
{
    static  DWORD   lasttickcount = 0;
    static  ulong   lastcapturedfieldcount = 0;
    DWORD   tickcount;
    ulong   capturedfieldcount;
    int     i;
    int     err = 0;

    //
    // How often should be attempt to adjust?
    // Don't waste time adjusting too often,
    // or if nothing new has been captured.
    // We choose every .25 seconds and two fields.
    //
    tickcount = GetTickCount();
    capturedfieldcount =  pxd_capturedFieldCount(0x1);
    if ((tickcount-lasttickcount) < 250
     || (capturedfieldcount-lastcapturedfieldcount) < 2)
	return;
    lasttickcount = tickcount;
    lastcapturedfieldcount = capturedfieldcount;

    //
    // Obtain some metric of image brightness.
    // This could be mean or max of the whole image,
    // or of an AOI, done with a PXIPL function such as pxip8_masscenter(),
    // or pxip8_histab, or done with pxd_readushort or pxd_readuchar.
    // We choose to get the max of a thin sample through the vertical
    // and horizontal w/out PXIPL.
    //
    // This should be improved to read the entire column or row
    // using a single pxd_readuchar; but then we must deal with
    // malloc'ing a suitable buffer and errors therefrom, making
    // the example less readable.
    //
    double  metric = 0;
    uchar   pixel;
    int xdim = pxd_imageXdim();
    int ydim = pxd_imageYdim();
    for (i = 0; i < ydim; i++) {
	err = pxd_readuchar(0x1,1,xdim/2,i,1+xdim/2,i+1,&pixel,1,"Grey");
	// mean?
	//metric += ((double)pixel/(ydim+xdim)) * (100/255.0);	 // scaled to 100% max
	// max?
	metric = max(metric, pixel * (100/255.0));
    }
    for (i = 0; i < xdim; i++) {
	err = pxd_readuchar(0x1,1,i,ydim/2,i+1,1+ydim/2,&pixel,1,"Grey");
	// mean?
	//metric += ((double)pixel/(ydim+xdim)) * (100/255.0);	 // scaled to 100% max
	// max?
	metric = max(metric, pixel * (100/255.0));
    }
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_readuchar", MB_OK|MB_TASKMODAL);

    //
    // Compare metric to desired target.
    // We choose 75% of max.
    //
    double delta = metric - 75.0;

    //
    // If we are close, don't adjust
    // so as to avoid gain/exposure fluctuations for
    // very small changes. We choose 10% hysteresis.
    //
    if (fabs(delta) < 10.0)
	return;

    //
    // Choose whether to implement AGC by modifying
    // gain or exposure. In general, given a choice,
    // it is better to adjust exposure, as increasing gain
    // increases noise. In some applications, it may be beneficial
    // to do both. Here we choose to adjust exposure.
    //
    // Get current exposure value.
    //
    double  exps = pxd_SILICONVIDEO_getExposure(1);

    //
    // Compute new exposure value.
    //
    exps *= (delta < 0? 1.10: 0.90);
    if (fabs(delta) >= 20.0)
	exps *= (delta < 0? 1.10: 0.90);

    //
    // Set. Don't worry if the exposure value is out of bounds.
    // It will be corrected.
    //
    err = pxd_SILICONVIDEO_setExposureColorGainOffsets(UNITSMAP, 0, exps, NULL, NULL, NULL, NULL);
    if (err < 0)
	MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_SILICONVIDEO_setExposureColorGainOffsets", MB_OK|MB_TASKMODAL);
    //
    // Modify GUI control to show the effect.
    //
    double minexp = pxd_SILICONVIDEO_getMinMaxExposure(UNITSMAP, 0.0);
    double maxexp = pxd_SILICONVIDEO_getMinMaxExposure(UNITSMAP, 9E99);
    SetScrollPos(GetDlgItem(hDlg, IDEXPSCROLL), SB_CTL, ToScroll(pxd_SILICONVIDEO_getExposure(UNITSMAP),minexp,maxexp), TRUE);
}


/*
 * The Dialog Guts
 */
BOOL CALLBACK
PIXCIDialogProc1(HWND hDlg, UINT wMsg, WPARAM wParam, LPARAM lParam)
{
    int     err;

    #if SHOWIM_DIRECTXDISPLAY
	static LPDIRECTDRAW lpDD = NULL;
	static HINSTANCE    hDDLibrary = NULL;
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
	_snprintf(driverparms, sizeof(driverparms)-1, "-DM 0x%x %s", UNITSOPENMAP, DRIVERPARMS);
	//
	// Either FORMAT or FORMATFILE should have been
	// selected above.
	//
	#if defined(FORMAT)
	    if (pxd_PIXCIopen(driverparms, FORMAT, "") < 0)
		pxd_mesgFault(UNITSMAP);
	#elif defined(FORMATFILE)
	    if (pxd_PIXCIopen(driverparms, "", FORMATFILE) < 0)
		pxd_mesgFault(UNITSMAP);
	#endif


	//
	// Set our title.
	//
	SetWindowText(hDlg, "EPIX(R) SILICON VIDEO(R) Example");

	//
	// Enable timer, for live video updates.
	// Or, create event and thread for live video updates.
	// Use of the timer is simpler; use of an event and thread
	// is only slightly more efficient.
	//
	#if UPDATE_TIMER
	    SetTimer(hDlg, 1, 5, NULL);
	#endif
	#if UPDATE_EVENT
	    HANDLE  Event;
	    DWORD   ThreadId;
	    Event = pxd_eventCapturedFieldCreate(0x1);
	    if (!Event)
		MessageBox(NULL, "Can't create image update display event!", "XCLIBEXC", MB_OK|MB_TASKMODAL);
	    else
		CreateThread(0, 0x1000, ServiceThread, Event, 0, &ThreadId);
	    //
	    // Timer pops every .25 seconds to check for faults,
	    // not display video.
	    //
	    SetTimer(hDlg, 1, 250, NULL);
	#endif

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
	// Warn if LIVE_LIVE2 was selected and there is only one buffer.
	//
	#if LIVE_LIVE2
	    if (pxd_imageZdim() < 2)
		MessageBox(NULL, "Only 1 image frame buffer available!", "XCLIBEXC", MB_OK|MB_TASKMODAL);
	#endif

	//
	// Customize and init dialog controls
	//
	initControls(hDlg, 1);

	return(TRUE);
      }

      case WM_COMMAND:
	switch (LOWORD(wParam)) {

	  case IDAGCCHECK:
	    return(doAGC(wParam));
	  case IDAECCHECK:
	    return(doAEC(wParam));

	  case IDLISTAOI:
	  case IDLISTDECIMATE:
	    if (HIWORD(wParam) != LBN_SELCHANGE)
		return(FALSE);
	    if (doDecimateList()) {
		if (liveon)	// changing decimation may imply turning video off
		    goto live;
		return(TRUE);
	    }
	    return(FALSE);

	  case IDLISTSCAN:
	    if (HIWORD(wParam) != LBN_SELCHANGE)
		return(FALSE);
	    if (doScanList()) {
		if (liveon)	// changing scan direction may imply turning video off
		    goto live;
		return(TRUE);
	    }
	    return(FALSE);

	  case IDWHITEBAL:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    return(doWhiteButton());

	  case IDSNAP:
	    if (HIWORD(wParam) != BN_CLICKED)
	       return(FALSE);
	    if (liveon) {
		pxd_goUnLive(UNITSMAP);
		liveon = FALSE;
		return(TRUE);
	    }

	    //
	    // As the UPDATE_TIMER or UPDATE_EVENT code monitors
	    // completed capture buffers, there is no need for this
	    // example to wait for the snap to be done.
	    //
	  //err = pxd_doSnap(UNITSMAP, 1, 0);
	    err = pxd_goSnap(UNITSMAP, 1);
	    if (err < 0)
		MessageBox(NULL, pxd_mesgErrorCode(err), "XCLIBEXC", MB_OK|MB_TASKMODAL);
	    return(TRUE);

	  case IDVIDEOMODE:
	    if (HIWORD(wParam) != LBN_SELCHANGE)
		return(FALSE);
	    // Go unlive. In case we were in trigger mode
	    // and snap'ed awaiting a trigger use AbortLive to cancel.
	    pxd_goAbortLive(UNITSMAP);
	    liveon = FALSE;
	    return(doVideoMode());

	  case IDLIVE:
	    if (HIWORD(wParam) != BN_CLICKED)
	       return(FALSE);
	  live:
	    liveon = TRUE;
	    #if LIVE_LIVE
		err = pxd_goLive(UNITSMAP, 1L);
	    #elif LIVE_LIVE2
		err = pxd_goLivePair(UNITSMAP, 1L, 2L);
	    #elif LIVE_SNAP
		err = pxd_goSnap(UNITSMAP, 1);
	    #endif
	    if (err < 0)
		MessageBox(NULL, pxd_mesgErrorCode(err), "XCLIBEXC", MB_OK|MB_TASKMODAL);
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

      case WM_HSCROLL:
	switch (GetWindowLong((HWND)lParam, GWL_ID)) {
	  case IDEXPSCROLL:
	    return(doExposureScroll(wParam));
	  case IDGAINSCROLL:
	    return(doGainScroll(wParam));
	  case IDGAIN2SCROLL:
	    return(doGainBScroll(wParam));
	  case IDOFFSETSCROLL:
	    return(doOffsetScroll(wParam));
	  case IDCLKSCROLL:
	    if (doClockScroll(wParam)) {
		if (liveon) // changing clock may imply turning video off
		    goto live;
		return(TRUE);
	    }
	    return(FALSE);
	  case IDFPSSCROLL:
	    if (doFpsScroll(wParam)) {
		if (liveon) // changing fps may imply turning video off
		    goto live;
		return(TRUE);
	    }
	    return(FALSE);
	}
	return(FALSE);

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

	DestroyWindow(hWnd);
	EndDialog(hDlg, 0);
	return(TRUE);

      case WM_PAINT:
	// our window was covered & uncovered
	displayImage();
	return(FALSE);

      case WM_TIMER:
	//
	// Monitor for asynchronous faults, such as video
	// being disconnected while capturing. These faults
	// can't be reported by functions such as pxd_goLive()
	// which initiate capture and return immediately.
	//
	// Should there be a fault pxd_mesgFault() pop up a dialg,
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
	    //
	    // Show
	    //	spatial resolution
	    //	bit depth & color
	    //	measured video rate
	    //	measured display rate
	    //	selected video format
	    //
	    // We are measuring the frame rate against the imprecise TIMER
	    // as a reference; the measurement will not be exact
	    // and may fluctuate. We do some averaging of the computed
	    // framerates to reduce fluctuations, at the expense of speed
	    // of response when the frame rate actually changes.
	    //
	    static DWORD      lasttickcount = 0;
	    static pxvbtime_t lastfieldcount = 0;
	    DWORD ticks = GetTickCount();
	    if ((ticks - lasttickcount) > 1000) { // update once per second
		pxvbtime_t  fields = pxd_videoFieldCount(1);
		double	    fps;
		char	    buf[100];
		buf[sizeof(buf)-1] = 0; // this & snprintf: overly conservative - avoids warning messages
		fps = lasttickcount?1000.0*(double)(fields-lastfieldcount)/((ticks-lasttickcount)*pxd_videoFieldsPerFrame()):0;
						    // the raw computed framerate
		_snprintf(buf, sizeof(buf)-1, "dim: %dx%d  res: %dx%d  fps: %.3f disp: %.3f",
			pxd_imageXdim(), pxd_imageYdim(), pxd_imageBdim(), pxd_imageCdim(), fps,
			1000.0*(double)displaycount/(ticks-lasttickcount));
		SetWindowText(GetDlgItem(hDlg, IDSTATUS1), buf);
		_snprintf(buf, sizeof(buf)-1, "camera: %s  fmt: %s",
		    "SILICON VIDEO",
		    #if defined(FORMAT)
			FORMAT);
		    #elif defined(FORMATFILE)
			FORMATFILE);
		    #else
			"");
		    #endif
		SetWindowText(GetDlgItem(hDlg, IDSTATUS2), buf);
		lasttickcount = ticks;
		lastfieldcount = fields;
		displaycount   = 0;
	    }
	}
	//
	// If using the timer to update image display, do so.
	//
	#if UPDATE_TIMER
	    displayImage();
	#endif

	//
	// Implement AGC, calling periodically?
	//
	#if SOFT_AGC
	    doSoftwareAxC();
	#endif
	return(TRUE);
    }
    return(FALSE);
}

/*
 * The Dialog
 * When using events from a different thread,
 * we must serialize access to XCLIB.
 * For simplicitty, we simply wrap this around the
 * non-serialized guts of the dialog.
 */
BOOL CALLBACK
PIXCIDialogProc(HWND hDlg, UINT wMsg, WPARAM wParam, LPARAM lParam)
{
    #if UPDATE_EVENT
	BOOL	b;
	EnterCriticalSection(&critsect);
	b = PIXCIDialogProc1(hDlg, wMsg, wParam, lParam);
	LeaveCriticalSection(&critsect);
	return(b);
    #else
	return(PIXCIDialogProc1(hDlg, wMsg, wParam, lParam));
    #endif

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
 * The Main.
 * Windows boilerplate.
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
			"XCLIBEXC Windows Example",
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

    //
    // Launch dialog.
    //
    InitializeCriticalSection(&critsect);
    hDlg = CreateDialogParam(hInstance, "SILICONVIDEODIALOG", NULL, (DLGPROC)PIXCIDialogProc, NULL);
    if (!hDlg) {
	MessageBox(NULL, "Missing Dialog Resource - Compilation or Link Error!", "XCLIBEXC", MB_OK|MB_TASKMODAL);
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
