/*
 *
 *	xclibex4.cpp	External	27-Dec-2010
 *
 *	Copyright (C)  1998-2010  EPIX, Inc.  All rights reserved.
 *
 *	Example program for the XCLIB 'C' Library.
 *	Example assumes Windows 95/98/ME/NT/2000/XP/Vista/7/XP(x64)/Vista(x64)/7(x64).
 *
 *	Demonstrates XCLIB and/or PXIPL functions for capture and
 *	display of sequences as well as saving of an image sequence.
 *	This Windows program must, of course, also make use of
 *	various Windows GDI API functions; however, this is not
 *	intended to serve as a Windows tutorial.
 *
 *	Also optionally demonstrates use of multiple PIXCI(R)
 *	imaging boards operating simultaneously.
 *
 *	Many parts similar to the XCLIBEX2.CPP example; that
 *	example provides PIXCI SV2, SV3, SV4, SV5, SV5A, SV5B, SV5L, SV6, and SV7
 *	specific controls, and demonstrates more display options including
 *	graphic overlays. It also demonstrates access to numeric
 *	pixel values and use of Events rather than a timer.
 *	For simplicity this example concentrates on sequence
 *	capture and display features and does not illustrate
 *	as many display options nor duplicate the other features
 *	demonstrated in that example.
 *
 */

/*
 *  INSTRUCTIONS:
 *
 *  1)	Set 'define' options below according to the intended camera
 *	and video format.
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
				  // EC1, ECB1, ECB1-34, ECB2, EL1, EL1DB, ELS2,
				  // SI, SI1, SI2, SI4
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
 *  3.1) Choose which form of image display is to be demonstrated.
 *  Some of these  options expect that the optional PXIPL library is present.
 *  Others may expect that the Windows DirectDraw SDK is present
 *  (available from Microsoft) and that the S/VGA supports DirectDraw.
 *
 *  Only one of these choices should have value 1, the others should be 0.
 *
 */
#if !defined(SHOWIM_STRETCHDIBITS) && !defined(SHOWIM_DRAWDIBDRAW) && !defined(SHOWIM_DRAWDIBDISPLAY) \
 && !defined(SHOWIM_GDIDISPLAY)    && !defined(SHOWIM_DIRECTXDISPLAY)

    #define SHOWIM_STRETCHDIBITS    1	// use XCLIB or XCLIB-Lite and GDI
    #define SHOWIM_DRAWDIBDRAW	    0	// use XCLIB or XCLIB-Lite and Video for Windows
    #define SHOWIM_DRAWDIBDISPLAY   0	// use XCLIB and PXIPL and Video for Windows
    #define SHOWIM_GDIDISPLAY	    0	// use XCLIB and PXIPL
    #define SHOWIM_DIRECTXDISPLAY   0	// use XCLIB and PXIPL and DirectDraw
#endif


/*
 *  3.2)  Choose whether the PXIPL Image Processing Library
 *	  is available for saving multiple images in one file.
 */
#if !defined(USE_PXIPL)
    #define USE_PXIPL	1
#endif


/*
 *  4)	Select whether the start of sequence capture should be triggered,
 *	by a general pupose input.
 *
 *	This is independent from the triggering of each image,
 *	of which some cameras are capable. The option below
 *	demonstrates a 'once per sequence' trigger, regardless
 *	of the camera's mode.
 */
#define TRIG_SEQUENCE	0	    // 0: don't use triggered sequence capture
#define TRIG_GPIN	0x01	    // which G.P. input, as a bitmap, to trigger upon
#define TRIG_DELAY	1000	    // number of fields to delay following
				    // trigger before commencing sequence capture



/*
 *  5)	Compile with Microsoft Visual Studio VS2010 as Win32 Platform
 *	application using command line tools:
 *
 *	    cl -I "\Program Files\EPIX\XCLIB" -c xclibex4.cpp
 *	    rc.exe /l 0x409 /fo tt.res	"\Program Files\EPIX\XCLIB\xclibex4.rc"
 *	    echo -subsystem:windows,4.0 -entry:WinMainCRTStartup comdlg32.lib ole32.lib >t1
 *	    echo /NODEFAULTLIB libc.lib oldnames.lib user32.lib gdi32.lib kernel32.lib vfw32.lib largeint.lib winmm.lib >t2
 *
 *	    Either (XCLIB w/out PXIPL for Windows NT/2000/XP/Vista/7)
 *		link xclibex4.obj "\Program Files\EPIX\XCLIB\XCLIBWNT.LIB" tt.res @t1 @t2
 *	    or	(XCLIB+PXIPL for Windows NT/2000/XP/Vista/7)
 *		link xclibex4.obj "\Program Files\EPIX\XCLIB\XCLIBWNT.LIB" "\Program Files\EPIX\XCLIB\PXIPLWNT.LIB" tt.res @t1 @t2
 *	    or (XCLIB-Lite for Windows NT/2000/XP/Vista/7)
 *		link xclibex4.obj "\Program Files\EPIX\XCLIB\XCLYBWNT.LIB" tt.res @t1 @t2
 *
 *	    del t1
 *	    del t2
 *	    del tt.res
 *	    del xclibex4.obj
 *
 *	Or, compile with Microsoft Visual Studio VS2010 as x64 Platform
 *	application using command line tools:
 *
 *	    cl -I "\Program Files\EPIX\XCLIB" -c xclibex4.cpp
 *	    rc.exe /l 0x409 /fo tt.res "\Program Files\EPIX\XCLIB\xclibex4.rc"
 *	    echo -subsystem:windows -machine:X64 -entry:WinMainCRTStartup comdlg32.lib ole32.lib >t1
 *	    echo /NODEFAULTLIB libcmt.lib oldnames.lib user32.lib gdi32.lib kernel32.lib vfw32.lib winmm.lib >t2
 *
 *	Either (XCLIB w/out PXIPL for Windows XP(x64)/Vista(x64)/7(x64)
 *	    link xclibex4.obj "\Program Files\EPIX\XCLIB\XCLIBW64.LIB" tt.res @t1 @t2
 *	or  (XCLIB+PXIPL for Windows XP(x64)/Vista(x64)/7(x64)
 *	    link xclibex4.obj "\Program Files\EPIX\XCLIB\XCLIBW64.LIB" "\Program Files\EPIX\XCLIB\PXIPLW64.LIB" tt.res @t1 @t2
 *
 *	    del t1
 *	    del t2
 *	    del tt.res
 *	    del xclibex4.obj
 *
 *    Or use the Microsoft Visual Studio VS2010 IDE, compiling xclibex4.cpp
 *    as a Windows NT/2000/XP/Vista/7 Win32 Platform or x64 Platform app,
 *    using the XCLIBEX4.RC resource, linking to library(s):
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
 *	   7. Copy XCLIBEX4.RC to your project directory.
 *	   8. Select 'Project' -> 'Add Existing Item' and select XCLIBEX4.RC.
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
 *  'XCLIBEX2.CPP' and XCLIBEX2.RC with 'XCLIBEX4.CPP' and 'XCLIBEX4.RC'.
 *
 */


/*
 *  NECESSARY INCLUDES:
 */
#include <windows.h>
#include <windowsx.h>
#include <stdio.h>
#include <stdlib.h>
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
 * Display specified buffer from specified unit,
 * in specified AOI of specified HWND,
 * using a compile-time selected method.
 */
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


/*
 * The Dialog
 */
BOOL CALLBACK
PIXCIDialogProc(HWND hDlg, UINT wMsg, WPARAM wParam, LPARAM lParam)
{
    static  UINT	svgaBits;			    // pixel format of S/VGA
    static  int 	liveon = 0;
    static  int 	seqdisplayon = 0;
    static  pxbuffer_t	seqdisplaybuf = 1;		    // which buffer being displayed?
    static  DWORD	seqdisplaytime; 		    // when was last buffer displayed
    static  pxvbtime_t	lastcapttime[UNITS] = {0};	    // when was image last captured
    static  struct	pxywindow windImage[max(4,UNITS)];  // subwindow of child window for image display
    static  HWND	hWndImage;			    // child window of dialog for image display
	    int 	err = 0;

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
	    // the compiler objects to this code (C2006) when FORMATFILE_COMP
	    // is not defined, even though this shouldn't be compiled
	    // when FORMATFILE_COMP is not defined.
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
	// Init dialog controls.
	//
	SetScrollRange(GetDlgItem(hDlg, IDBUFFERSCROLL), SB_CTL, 1, pxd_imageZdim(), TRUE);
	EnableWindow(GetDlgItem(hDlg, IDLIVE),	      TRUE);
	EnableWindow(GetDlgItem(hDlg, IDSNAP),	      TRUE);
	EnableWindow(GetDlgItem(hDlg, IDSEQCAPTURE),  TRUE);
	EnableWindow(GetDlgItem(hDlg, IDSEQDISPLAY),  TRUE);
	EnableWindow(GetDlgItem(hDlg, IDSTOP),	      FALSE);
	EnableWindow(GetDlgItem(hDlg, IDBUFFERSCROLL),TRUE);
	#if USE_PXIPL
	    EnableWindow(GetDlgItem(hDlg, IDSEQSAVE),  TRUE);
	#else
	    EnableWindow(GetDlgItem(hDlg, IDSEQSAVE),  FALSE);
	#endif

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
	    liveon = FALSE;
	    seqdisplaybuf = FALSE;
	    err = pxd_goSnap(UNITSMAP, 1);
	    if (err < 0)
		MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_goSnap", MB_OK|MB_TASKMODAL);
	    EnableWindow(GetDlgItem(hDlg, IDLIVE),	  TRUE);
	    EnableWindow(GetDlgItem(hDlg, IDSNAP),	  TRUE);
	    EnableWindow(GetDlgItem(hDlg, IDSEQCAPTURE),  TRUE);
	    EnableWindow(GetDlgItem(hDlg, IDSEQDISPLAY),  TRUE);
	    EnableWindow(GetDlgItem(hDlg, IDSTOP),	  FALSE);
	    SetScrollPos(GetDlgItem(hDlg, IDBUFFERSCROLL), SB_CTL, 1, TRUE);
	    return(TRUE);

	  case IDLIVE:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    liveon = TRUE;
	    seqdisplaybuf = FALSE;
	    err = pxd_goLive(UNITSMAP, 1L);
	    if (err < 0)
		MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_goLive", MB_OK|MB_TASKMODAL);
	    EnableWindow(GetDlgItem(hDlg, IDLIVE),	  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSNAP),	  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSEQCAPTURE),  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSEQDISPLAY),  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSTOP),	  TRUE);
	    EnableWindow(GetDlgItem(hDlg, IDBUFFERSCROLL),FALSE);
	    SetScrollPos(GetDlgItem(hDlg, IDBUFFERSCROLL), SB_CTL, 1, TRUE);
	    return(TRUE);

	  case IDSTOP:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    pxd_goUnLive(UNITSMAP);
	    liveon = FALSE;
	    seqdisplayon = FALSE;
	    EnableWindow(GetDlgItem(hDlg, IDLIVE),	  TRUE);
	    EnableWindow(GetDlgItem(hDlg, IDSNAP),	  TRUE);
	    EnableWindow(GetDlgItem(hDlg, IDSEQCAPTURE),  TRUE);
	    EnableWindow(GetDlgItem(hDlg, IDSEQDISPLAY),  TRUE);
	    EnableWindow(GetDlgItem(hDlg, IDSTOP),	  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDBUFFERSCROLL),TRUE);
	    return(TRUE);

	  case IDSEQCAPTURE:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    //
	    // Reminder: The pxd_goLiveSeq and pxd_goLiveSeqTrig
	    // return immediately with the sequence capture running
	    // in the background. In the context of this example program,
	    // being 'user-event-driven', we prefer not waiting for completion
	    // of the sequence capture.
	    //
	    #if TRIG_SEQUENCE
		err = pxd_goLiveSeqTrig(UNITSMAP, 1, pxd_imageZdim(), 1, 0, 1,
				  0, 0,
				  TRIG_GPIN, 1, TRIG_DELAY,
				  0, 0, 0, 0, 0, 0, 0,
				  0, 0, 0,
				  0, 0, 0, 0, 0, 0);
	    #else
		err = pxd_goLiveSeq(UNITSMAP, 1, pxd_imageZdim(), 1, 0, 1);
	    #endif
	    if (err < 0)
		MessageBox(NULL, pxd_mesgErrorCode(err), "pxd_goLiveSeq", MB_OK|MB_TASKMODAL);
	    liveon = FALSE;
	    seqdisplayon = FALSE;
	    EnableWindow(GetDlgItem(hDlg, IDLIVE),	  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSNAP),	  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSEQCAPTURE),  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSEQDISPLAY),  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSTOP),	  TRUE);
	    // Enable scroll so it can show capture status;
	    // it doesn't allow changing the currently capture buffer.
	    EnableWindow(GetDlgItem(hDlg, IDBUFFERSCROLL),TRUE);
	    SetScrollPos(GetDlgItem(hDlg, IDBUFFERSCROLL), SB_CTL, 1, TRUE);
	    return(TRUE);

	  case IDSEQDISPLAY:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    seqdisplaybuf = 1;
	    liveon = FALSE;
	    seqdisplayon = TRUE;
	    seqdisplaytime = GetTickCount();
	    EnableWindow(GetDlgItem(hDlg, IDLIVE),	  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSNAP),	  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSEQCAPTURE),  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSEQDISPLAY),  FALSE);
	    EnableWindow(GetDlgItem(hDlg, IDSTOP),	  TRUE);
	    EnableWindow(GetDlgItem(hDlg, IDBUFFERSCROLL),FALSE);
	    SetScrollPos(GetDlgItem(hDlg, IDBUFFERSCROLL), SB_CTL, 1, TRUE);
	    return(TRUE);

	  case IDSEQSAVE:
	    if (HIWORD(wParam) != BN_CLICKED)
		return(FALSE);
	    #if USE_PXIPL
	    {
		OPENFILENAME ofn;
		char	pathname[_MAX_PATH] = "";
		int	r;
		memset(&ofn, 0, sizeof(ofn));
		ofn.lStructSize = sizeof(ofn);
		ofn.hwndOwner	= hWnd;
		ofn.lpstrFilter = "TIFF Files (*.tif)\0*.tif\0\0";
		ofn.lpstrFile	= pathname;
		ofn.nMaxFile	= _MAX_PATH;
		ofn.lpstrTitle	= "Save Sequence";
		ofn.Flags   |= OFN_EXPLORER|OFN_OVERWRITEPROMPT|OFN_HIDEREADONLY;
		r = GetSaveFileName(&ofn);
		if (r != 0) {
		    err = pxio8_tifwriteseq(NULL,
					  pxd_defineImage3(0x1, 1, -1, 0, 0, -1, -1, "Default"),
					  NULL, pathname, pxd_imageBdim(), 0, 0, NULL);
		    if (err < 0)
			MessageBox(NULL, pxd_mesgErrorCode(err), "pxio8_tifwriteseq", MB_OK|MB_TASKMODAL);
		}
	    }
	    #endif
	    return(TRUE);
	}
	break;

      case WM_HSCROLL:
      {
	HWND hCtrl = (HWND)lParam;
	switch (GetWindowLong(hCtrl, GWL_ID)) {
	case IDBUFFERSCROLL:
	  {
	    if (liveon) {
		SetScrollPos(GetDlgItem(hDlg, IDBUFFERSCROLL), SB_CTL, pxd_capturedBuffer(1), TRUE);
	    } else {
		pxbuffer_t b = seqdisplaybuf;
		switch (LOWORD(wParam)) {
		case SB_PAGEDOWN:	b += 5; 		break;
		case SB_LINEDOWN:	b += 1; 		break;
		case SB_PAGEUP: 	b -= 5; 		break;
		case SB_LINEUP: 	b -= 1; 		break;
		case SB_TOP:		b = pxd_imageZdim();	break;
		case SB_BOTTOM: 	b = 1;			break;
		case SB_THUMBPOSITION:
		case SB_THUMBTRACK:	b = HIWORD(wParam); break;
		default:
		    return(FALSE);
		}
		b = max(1, min(pxd_imageZdim(), b));
		SetScrollPos(GetDlgItem(hDlg, IDBUFFERSCROLL), SB_CTL, b, TRUE);
		seqdisplaybuf = b;
		if (!seqdisplayon)
		    for (int u = 0; u < UNITS; u++)
			DisplayBuffer(u, b, hWndImage, windImage);
	    }
	    return(TRUE);
	  }
	}
	return(FALSE);
      }

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
	// Should there be a fault and pxd_mesgFault() pop up a dialog,
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
	// Or, in sequence display mode, is it
	// time to display the next image?
	//
	// In sequence capture, the PIXCI driver is handles
	// switching from one capture buffer to the next -
	// this need only monitor the result.
	// During sequence display, this determines when,
	// and it what order, each previously captured buffer
	// should be displayed.
	//
	pxbuffer_t  buf = 1;
	for (int u = 0; u < UNITS; u++) {
	    if (seqdisplayon) {
		if (u == 0) {
		    if (seqdisplaytime+500 > GetTickCount())
			break;	// no display yet, all units.
		    seqdisplaytime = GetTickCount();
		    buf = seqdisplaybuf++;
		    if (seqdisplaybuf > pxd_imageZdim())
			seqdisplaybuf = 1;
		}
	    } else {
		pxvbtime_t lasttime = pxd_capturedFieldCount(1<<u);
		if (lastcapttime[u] == lasttime)
		    continue;
		lastcapttime[u] = lasttime;
		buf = pxd_capturedBuffer(1<<u);
	    }
	    DisplayBuffer(u, buf, hWndImage, windImage);
	    //
	    // Let buffer scroll bar show sequence capture activity.
	    // Especially useful in triggered sequence mode, as it
	    // will show when the trigger has arrived and the delay
	    // expired so as to let the sequence capture run.
	    //
	    SetScrollPos(GetDlgItem(hDlg, IDBUFFERSCROLL), SB_CTL, buf, TRUE);
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
	MessageBox(NULL, "Missing Dialog Resource - Compilation or Link Error!", "XCLIBEX4", MB_OK|MB_TASKMODAL);
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
