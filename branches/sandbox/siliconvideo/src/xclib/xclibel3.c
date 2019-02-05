/*
 *
 *	xclibel3.c	External	13-Nov-2010
 *
 *	Copyright (C)  2004-2010  EPIX, Inc.  All rights reserved.
 *
 *	Example program for XCLIB C Library, SCF Level functions.
 *	Example assumes Linux 'command line' environment.
 *	Assumes GTK+2.0 libraries installed. See http://www.gtk.org
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
 *	imaging boards, use "default" to select the default format for the camera
 *	for which the PIXCI(R) imaging board is intended.
 *	For non default formats, use XCAP to save the video set-up to a
 *	file, and set FORMAT to the saved file's name.
 *	For camera's with RS-232 control, note that the saved
 *	video set-up only resets the PIXCI(R) imaging board's
 *	settings, but XCLIB does not reset the camera's settings.
 *
 *	Alternately, this could be modified to use any
 *	other convention chosen by the programmer to allow
 *	run time selection of the video format and resolution.
 *
 */

#if !defined(FORMAT) && !defined(FORMATFILE)
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
    #define FORMAT  "default"	  // NSTC

				  // For PIXCI(R) A, CL1, CL2, CL3SD, D, D24, D32,
				  // D2X, D3X, E1, E1DB, E4, E4DB, EB1, EB1POCL,
				  // EC1, ECB1, ECB1-34, ECB2, EL1, EL1DB,
				  // ELS2, SI, SI1, SI2, SI4
  //#define FORMAT  "default"	  // as per board's intended camera

					  // For any PIXCI(R) imaging board
  //#define FORMATFILE	"xcvidset.fmt"	  // using format file saved by XCAP
#endif


/*
 *  2.1) Set number of expected PIXCI(R) image boards.
 *  The XCLIB Simple 'C' Functions expect that the boards are
 *  identical and operated at the same resolution.
 *
 *  For PIXCI(R) imaging boards with multiple, functional units,
 *  the XCLIB presents the two halves of the
 *  PIXCI\*(Rg\ E1DB, E4DB, ECB2, EL1DB, ELS2, SI2, or SV7 imaging boards
 *  or the four quarters of the PIXCI\*(Rg\ SI4 imaging board
 *  as two or four independent PIXCI\*(Rg\ imaging boards, respectively.
 *
 *  This example expects only one unit.
 */
#if !defined(UNITS)
    #define UNITS	1
#endif
#define UNITSMAP    ((1<<UNITS)-1)  // shorthand - bitmap of all units


/*
 *  2.2) Optionally, set driver configuration parameters.
 *  These are normally left to the default, "".
 *  The actual driver configuration parameters include the
 *  desired PIXCI(R) imaging boards, but to make configuation easier,
 *  code, below, will automatically add board selection to this.
 *
 *  Note: Under Linux, the image frame buffer memory can't be set as
 *  a run time option. It MUST be set via insmod so the memory can
 *  be reserved during Linux's initialization.
 */
#if !defined(DRIVERPARMS)
  //#define DRIVERPARMS "-QU 0"       // don't use interrupts
  //#define DRIVERPARMS "-IM 8192"    // request 8192 mbyte of frame buffer memory
    #define DRIVERPARMS ""	      // default
#endif

/*
 *  3)	Choose whether the optional PXIPL Image Processing Library
 *	is available, and some PXIPL functions should be demonstrated.
 */
#if !defined(USE_PXIPL)
    #define USE_PXIPL	1
#endif


/*
 *  4)	Compile with GCC w/out PXIPL for 32 bit Linux as:
 *
 *	    gcc -DC_GNU32=400 -DOS_LINUX_GNU xclibel3.c xclib_i386.a -o xclibel3 -lm `pkg-config gtk+-2.0 --cflags --libs`
 *    
 *	Compile with GCC with PXIPL for 32 bit Linux as:
 *
 *	    gcc -DC_GNU32=400 -DOS_LINUX_GNU xclibel3.c pxipl_i386.a xclib_i386.a -o xclibel3 -lm `pkg-config gtk+-2.0 --cflags --libs`
 *
 *	Compile with GCC w/out PXIPL for 64 bit Linux as:
 *
 *	    gcc -DC_GNU64=400 -DOS_LINUX_GNU xclibel3.c xclib_x86_64.a -o xclibel3 -lm `pkg-config gtk+-2.0 --cflags --libs`
 *    
 *	Compile with GCC with PXIPL for 64 bit Linux as:
 *
 *	    gcc -DC_GNU64=400 -DOS_LINUX_GNU xclibel3.c pxipl_x86_64.a xclib_x86_64.a -o xclibel3 -lm `pkg-config gtk+-2.0 --cflags --libs`
 *
 *	Run as:
 *	    ./xclibel3
 *
 */


/*
 *  NECESSARY INCLUDES:
 */
#include <stdio.h>		// c library
#include <signal.h>		// c library
#include <stdlib.h>		// c library
#include <stdarg.h>		// c library
#include "xcliball.h"		// function prototypes
#if USE_PXIPL
#include "pxipl.h"		// function prototypes
#endif
#include <gtk/gtk.h>		// GTK Window Library
#include <gdk/gdkx.h>


/* 
 * GTK Image Buffer and Status Bar and Global Variables 
 */
GtkWidget   *display_area;
GtkWidget   *image_buffer_display, *field_count_display, *display_rate_display;
gboolean    live_display = FALSE;
gboolean    live_snap = FALSE;
gboolean    live_pair = FALSE;
gboolean    update_window = FALSE;
gboolean    buffer_toggle = FALSE;
gint 	    display_width, display_height;
guint	    field_count = 0;
guint	    display_count = 0;		// how often the display is updated
pxvbtime_t  lastcapttime = 0;		// when was image last captured
#if USE_PXIPL	
struct pxywindow   pxywindow_display_size;
struct pximage     *pximage;
#else
guchar	    *rgbbuf = NULL;	// GTK image buffer
size_t      rgbbufsize;         // Size of buffer
#endif
gboolean    PIXCI_is_SV234567 = FALSE;


/*
 *  SUPPORT STUFF:
 *
 *  Catch CTRL+C and floating point exceptions so that once
 *  opened, the PIXCI(R) library is always closed before exit.
 *  In some environments, this isn't critical; the operating system
 *  advises the PIXCI(R) driver that the program has terminated.
 *  In other environments, the operating system isn't as helpful
 *  and the driver would be left open.
 */
void signaled(int sig)
{
    /*
     * Some languages/environments don't allow
     * use of printf from a signal catcher.
     * Printing the message isn't important anyhow.
     *
    if (sig == SIGINT)
	printf("Break\n");
    if (sig == SIGFPE)
	printf("Float\n");
    */

    pxd_PIXCIclose();
    exit(1);
}


/*
 * Open the XCLIB C Library for use.
 */
int do_open(void)
{
    int i;

    //
    // Either FORMAT or FORMATFILE should have been
    // selected above.
    //
    #if defined(FORMAT)
	i = pxd_PIXCIopen(DRIVERPARMS, FORMAT, "");
    #elif defined(FORMATFILE)
	i = pxd_PIXCIopen(DRIVERPARMS, "", FORMATFILE);
    #endif
    if (i >= 0) {
	g_print( "%s\n", "Open PIXCI" );   // success!
    } else {
	printf("Open Error %d\a\a\n", i);
	pxd_mesgFault(UNITSMAP);
    }
    switch (pxd_infoModel(UNITSMAP)) {
      case PIXCI_SV2:
      case PIXCI_SV3:
      case PIXCI_SV4:
      case PIXCI_SV5:
      case PIXCI_SV6:
      case PIXCI_SV7:
	PIXCI_is_SV234567 = TRUE;
	break;
    }
    return(i);
}

/*
 * Utilities for SV2/3/4/5/7.
 * Get/set current brightness and gain/contrast.
 * These translate the XCLIB's generic floating, scaled,
 * values to integers between 0 & 100, so they will be
 * useful for scroll bars.
 */
int getBright()
{
    if (pxd_infoModel(1) == PIXCI_SV7)
	return((int)(50+pxd_getBrightness(1)*100/(76*2)));
    else
	return((int)(pxd_getBrightness(1)+50));
}
void setBright(int b)
{
    if (pxd_infoModel(1) == PIXCI_SV7)
	pxd_setContrastBrightness(UNITSMAP, pxd_getContrast(1), (b-50)*76*2/100);
    else
	pxd_setContrastBrightness(UNITSMAP, pxd_getContrast(1), b-50);
}
int getGain()
{
    if (pxd_infoModel(1) == PIXCI_SV7)
	return((int)(pxd_getContrast(1)/2));
    else
	return((int)(pxd_getContrast(1)*100./237.07));
}
void setGain(int b)
{
    if (pxd_infoModel(1) == PIXCI_SV7)
	pxd_setContrastBrightness(UNITSMAP, b*2, pxd_getBrightness(1)*2);
    else
	pxd_setContrastBrightness(UNITSMAP, (b*237.07/100.), pxd_getBrightness(1));   // + 255 for rounding!
}


/* Refreshes the display Window*/
gboolean update_display(gpointer user_data)
{

    if (live_snap) {
	if (field_count != pxd_videoFieldCount(1)) {
	    pxd_goSnap(UNITSMAP, 1L);
	    field_count = pxd_videoFieldCount(1);
	}
    }

    if (lastcapttime != pxd_capturedFieldCount(1) || update_window) {
	lastcapttime = pxd_capturedFieldCount(1);
	update_window = FALSE;

	#if USE_PXIPL
	    /* Use PXIPL for Display - Requires PXIPL Library */
	    /* Performs video scaling into display window. */
	    pximage = pxd_defineImage(1, 1 + buffer_toggle, 0, 0, -1, -1, "Display");
	    pxio8_X11Display ( 0, pximage, 0, 0, 'n', 0, 0, GDK_WINDOW_XDISPLAY(display_area->window),
		GDK_DRAWABLE_XID (display_area->window) , 0, 0, &pxywindow_display_size, 0, 0);
	#else
	    /* Use GDK for Display */
	    /* Performs a one to one display the size of the window. */
	    pxd_readuchar(UNITSMAP, 1 + buffer_toggle, 0, 0, pxd_imageXdim(), pxd_imageYdim(), rgbbuf, rgbbufsize, "RGB");
	    gdk_draw_rgb_image (display_area->window, display_area->style->fg_gc[GTK_STATE_NORMAL],
		0, 0, display_width, display_height, GDK_RGB_DITHER_NONE, rgbbuf, pxd_imageXdim() * 3);
	#endif

	if (live_pair)
		buffer_toggle=!buffer_toggle;

	display_count++;
    }
  
    return TRUE;
}


/* Snap an Image */
gboolean do_snap (GtkWidget *widget, GdkEventExpose *event, gpointer user_data)
{

    buffer_toggle = FALSE;

    if ( live_display ) {
	live_display = FALSE;
	live_pair = FALSE;
	live_snap = FALSE;
	pxd_goUnLive(UNITSMAP);
        g_print ( "%s\n", "UnLive" );
	return TRUE;
    }

    pxd_goSnap(UNITSMAP, 1L);
    g_print ( "%s\n", "Snap" );

    return TRUE;
}

/* Toggle Live Display */
gboolean do_live (GtkWidget *widget, GdkEventExpose *event, gpointer user_data)
{
    live_display = !live_display;
    live_snap = FALSE;
    live_pair = FALSE;
    buffer_toggle = FALSE;

    if ( live_display ) {
  	pxd_goLive(UNITSMAP, 1L);
        g_print ( "%s\n", "Live" );
    } else {
  	pxd_goUnLive(UNITSMAP);
        g_print ( "%s\n", "UnLive" );
    }
  
    return TRUE;
}

/* Toggle Live Snap Display */
gboolean do_live_snap (GtkWidget *widget, GdkEventExpose *event, gpointer user_data)
{
    if ( live_snap != live_display ) {
	live_display = FALSE;
	live_pair = FALSE;
	pxd_goUnLive(UNITSMAP);
        g_print ( "%s\n", "UnLive" );
	return TRUE;
    }

    live_snap = !live_snap;
    live_display = live_snap;
    live_pair = FALSE;
    buffer_toggle = FALSE;
    g_print ( "%s\n", "Live Snap" );

    return TRUE;
}

/* Toggle Live Display */
gboolean do_live_pair (GtkWidget *widget, GdkEventExpose *event, gpointer user_data)
{
    if ( live_pair != live_display ) {
	live_display = FALSE;
	live_snap = FALSE;
	live_pair = FALSE;
	pxd_goUnLive(UNITSMAP);
        g_print ( "%s\n", "UnLive" );
	return TRUE;
    }

    live_pair = !live_pair;

    if ( live_pair ) {
	live_display = TRUE;
  	pxd_goLivePair(UNITSMAP, 1L, 2L);
        g_print ( "%s\n", "Live Pair" );
    } else {
	live_display = FALSE;
  	pxd_goUnLive(UNITSMAP);
        g_print ( "%s\n", "UnLive" );
    }
  
    return TRUE;
}

/* Swap Display Buffer */
gboolean do_swap_pair (GtkWidget *widget, GdkEventExpose *event, gpointer user_data)
{

    update_window = TRUE;
    buffer_toggle =! buffer_toggle;

    if ( live_display ) {
	live_display = FALSE;
	live_snap = FALSE;
	live_pair = FALSE;
	pxd_goUnLive(UNITSMAP);
        g_print ( "%s\n", "UnLive" );
    }
  
    g_print ( "%s\n", "Swap" );

    return TRUE;
}

/* PXIPL Complement - Requires PXIPL Library */
#if USE_PXIPL 
gboolean do_pxipl_comp (GtkWidget *widget, GdkEventExpose *event, gpointer user_data)
{

    update_window = TRUE;

    if ( live_display ) {
	live_display = FALSE;
	live_snap = FALSE;
	live_pair = FALSE;
	pxd_goUnLive(UNITSMAP);
        g_print ( "%s\n", "UnLive" );
    }

    /*
     * On some boards, the pxd_defineImage(..., "Default")
     * might be YCrCb. While complementing YCrCb data would work,
     * the result might not be what the viewer expects.
     */
    if (pxd_imageCdim() == 1)
	pximage = pxd_defineImage(1, 1, 0, 0, pxd_imageXdim()/2, pxd_imageYdim()/2, "Grey");
    else
	pximage = pxd_defineImage(1, 1, 0, 0, pxd_imageXdim()/2, pxd_imageYdim()/2, "RGB");
    pxip8_pixneg(0, pximage, pximage);

    g_print ( "%s\n", "PXIPL Compliment" );

    return TRUE;
}
#endif

/* Called by timer to update Status Bars once every second */

gboolean update_status (gpointer user_data)
{
    static gchar *status;
    static guint last_display_count = 0;
    //
    // Display field count in status bar.
    //
    status = g_strdup_printf ("%d", pxd_videoFieldCount(1) );
    gtk_label_set_text (GTK_LABEL (field_count_display), status);
    //
    // Display number of image buffers in status bar.
    //
    status = g_strdup_printf ("%d", pxd_imageZdim() );
    gtk_label_set_text (GTK_LABEL ( image_buffer_display ), status );

    //
    // Display update rate of display area.
    //
    status = g_strdup_printf ("%d", (display_count - last_display_count) );
    gtk_label_set_text (GTK_LABEL ( display_rate_display ), status );
    last_display_count = display_count;
    return TRUE;
}

/* Change Contrast for PIXCI SV2/3/4/5 */
void change_contrast (GtkAdjustment *adjust, gpointer user_data)
{
    setGain(adjust->value);
    update_window = TRUE;
}

/* Change Brightness */
void change_brightness (GtkAdjustment *adjust, gpointer user_data)
{
    setBright(adjust->value);
    update_window = TRUE;
}

/* Redraw window */
expose_event( GtkWidget *widget, GdkEventExpose *event )
{
    update_window = TRUE;
    return FALSE;
}

/* Handles Closing the Window */
void close_window ( GtkWidget * window, gpointer data )
{
    pxd_goUnLive(UNITSMAP);
    g_print ( "%s\n", "Close Window" );
    gtk_main_quit ();
}

/*
 * Close the PIXCI(R) imaging board
 */
void do_close()
{
    g_print ( "%s\n", "Close PIXCI" );
    pxd_PIXCIclose();
}


/*
 *  MAIN:
 *
 *  Each library function is demonstrated in its own subroutine,
 *  the main calls each subroutine to produce the interactive demonstration.
 *
 *  It is suggested that this program source be read at the same time
 *  as the compiled program is executed.
 *
 */
int main (int argc, char *argv[])
{
    //
    // Setup and initialize GTK window library.
    //
    GtkWidget *main_window, *vert_box, *control_table;
    GtkWidget *snap_button, *live_button, *live_snap_button, *pxipl_button, *swap_pair_button, *live_pair_button;
    GtkWidget *field_count_label, *image_buffer_label, *display_rate_label;
    // Contrast and Brightness controls for PIXCI SV2/3/4/5/6/7
    GtkWidget *contrast_slider, *brightness_slider;
    GtkWidget *contrast_label, *brightness_label;
    GtkObject *contrast_adjust, *brightness_adjust;

    gtk_init (&argc, &argv);

    //
    // Catch signals.
    //
    signal(SIGINT, signaled);
    signal(SIGFPE, signaled);
    //
    // Open and set video format.
    //
    if (do_open() < 0)
	return(1);

    //
    // Setup Display window size, make sure it is smaller than the desktop
    //
    display_width  = MIN ( pxd_imageXdim(), gdk_screen_get_width (gdk_screen_get_default ())/2 );
    display_height = MIN ( pxd_imageYdim(), gdk_screen_get_height(gdk_screen_get_default ())/2 );

    #if USE_PXIPL
	// Set the display window size
	pxywindow_display_size.nw.x = 0;
	pxywindow_display_size.nw.y = 0;
	pxywindow_display_size.se.x = display_width;
	pxywindow_display_size.se.y = display_height;
    #else 
	rgbbuf = malloc (rgbbufsize = pxd_imageXdim() * pxd_imageYdim() * 3 );
    #endif

    g_print ( "Display  Width: %d\n", display_width   );
    g_print ( "Display Height: %d\n", display_height  );
    g_print ( "PIXCI(R) X Dim: %d\n", pxd_imageXdim() );
    g_print ( "PIXCI(R) Y Dim: %d\n", pxd_imageYdim() );

    //
    // Define GTK windows and controls.
    //
    main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (main_window), "EPIX(R) PIXCI(R) GTK 2.0 Example" );
    vert_box = gtk_vbox_new ( FALSE, 0);
    display_area = gtk_drawing_area_new();
    gtk_widget_set_size_request (display_area, display_width, display_height);
    control_table = gtk_table_new (10, 10, FALSE);
    snap_button = gtk_button_new_with_label ( "Snap" );
    live_button = gtk_button_new_with_label ( "Live" );
    live_snap_button = gtk_button_new_with_label ( "Live Snap" );
    #if USE_PXIPL
	pxipl_button = gtk_button_new_with_label ( "PXIPL Compliment" );
    #endif
    swap_pair_button = gtk_button_new_with_label ( "Swap Pair" );
    live_pair_button = gtk_button_new_with_label ( "Live Pair" );
    if (PIXCI_is_SV234567) {  // Contrast and Brightness controls for PIXCI SV2/3/4/5/6/7
	contrast_adjust = GTK_OBJECT ( gtk_adjustment_new (getGain(), 0, 100, 1, 10, 0) ); // 100 is max
	brightness_adjust = GTK_OBJECT ( gtk_adjustment_new (getBright(), 0, 100, 1, 10, 0) ); // 100 is max
	contrast_slider = gtk_hscale_new ( GTK_ADJUSTMENT(contrast_adjust) );
	brightness_slider = gtk_hscale_new ( GTK_ADJUSTMENT(brightness_adjust) );
	gtk_scale_set_digits ( GTK_SCALE ( contrast_slider ), 0 );
	gtk_scale_set_digits ( GTK_SCALE ( brightness_slider ), 0 );
	gtk_scale_set_draw_value ( GTK_SCALE ( contrast_slider ), TRUE );
	gtk_scale_set_draw_value ( GTK_SCALE ( brightness_slider ), TRUE );
	contrast_label = gtk_label_new ( "Adjust Contrast:" );
	gtk_label_set_justify ( GTK_LABEL (contrast_label), GTK_JUSTIFY_LEFT );
	brightness_label = gtk_label_new ( "Adjust Brightness:" );
	gtk_label_set_justify ( GTK_LABEL (brightness_label), GTK_JUSTIFY_RIGHT );
    }
    field_count_label = gtk_label_new ( "Field Count:" );
    image_buffer_label = gtk_label_new ( "Image Buffers:" );
    display_rate_label = gtk_label_new ( "Display Rate:" );
    field_count_display = gtk_label_new ( "0" );
    image_buffer_display = gtk_label_new ( "0" );
    display_rate_display = gtk_label_new ( "0" );

    //
    // Put display area in box, layout controls in table, put table in box, and place box in main window
    //
    gtk_box_pack_start (GTK_BOX (vert_box), display_area, FALSE, FALSE, 0);
    gtk_table_attach_defaults (GTK_TABLE (control_table), snap_button, 0, 3, 0, 2);
    gtk_table_attach_defaults (GTK_TABLE (control_table), live_snap_button, 0, 3, 2, 4);
    #if USE_PXIPL
	gtk_table_attach_defaults (GTK_TABLE (control_table), live_button, 3, 7, 0, 2);
	gtk_table_attach_defaults (GTK_TABLE (control_table), pxipl_button, 3, 7, 2, 4);
    #else
	gtk_table_attach_defaults (GTK_TABLE (control_table), live_button, 3, 7, 0, 4);
    #endif
    gtk_table_attach_defaults (GTK_TABLE (control_table), live_pair_button, 7, 10, 0, 2);
    gtk_table_attach_defaults (GTK_TABLE (control_table), swap_pair_button, 7, 10, 2, 4);
    if (PIXCI_is_SV234567) {  // Contrast and Brightness controls for PIXCI SV2/3/4/5/6/7
	gtk_table_attach_defaults (GTK_TABLE (control_table), contrast_label, 0, 2, 4, 5);
	gtk_table_attach_defaults (GTK_TABLE (control_table), contrast_slider, 2, 10, 4, 5);
	gtk_table_attach_defaults (GTK_TABLE (control_table), brightness_label, 0, 2, 5, 6);
	gtk_table_attach_defaults (GTK_TABLE (control_table), brightness_slider, 2, 10, 5, 6);
    }
    gtk_table_attach_defaults (GTK_TABLE (control_table), field_count_label, 0, 2, 9, 10);
    gtk_table_attach_defaults (GTK_TABLE (control_table), field_count_display, 2, 4, 9, 10);
    gtk_table_attach_defaults (GTK_TABLE (control_table), image_buffer_label, 4, 6, 9, 10);
    gtk_table_attach_defaults (GTK_TABLE (control_table), image_buffer_display, 6, 7, 9, 10);
    gtk_table_attach_defaults (GTK_TABLE (control_table), display_rate_label, 7, 9, 9, 10);
    gtk_table_attach_defaults (GTK_TABLE (control_table), display_rate_display, 9, 10, 9, 10);
    gtk_box_pack_end (GTK_BOX (vert_box), control_table, FALSE, FALSE, 0);
    gtk_container_add (GTK_CONTAINER (main_window), vert_box);

    // The following timers control how often in milliseconds the display window is checked
    // for updating and how frequently the counters are updated.
    // Note: On slower PCs the second timer seems to get ignored.
    g_timeout_add (2, (GtkFunction) update_display, NULL);
    g_timeout_add (1000, (GtkFunction) update_status, NULL);

    //
    // Setup GTK control signals to call functions.
    //
    g_signal_connect (G_OBJECT ( snap_button ), "clicked",
			G_CALLBACK ( do_snap ), NULL);
    g_signal_connect (G_OBJECT ( live_button ), "clicked",
			G_CALLBACK ( do_live ), NULL);
    g_signal_connect (G_OBJECT ( live_snap_button ), "clicked",
			G_CALLBACK ( do_live_snap ), NULL);
    #if USE_PXIPL
	g_signal_connect (G_OBJECT ( pxipl_button ), "clicked",
			  G_CALLBACK ( do_pxipl_comp ), NULL);
    #endif
    g_signal_connect (G_OBJECT ( live_pair_button ), "clicked",
			G_CALLBACK ( do_live_pair ), NULL);
    g_signal_connect (G_OBJECT ( swap_pair_button ), "clicked",
			G_CALLBACK ( do_swap_pair ), NULL);
    if (PIXCI_is_SV234567) { // Contrast and Brightness controls for PIXCI SV2/3/4/5/6/7
	g_signal_connect (G_OBJECT ( contrast_adjust ), "value_changed",
			    G_CALLBACK ( change_contrast ), NULL);
	g_signal_connect (G_OBJECT ( brightness_adjust ), "value_changed",
			    G_CALLBACK ( change_brightness ), NULL);
    }
    g_signal_connect (G_OBJECT ( display_area ), "expose_event",
			G_CALLBACK ( expose_event ), NULL);
    g_signal_connect (G_OBJECT ( main_window ), "destroy",
			G_CALLBACK ( close_window ), NULL);

    //
    // Display GTK windows
    //
    gtk_widget_show_all (main_window);

    //
    // Run GTK Program
    //
    gtk_main();

    //
    // All done
    //
    do_close();
    #if USE_PXIPL
    #else
	free(rgbbuf);
    #endif
    return 0;
}
