//#include <pthread.h>
#include <unistd.h>
#include <stdio.h>		// c library
#include <signal.h>		// c library
#include <stdlib.h>		// c library
#include <stdarg.h>		// c library
#include "xclib/xcliball.h"    	// function prototypes
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
//#include <dynamic_reconfigure/server.h>
//#include <siliconvideo/svstereoConfig.h>

// SSE instructions
#include <xmmintrin.h>
#include <emmintrin.h>
#include <mmintrin.h>


#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define MIN(x,y) ((x) < (y) ? (x) : (y))


#define DRIVERPARMS ""	          // default
#define FORMAT "default"	  // NSTC S-Video on input 1
#define FORMATFILE ""             // no format file

#define UNITS	2
#define UNITSMAP    ((1<<UNITS)-1)  // shorthand - bitmap of all units

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

struct __attribute__((__aligned__(16))) image_buffer {
  uchar data[IMAGE_WIDTH*IMAGE_HEIGHT];
  ros::Time stamp;
};

image_buffer image1_buf[3], image2_buf[3];
int image1_last_buf = 0, image2_last_buf = 0;
sensor_msgs::Image image1_msg, image2_msg;
sensor_msgs::CameraInfo cinfo1, cinfo2;
ros::Publisher image1_pub, image2_pub, cinfo1_pub, cinfo2_pub;

// live mode
bool live_mode = true;
bool got1 = false;
bool got2 = false;
bool publishing = false;
//pthread_mutex_t pub_mutex, image1_mutex, image2_mutex;
//pthread_t pub_thread;
//pthread_cond_t pub_cond;

// parameters
double exposure = 8.333/4.0; //1.6666; //8.3333;
bool capture_offset = false;
int subsample = 0x0101;
int aoileft = 0;
int aoitop = 0;
int aoiwidth = IMAGE_WIDTH;
int aoiheight = IMAGE_HEIGHT;
int scandirection = ('L'<<8)|'T';
int bitdepth = 8;
double pixelClkFreq = 66;
double framePeriod = 2; //9;

// image statistics
double avg_brightness[2] = {-1, -1};
double min_brightness[2] = {256, 256};
double max_brightness[2] = {0, 0};
int fpn_offsets[2][IMAGE_WIDTH];
bool got_fpn_offsets[2] = {false, false};
std::string data_folder;


/* old camera calibration parameters
double D1[5] =  {-0.23123464918899728, 0.11527943782828139, -0.00063647970340184352, -0.00049014803624485611, 0.0};
double K1[9] =  {497.76983339528766, 0.0, 251.78922851203984, 0.0, 497.92459197955327, 305.24271355497592, 0.0, 0.0, 1.0};
double R1[9] =  {0.99759955642924902, -0.0039288720158517712, -0.069135294725984398, 0.002201813848642353, 0.99968404182391213, -0.025039339815232445, 0.069211827225854949, 0.024827011243599477, 0.99729300733774762};
double P1[12] = {505.68916812179162, 0.0, 278.3145694732666, 0.0, 0.0, 505.68916812179162, 311.85904312133789, 0.0, 0.0, 0.0, 1.0, 0.0};

double D2[5] = {-0.23017783299860542, 0.11382936465939553, -0.00064449809084302209, -0.001269694220087184, 0.0};
double K2[9] = {498.16871807038228, 0.0, 231.48704519999416, 0.0, 498.59899176008793, 319.78224346818968, 0.0, 0.0, 1.0};
double R2[9] = {0.99778512820968579, 0.0035427711563276883, -0.066425045699004398, -0.0018835667314921979, 0.99968506106858424, 0.025024604945314131, 0.066492782314691468, -0.024844062647540504, 0.99747755987351228};
double P2[12] = {505.68916812179162, 0.0, 278.3145694732666, -75.002830310540091, 0.0, 505.68916812179162, 311.85904312133789, 0.0, 0.0, 0.0, 1.0, 0.0};
*/

/* camera calibration parameters (2013-07-18)
double D1[5] =  {-0.221070, 0.086115, -0.000229, -0.000372, 0};
double K1[9] =  {497.450277, 0, 250.681261, 0, 497.442642, 303.624189, 0, 0, 1};
double R1[9] =  {0.999897, -0.002418, 0.014142, 0.002813, 0.999603, -0.028017, -0.014069, 0.028054, 0.999507};
double P1[12] = {493.406922, 0, 232.632809, 0, 0, 493.406922, 311.134407, 0, 0, 0, 1, 0};

double D2[5] = {-0.223296, 0.089609, 0.000218, 0.001179, 0};
double K2[9] = {489.767588, 0, 231.467071, 0, 489.703603, 319.804147, 0, 0, 1};
double R2[9] = {0.999836, 0.002563, 0.017905, -0.003064, 0.999603, 0.028010, -0.017826, -0.028060, 0.999447};
double P2[12] = {493.406922, 0, 232.632809, -70.159185, 0, 493.406922, 311.134407, 0, 0, 0, 1, 0};
*/

/* camera calibration parameters (2013-07-22)
double K1[9] = {503.866933, 0, 252.655004, 0, 504.023332, 301.313010, 0, 0, 1};
double D1[5] = {-0.220828, 0.101266, -0.000351, -0.000330, 0};
double R1[9] = {0.999830, 0.007548, 0.016807, -0.007054, 0.999546, -0.029285, -0.017021, 0.029161, 0.999430};
double P1[12] = {498.014718, 0, 234.392015, 0, 0, 498.014718, 310.881405, 0, 0, 0, 1, 0};

double K2[9] = {496.405757, 0, 231.671077, 0, 496.511321, 319.949472, 0, 0, 1};
double D2[5] = {-0.227128, 0.116539, 0.001343, 0.001937, 0};
double R2[9] = {0.999852, 0.010852, 0.013373, -0.011239, 0.999512, 0.029150, -0.013050, -0.029296, 0.999486};
double P2[12] = {498.014718, 0, 234.392015, -73.336070, 0, 498.014718, 310.881405, 0, 0, 0, 1, 0};
*/

/* camera calibration parameters (2014-01-15)
double K1[9] = {500.469655, 0, 244.691584, 0, 500.177128, 304.123213, 0, 0, 1};
double D1[5] = {-0.231741, 0.117164, -0.000387, -0.001151, 0};
double R1[9] = {0.999995, 0.002956, 0.000830, -0.002931, 0.999601, -0.028098, -0.000913, 0.028096, 0.999605};
double P1[12] = {496.112974, 0, 233.550953, 0, 0, 496.112974, 313.743427, 0, 0, 0, 1, 0};

double K2[9] = {492.777381, 0, 223.876650, 0, 492.713103, 322.178936, 0, 0, 1};
double D2[5] = {-0.233357, 0.124047, 0.001583, -0.000473, 0};
double R2[9] = {0.999976, 0.006876, 0.000372, -0.006884, 0.999582, 0.028096, -0.000179, -0.028098, 0.999605};
double P2[12] = {496.112974, 0, 233.550953, -73.139011, 0, 496.112974, 313.743427, 0, 0, 0, 1, 0};
*/

/* camera calibration parameters (2014-01-20)
double K1[9] = {499.172270, 0.000000, 252.306957, 0.000000, 499.111226, 304.455289, 0.000000, 0.000000, 1.000000};
double D1[5] = {-0.227397, 0.113161, -0.000341, -0.000059, 0.000000};
double R1[9] = {0.999852, 0.006955, 0.015741, -0.006522, 0.999603, -0.027424, -0.015926, 0.027318, 0.999500};
double P1[12] = {492.836047, 0.000000, 234.651081, 0.000000, 0.000000, 492.836047, 313.156033, 0.000000, 0, 0, 1, 0};

double K2[9] = {491.771554, 0.000000, 233.444110, 0.000000, 491.436129, 321.347610, 0.000000, 0.000000, 1.000000};
double D2[5] = {-0.227620, 0.107996, 0.000931, 0.001850, 0.000000};
double R2[9] = {0.999814, 0.010418, 0.016211, -0.010858, 0.999569, 0.027285, -0.015920, -0.027456, 0.999496};
double P2[12] = {492.836047, 0.000000, 234.651081, -72.825609, 0.000000, 492.836047, 313.156033, 0.000000, 0, 0, 1, 0};
*/

/* camera calibration parameters (2014-01-24)
double K1[9] = {496.417008, 0.000000, 249.948522, 0.000000, 496.198695, 303.106660, 0.000000, 0.000000, 1.000000};
double D1[5] = {-0.226643, 0.104528, -0.000482, -0.000246, 0.000000};
double R1[9] = {0.999981, 0.005465, 0.002740, -0.005389, 0.999619, -0.027087, -0.002887, 0.027072, 0.999629};
double P1[12] = {491.758013, 0.000000, 238.857386, 0.000000, 0.000000, 491.758013, 311.226089, 0.000000, 0, 0, 1, 0};

double K2[9] = {489.554548, 0.000000, 229.669462, 0.000000, 489.463526, 319.484221, 0.000000, 0.000000, 1.000000};
double D2[5] = {-0.233657, 0.115875, 0.000605, 0.001163, 0.000000};
double R2[9] = {0.999954, 0.009456, 0.001782, -0.009500, 0.999588, 0.027071, -0.001525, -0.027087, 0.999632};
double P2[12] = {491.758013, 0.000000, 238.857386, -72.432599, 0.000000, 491.758013, 311.226089, 0.000000, 0, 0, 1, 0};
*/

/* camera calibration parameters (2014-01-31)
double K1[9] = {497.331530, 0.000000, 248.817413, 0.000000, 497.771040, 300.801409, 0.000000, 0.000000, 1.000000};
double D1[5] = {-0.228342, 0.110980, -0.000555, -0.000415, 0.000000};
double R1[9] = {0.999932, 0.007599, 0.008901, -0.007342, 0.999566, -0.028524, -0.009114, 0.028457, 0.999553};
double P1[12] = {494.406850, 0.000000, 232.827019, 0.000000, 0.000000, 494.406850, 309.533051, 0.000000, 0, 0, 1, 0};

double K2[9] = {490.019626, 0.000000, 225.961695, 0.000000, 490.321348, 318.300957, 0.000000, 0.000000, 1.000000};
double D2[5] = {-0.230663, 0.114448, 0.000522, -0.000309, 0.000000};
double R2[9] = {0.999916, 0.011100, 0.006731, -0.011288, 0.999531, 0.028453, -0.006412, -0.028527, 0.999572};
double P2[12] = {494.406850, 0.000000, 232.827019, -72.935876, 0.000000, 494.406850, 309.533051, 0.000000, 0, 0, 1, 0};
*/

// camera calibration parameters (2014-02-03)
double K1[9] = {498.914644, 0.000000, 249.548509, 0.000000, 499.316032, 301.380072, 0.000000, 0.000000, 1.000000};
double D1[5] = {-0.226424, 0.105795, -0.001135, -0.000252, 0.000000};
double R1[9] = {0.999731, 0.012321, 0.019647, -0.011749, 0.999512, -0.028958, -0.019994, 0.028720, 0.999388};
double P1[12] = {492.609790, 0.000000, 228.780577, 0.000000, 0.000000, 492.609790, 310.496410, 0.000000, 0, 0, 1, 0};

double K2[9] = {491.127361, 0.000000, 227.359592, 0.000000, 491.320019, 320.578598, 0.000000, 0.000000, 1.000000};
double D2[5] = {-0.229384, 0.107279, 0.000333, 0.000893, 0.000000};
double R2[9] = {0.999752, 0.015213, 0.016238, -0.015675, 0.999465, 0.028716, -0.015792, -0.028963, 0.999456};
double P2[12] = {492.609790, 0.000000, 228.780577, -72.329873, 0.000000, 492.609790, 310.496410, 0.000000, 0, 0, 1, 0};





void save_fpn_offsets();

void sigintfunc(int sig)
{
  printf("Closing PIXCI cameras...");
  fflush(0);
  pxd_PIXCIclose();
  printf("Done\n");
  
  if (capture_offset) {
    printf("Saving fpn offsets...");
    fflush(0);
    save_fpn_offsets();
    printf("Done\n");
  }

  signal(SIGINT, SIG_DFL);
  kill(getpid(), SIGINT);
  //exit(1);
}

void copy_image(int cam, int cambuf, int x0, int y0, int w, int h)
{
  uchar *image;
  if (cam == 1)
    image = image1_buf[cambuf-1].data;
  else
    image = image2_buf[cambuf-1].data;

  int retval = pxd_readuchar(cam, cambuf, x0, y0, x0+w, y0+h, image, w*h, (char *)"Grey");
  if (retval < 0)
    printf("pxd_readuchar: %s\n", pxd_mesgErrorCode(retval));
  else if (retval < w*h)
    printf("pxd_readuchar error: %d != %d\n", retval, w*h);
}

void copy_full_image(int cam, int cambuf)
{
  copy_image(cam, cambuf, 0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
}

void publish_images()
{
  // set timestamps
  ros::Time now = MAX(image1_msg.header.stamp, image2_msg.header.stamp);  //ros::Time::now();
  image1_msg.header.stamp = now;
  image2_msg.header.stamp = now;
  cinfo1.header.stamp = now;
  cinfo2.header.stamp = now;

  // publish Image and CameraInfo messages
  image1_pub.publish(image1_msg);
  image2_pub.publish(image2_msg);
  cinfo1_pub.publish(cinfo1);
  cinfo2_pub.publish(cinfo2);
}

void publish_image(int cam)
{
  if (cam == 1) {
    // publish Image and CameraInfo messages
    image1_pub.publish(image1_msg);
    cinfo1_pub.publish(cinfo1);
  }
  else {
    // publish Image and CameraInfo messages
    image2_pub.publish(image2_msg);
    cinfo2_pub.publish(cinfo2);
  }
}

void save_fpn_offsets()
{
  char fname[512];
  sprintf(fname, "%s/fpn_offsets.txt", data_folder.c_str());
  FILE *f = fopen(fname, "w");
  for (int i = 0; i < IMAGE_WIDTH; i++)
    fprintf(f, "%d ", fpn_offsets[0][i]);
  fprintf(f, "\n");
  for (int i = 0; i < IMAGE_WIDTH; i++)
    fprintf(f, "%d ", fpn_offsets[1][i]);
  fprintf(f, "\n");
  fclose(f);
}

void load_fpn_offsets()
{
  char fname[512];
  sprintf(fname, "%s/fpn_offsets.txt", data_folder.c_str());
  FILE *f = fopen(fname, "r");
  if (f != NULL) {
    for (int i = 0; i < IMAGE_WIDTH; i++)
      fscanf(f, "%d ", &fpn_offsets[0][i]);
    for (int i = 0; i < IMAGE_WIDTH; i++)
      fscanf(f, "%d ", &fpn_offsets[1][i]);
    fclose(f);
  }
  else
    for (int i = 0; i < IMAGE_WIDTH; i++)
      fpn_offsets[0][i] = fpn_offsets[1][i] = 0;
}

void process_image(int cam)
{
  int w = IMAGE_WIDTH;
  int h = IMAGE_HEIGHT;

  uchar image[w*h];
  ros::Time stamp;
  if (cam == 1) {
    //image = image1_buf[image1_last_buf-1].data;
    memcpy(image, image1_buf[image1_last_buf-1].data, w*h);
    stamp = image1_buf[image1_last_buf-1].stamp;
  }
  else {
    //image = image2_buf[image2_last_buf-1].data;
    memcpy(image, image2_buf[image2_last_buf-1].data, w*h);
    stamp = image2_buf[image2_last_buf-1].stamp;
  }

  if (capture_offset) {
    // compute brightness of image center
    double brightness = 0.0;
    int n = 0;
    for (int i = h/4; i < 3*h/4; i++) {
      for (int j = w/4; j < 3*w/4; j++) {
	n++;      
	brightness += image[i*w+j];
      }
    }
    brightness /= (double)n;
    //printf("(b=%.0f, bmin=%.0f, bmax=%.0f\n", brightness, min_brightness[cam-1], max_brightness[cam-1]);  //dbug

    // update average brightness
    if (avg_brightness[cam-1] < 0)
      avg_brightness[cam-1] = brightness;
    else
      avg_brightness[cam-1] = .99*avg_brightness[cam-1] + .01*brightness;

    // update max brightness
    if (brightness > max_brightness[cam-1])
      max_brightness[cam-1] = brightness;
  
    // update min brightness
    if (brightness < min_brightness[cam-1]) {
      min_brightness[cam-1] = brightness;

      if (brightness < .3*max_brightness[cam-1]) {
	// capture fixed pattern noise offsets
	printf("Capturing fixed pattern noise offsets...\n");
	got_fpn_offsets[cam-1] = true;
	for (int i = 0; i < w; i++)
	  fpn_offsets[cam-1][i] = 0;
	for (int i = 0; i < w*h; i++)
	  fpn_offsets[cam-1][i%w] += image[i];
	for (int i = 0; i < w; i++)
	  fpn_offsets[cam-1][i] = fpn_offsets[cam-1][i] / h;
      }
    }
  }

  // subtract fixed pattern noise
  //for (int i = 0; i < w*h; i++)
  //  image[i] = MAX(image[i] - fpn_offsets[cam-1][i%w], 0);
  __m128i fpn_offsets_sse[w/16];
  for (int x = 0; x < w; x+=16)
    fpn_offsets_sse[x/16] = _mm_set_epi8(fpn_offsets[cam-1][x+15], fpn_offsets[cam-1][x+14],
					 fpn_offsets[cam-1][x+13], fpn_offsets[cam-1][x+12],
					 fpn_offsets[cam-1][x+11], fpn_offsets[cam-1][x+10],
					 fpn_offsets[cam-1][x+9], fpn_offsets[cam-1][x+8],
					 fpn_offsets[cam-1][x+7], fpn_offsets[cam-1][x+6],
					 fpn_offsets[cam-1][x+5], fpn_offsets[cam-1][x+4],
					 fpn_offsets[cam-1][x+3], fpn_offsets[cam-1][x+2],
					 fpn_offsets[cam-1][x+1], fpn_offsets[cam-1][x]);

  uchar *I_ptr = image;
  for (int y = 0; y < IMAGE_HEIGHT; y++) {
    for (int x = 0; x < IMAGE_WIDTH; x+=16, I_ptr+=16) {
      __m128i a = _mm_load_si128((__m128i *)I_ptr);
      __m128i b = fpn_offsets_sse[x/16];
      __m128i c = _mm_subs_epu8(a, b);  // c = MAX(a-b, 0)
      _mm_store_si128((__m128i *)(I_ptr), c);
    }
  }

  // adjust brightness to match average (for flickering)
  /*
  if (got_fpn_offsets[cam-1]) {
    double a = //avg_brightness[cam-1] / brightness;
    for (int i = 0; i < w*h; i++)
    image[i] = MIN(image[i] * a, 255);
  }
  */

  // copy image data into message
  if (cam == 1) {
    //std::copy(image, image+w*h, image1_msg.data.begin());
    int cnt = 0;
    for (int y = 0; y < w; y++)
      for (int x = 0; x < h; x++, cnt++)
        image1_msg.data[cnt] = image[x*w+(w-y-1)];  // rotate 90 degrees clockwise

    /*
    uchar image_rot[w*h];
    uchar *image_rot_ptr = image_rot;
    for (int y = 0; y < w; y++) {
      for (int x = 0; x < h; x+=16, image_rot_ptr+=16) {
	uchar *I_ptr = &image[x*w+(w-y-1)];
	__m128i a = _mm_set_epi8(I_ptr[15*w], I_ptr[14*w], I_ptr[13*w], I_ptr[12*w], I_ptr[11*w], I_ptr[10*w],
				 I_ptr[9*w], I_ptr[8*w], I_ptr[7*w], I_ptr[6*w], I_ptr[5*w], I_ptr[4*w],
				 I_ptr[3*w], I_ptr[2*w], I_ptr[w], I_ptr[0]);
	_mm_store_si128((__m128i *)(image_rot_ptr), a);
      }
    }
    std::copy(image_rot, image_rot+w*h, image1_msg.data.begin());
    */

    // set timestamps
    image1_msg.header.stamp = stamp;
    cinfo1.header.stamp = stamp;
  }
  else {
    // copy image data into message
    //std::copy(image, image+w*h, image2_msg.data.begin());
    int cnt = 0;
    for (int y = 0; y < w; y++)
      for (int x = 0; x < h; x++, cnt++)
        image2_msg.data[cnt] = image[(h-x-1)*w+y];  // rotate 90 degrees clockwise

    /*
    uchar image_rot[w*h];
    uchar *image_rot_ptr = image_rot;
    for (int y = 0; y < w; y++) {
      for (int x = 0; x < h; x+=16, image_rot_ptr+=16) {
	uchar *I_ptr = &image[(h-x-1)*w+y];
	__m128i a = _mm_set_epi8(I_ptr[0], I_ptr[w], I_ptr[2*w], I_ptr[3*w], I_ptr[4*w], I_ptr[5*w],
				 I_ptr[6*w], I_ptr[7*w], I_ptr[8*w], I_ptr[9*w], I_ptr[10*w], I_ptr[11*w],
				 I_ptr[12*w], I_ptr[13*w], I_ptr[14*w], I_ptr[15*w]);
	_mm_store_si128((__m128i *)(image_rot_ptr), a);
      }
    }
    std::copy(image_rot, image_rot+w*h, image2_msg.data.begin());
    */

    // set timestamps
    image2_msg.header.stamp = stamp;
    cinfo2.header.stamp = stamp;
  }
}

void camera1_callback(int sig)
{
  //printf("L"); fflush(0);

  int cambuf = pxd_capturedBuffer(1);
  image1_buf[cambuf-1].stamp = ros::Time::now();
  copy_full_image(1, cambuf);  // transfer the image into memory
  image1_last_buf = cambuf;

  if (live_mode) {

    //if (publishing)
    //  return;

    got1 = true;

    /*
    if (got1 && got2) {
      publishing = true;
      got1 = got2 = false;
      //process_image(1);
      //process_image(2);
      publish_images();
      publishing = false;
    }
    */
  }
}

void camera2_callback(int sig)
{
  //printf("R"); fflush(0);

  int cambuf = pxd_capturedBuffer(2);
  image2_buf[cambuf-1].stamp = ros::Time::now();
  copy_full_image(2, cambuf);  // transfer the image into memory
  image2_last_buf = cambuf;

  if (live_mode) {

    //if (publishing)
    //  return;

    got2 = true;

    /*
    if (got1 && got2) {
      publishing = true;
      got1 = got2 = false;
      process_image(1);
      process_image(2);
      publish_images();
      publishing = false;
    }
    */
  }
}

void set_params()
{
  pxd_SILICONVIDEO_setResolutionAndTiming(UNITSMAP, 0, subsample, aoileft, aoitop, aoiwidth,
					  aoiheight, scandirection, bitdepth, 0, 0, pixelClkFreq,
					  framePeriod, 0, 0, 0);
}

/*
 * Open the XCLIB C Library for use.
 */
int do_open(void)
{
  printf("Opening EPIX(R) PIXCI(R) Imaging Board...");
  
  int i = pxd_PIXCIopen(DRIVERPARMS, FORMAT, FORMATFILE);
  if (i >= 0) {
    // For the sake of demonstrating optional interrupt hooks.
    if (live_mode) {
      signal(SIGUSR1, camera1_callback);
      pxd_eventCapturedFieldCreate(1, SIGUSR1, NULL);
      signal(SIGUSR2, camera2_callback);
      pxd_eventCapturedFieldCreate(2, SIGUSR2, NULL);
    }
    printf("Open OK");
  } else {
    printf("Open Error %d\a\a\n", i);
    pxd_mesgFault(UNITSMAP);
  }

  pxd_SILICONVIDEO_setResolutionAndTiming(UNITSMAP, 0, subsample, aoileft, aoitop, aoiwidth,
					  aoiheight, scandirection, bitdepth, 0, 0, pixelClkFreq,
					  framePeriod, 0, 0, 0);
  pxd_SILICONVIDEO_setExposure(UNITSMAP, 0, exposure);
  
  return i;
}

/*
 * Report image frame buffer memory size
 */
void do_imsize(void)
{
  printf("Image frame buffer memory size: %.3f Kbytes\n", (double)pxd_infoMemsize(UNITSMAP)/1024);
  printf("Image frame buffers           : %d\n", pxd_imageZdim());
  printf("Number of boards              : %d\n", pxd_infoUnits());
}

/*
 * Report image resolution.
 */
void do_vidsize(void)
{
  printf("Image resolution:\n");
  printf("xdim           = %d\n", pxd_imageXdim());
  printf("ydim           = %d\n", pxd_imageYdim());
  printf("colors         = %d\n", pxd_imageCdim());
  printf("bits per pixel = %d\n", pxd_imageCdim()*pxd_imageBdim());
  printf("exposures = %f, %f\n", pxd_SILICONVIDEO_getExposure(1), pxd_SILICONVIDEO_getExposure(2));
  printf("frame periods = %f, %f\n", pxd_SILICONVIDEO_getFramePeriod(1), pxd_SILICONVIDEO_getFramePeriod(2));
}


/*
void bw_display2(void)
{
  int x, y;
  //
  // Display data from the PC buffer.
  //
  for (y = 0; y < IMAGE_HEIGHT; y++) {
    printf("I = ");
    for (x = 0; x < IMAGE_WIDTH; x++)
      printf("%4d ", monoimage_buf8[y*IMAGE_WIDTH+x]);
    printf("\n");
  }
}
*/


int init_cameras()
{
  // Catch signals.
  signal(SIGINT, sigintfunc);
  signal(SIGFPE, sigintfunc);
  
  // Open and set video format.
  if (do_open() < 0)
    return -1;
  
  do_imsize();
  do_vidsize();

  return 0;
}


/*
void set_exposure(double e)
{
  exposure = e;
}

void do_capture_offset()
{
  capture_offset = false;
  system("rosrun dynamic_reconfigure dynparam set_from_parameters svstereo");
}

void param_callback(siliconvideo::svstereoConfig &config, uint32_t level)
{
  if (exposure != config.exposure)
    set_exposure(config.exposure);
  if (config.capture_offset)
    do_capture_offset();
}
*/

void init_params(ros::NodeHandle &nh)
{
  //dynamic_reconfigure::Server<siliconvideo::svstereoConfig> srv;
  //dynamic_reconfigure::Server<siliconvideo::svstereoConfig>::CallbackType f;
  //f = boost::bind(&param_callback, _1, _2);
  //srv.setCallback(f);

  nh.param("/svstereo/exposure", exposure, 8.333/4.0);
  nh.param("/svstereo/capture_offset", capture_offset, false);
  if (nh.getParam("/svstereo/data_folder", data_folder))
    ROS_INFO("Got param: %s", data_folder.c_str());
  else
    ROS_ERROR("Failed to get param 'data_folder'");

  printf("exposure = %f\n", exposure);
  printf("capture_offset = %d\n", capture_offset);

  load_fpn_offsets();
}

void init_messages(ros::NodeHandle &nh)
{
  image1_pub = nh.advertise<sensor_msgs::Image>("/svstereo/left/image_raw", 10);
  image2_pub = nh.advertise<sensor_msgs::Image>("/svstereo/right/image_raw", 10);
  cinfo1_pub = nh.advertise<sensor_msgs::CameraInfo>("/svstereo/left/camera_info", 10);
  cinfo2_pub = nh.advertise<sensor_msgs::CameraInfo>("/svstereo/right/camera_info", 10);

  // images are rotated!
  int w = IMAGE_HEIGHT;
  int h = IMAGE_WIDTH;

  // left image (rotate 90 degrees)
  image1_msg.header.frame_id = "svstereo_left";
  image1_msg.width = w;
  image1_msg.height = h;
  image1_msg.encoding = sensor_msgs::image_encodings::MONO8;
  image1_msg.is_bigendian = 0;
  image1_msg.step = w;
  image1_msg.data.resize(w*h);

  // right image (rotate -90 degrees)
  image2_msg.header.frame_id = "svstereo_right";
  image2_msg.width = w;
  image2_msg.height = h;
  image2_msg.encoding = sensor_msgs::image_encodings::MONO8;
  image2_msg.is_bigendian = 0;
  image2_msg.step = w;
  image2_msg.data.resize(IMAGE_WIDTH*IMAGE_HEIGHT);

  // left camera info
  cinfo1.header.frame_id = "svstereo_left";
  cinfo1.width = w;
  cinfo1.height = h;
  cinfo1.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  cinfo1.D.resize(sizeof(D1) / sizeof(double));
  for (int i = 0; i < cinfo1.D.size(); i++)
    cinfo1.D[i] = D1[i];
  for (int i = 0; i < cinfo1.K.size(); i++)
    cinfo1.K[i] = K1[i];
  for (int i = 0; i < cinfo1.R.size(); i++)
    cinfo1.R[i] = R1[i];
  for (int i = 0; i < cinfo1.P.size(); i++)
    cinfo1.P[i] = P1[i];

  // right camera info
  cinfo2.header.frame_id = "svstereo_left";
  cinfo2.width = w;
  cinfo2.height = h;
  cinfo2.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  cinfo2.D.resize(sizeof(D2) / sizeof(double));
  for (int i = 0; i < cinfo2.D.size(); i++)
    cinfo2.D[i] = D2[i];
  for (int i = 0; i < cinfo2.K.size(); i++)
    cinfo2.K[i] = K2[i];
  for (int i = 0; i < cinfo2.R.size(); i++)
    cinfo2.R[i] = R2[i];
  for (int i = 0; i < cinfo2.P.size(); i++)
    cinfo2.P[i] = P2[i];
}

int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "svstereo");
  ros::NodeHandle nh;

  init_params(nh);
  init_messages(nh);

  // init cameras
  if (init_cameras() >= 0) {

    if (live_mode) {  // live mode
      //pxd_goLivePair(UNITSMAP, 1, 2);
      pxd_goLiveSeq(UNITSMAP, 1, 3, 1, 0, 1);

      ros::Rate r(5000);
      while (1) {
 	if (got1 && got2) {
	  got1 = got2 = false;
	  process_image(1);
	  process_image(2);
	  publish_images();
	}
	ros::spinOnce();
	r.sleep();
      }

      //ros::spin();
    }
    else {  // snap mode
      ros::Rate r(5000);
      int b1 = 0;
      int b2 = 0;
      int err = pxd_doSnap(UNITSMAP, 1, 0);
      for (int cnt = 1; true; cnt++) {
	if (pxd_capturedBuffer(1) == b1+1) {
	  got1 = true;
	  camera1_callback(0);
	  b1 = !b1;
	  err = pxd_goSnap(1, b1+1);
	}
	if (pxd_capturedBuffer(2) == b2+1) {
	  got2 = true;
	  camera2_callback(0);
	  b2 = !b2;
	  err = pxd_goSnap(2, b2+1);
	}
	if (got1 && got2) {
	  got1 = got2 = false;
	  process_image(1);
	  process_image(2);
	  publish_images();
	}
	ros::spinOnce();
	r.sleep();
      }
    }
  }

  // close cameras
  pxd_PIXCIclose();  
}
