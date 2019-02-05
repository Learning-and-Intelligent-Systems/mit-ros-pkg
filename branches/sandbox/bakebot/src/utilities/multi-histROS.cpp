/*
 *  multi_hist_cam.cpp
 *  Object Tracking
 *
 *  Created by Wil Selby on 1/4/10.
 *  Copyright 2010 Naval Acadmy. All rights reserved.
 *
 */

/* TODO:
 *
 *
 *
 *
 */

/* Includes */

#include <inttypes.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
//#include "stdafx.h" 
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <sys/time.h>


//ROS
#include <ros/ros.h>
//#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

//TODO Tyler: Added these include statements in order to use a Float64MultiArray message.
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"

//namespace enc = sensor_msgs::image_encodings;

#define PI 3.14
/* Defines*/
#define RECORD 0
#define RCV_BUF 14000
#define HIST_SCALE 20			/* Size of histogram display*/
//#define PI 3.141592653589793 //defined in carmen3d_opencv_utils.hpp
#define POLY1_HULL0  1			/* Contour Finding Variables */
#define CVCONTOUR_APPROX_LEVEL  2 	  /* Simplicity of the foreground object */
#define BEACH_HUE 30			/*Hue value used to identify beach for logic */
#define THRESH_BIN 1			/* Display thresholds as binary images */
#define WEIGHT_CONTROL 0		/* Weight the control vector by the object area*/
#define THETA 0.0			/* Angle difference between camera and robot */
#define rescale_min 0.0 
#define rescale_max 3.33				/* rescaling for pixels to mm */
#define k_prop 1.0
#define k_der 0.1
#define k_int 0.1

//for blue clipboard in holodeck all lights on, tarp background
#define HUE_MAX 70
#define HUE_MIN 22
#define SAT_MAX 113
#define SAT_MIN 8

#define USLEEP_SEC 1000000.0
#define ROBOT_CNTL_FREQ 8.0   //practically like 8.0 for 5Hz

/* Globals */
CvCapture* capture;

//Initialize Images
IplImage* still;
IplImage* frame;
IplImage* frame_final;
IplImage* h_histimg_thresh;
IplImage* s_histimg_thresh;
IplImage* hsv_still;
IplImage* h_plane_still;
IplImage* s_plane_still;
IplImage* h_plane_orig;
IplImage* s_plane_orig;
IplImage* v_plane_orig;
IplImage* v_plane_still;
IplImage* planes[2];
IplImage* frame_copy2;
IplImage* frame_copy;
IplImage* back_project;
IplImage* contour_img;
IplImage* hsv_frame;
IplImage* h_plane_frame;
IplImage* s_plane_frame;
IplImage* v_plane_frame;
IplImage* r_plane_frame;
IplImage* b_plane_frame;
IplImage* g_plane_frame;
IplImage* planes_frame[2];
IplImage* gamma_corrected;
IplImage* back_projection_final;
IplImage* contours_final;
IplImage* OUTPUT;
IplImage* Resize;
IplImage* Resize2;
IplImage* rcvdFrame;

/* Histogram Variables */
int h_bins_w;
int s_bins_w;
float max_val;
int hist_size[2];
CvHistogram* h_hist;
CvHistogram* s_hist;
CvHistogram* model_hist;
CvHistogram* model_hist_copy;
CvHistogram* model_hist_thresh;
CvHistogram* hist_frame;
CvHistogram* ratio;
IplImage* hist_img; /* 2d Histogram */
int flag = 1;
float bin_count = 0;


/* Histogram Variables */ 
int h_bins = 30;	//30
int s_bins = 40;	//32
float h_ranges_arr[] = { 0, 180 };   //hue is [0,180] in opencv
float* h_ranges = h_ranges_arr;
float s_ranges_arr[] = { 0, 255 };
float* ranges[] = { h_ranges_arr, s_ranges_arr };
int tot_hist_num = 1;   //initially atleast 1 histogram

/* Centroid Location variables */
int num_obj;
//int CVCLOSE_ITR = 1;   // iterations of the open/close morphological operators
int pix_total;

int PERIMSCALE = 6;
int max_width;
int max_height;
int dim_flag = 1;
int frame_count = 0;

/* Control Calculation Variables */
CvPoint* obj_loc;
CvPoint* weighted_obj_loc;
CvPoint2D64f ctrl_vec;
CvPoint2D64f ctrl_vec_out;
CvPoint2D64f final_ctrl_vec;
CvPoint2D64f image_plane_vec;
CvPoint2D64f ctrl_vec_old;
CvPoint2D64f ctrl_vec_I;
CvPoint2D64f ctrl_vec_P;
CvPoint2D64f ctrl_vec_D;
CvPoint2D64f avg_loc;
CvPoint2D64f center;
double time_old;
int total_obj_found;
int area_total[10]; 	//make sure this matches NUM_OBJ
int pix_final;
double target_area = 0;
FILE *f; //used to log target position in camera

//Colors
const CvScalar CVX_WHITE = CV_RGB(0xff,0xff,0xff);
const CvScalar CVX_BLACK = CV_RGB(0x00,0x00,0x00);
const CvScalar CVX_RED   = CV_RGB(0xff,0x00,0x00);
const CvScalar CVX_GREEN = CV_RGB(0x00,0xff,0x00);

//Fetaure Variables
int bound_box = 1;
int hue_on = 1;
int sat_on = 1;
int thresholding_on = 1;
int model_hist_on = 0;
int b_projection = 0;
int contour_on = 0;
int new_hist = 0;

struct Data{
    /* Specific to each Histogram*/
    int hmax;   
    int hmin;	
    int smax;
    int smin;
    CvHistogram* model_hist_thresh;
};

Data global_data;

/* vector variable */
int vector_place = 0;

//Create vector to store each histogram's data
std::vector<Data> histograms;


//						seals_short | whales_long | Boats1 |Barge | max
int temp_hmax;//		180			112			180		 59		180    
int temp_hmin;//		88			100			116		 0		0	
int temp_smax; //		66			127			256		 87		255
int temp_smin;//		0			0			0		 0		0

//Click Point
int add_remove_pt = 0;
CvPoint click_pt;

//Time variables
struct timeval tim;
double time_origin = 1200000000;
double frame_time;
double contour_time;
double delta_time = 0;


static bool inFunction = false;


//IMU variables
double imu_rpy[3];
double last_imu_rpy[3];
double ekf_send_rpy[3];
double imu_utime = 0;
double last_imu_utime = 0;
double last_error_x = 0;
double last_error_y = 0;
double mag_scale = 0;
double height = 0;
double last_height = 0;

double vicon_height = 0;
int vicon_input = 0;

int debug = 0;

//POSIX Variables
pthread_mutex_t image_lock;
int img_rcvd = 0;


class ImageConverter {

    public:

        ImageConverter(ros::NodeHandle &n) :
            n_(n), it_(n_)
    {
        //image_pub_ = it_.advertise("image_topic_2",1);

        image_sub_ = it_.subscribe(
                "/wide_stereo/left/image_rect_color", 1, &ImageConverter::imageCallback, this);
        printf("done with constructor\n");
    }

        void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
        {
            printf("image callback\n");
            if(!inFunction){
                inFunction = true;
            }
            else {
                printf("Was already in function..returning from rcv_img \n");
                return;
            }

            pthread_mutex_lock(&image_lock);

            try
            {
                rcvdFrame2 = bridge_.imgMsgToCv(msg_ptr, "bgr8");
            }
            catch (sensor_msgs::CvBridgeException error)
            {
                ROS_ERROR("cv_bridge exception");
                return;
            }

            //Fill in openCV IplImage
            pthread_mutex_unlock(&image_lock);


            cvNamedWindow("Live Stream", CV_WINDOW_AUTOSIZE );
            cvShowImage("Live Stream", rcvdFrame2 );
            cvWaitKey(3);

            //cv::imshow(WINDOW, cv_ptr->image);

            inFunction = false;
        }

        IplImage* getImage() {
            return rcvdFrame2;
        }

    protected:

        ros::NodeHandle n_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        sensor_msgs::CvBridge bridge_;
        IplImage* rcvdFrame2;
        image_transport::Publisher image_pub_;

};


//Functions

void initialize(){
    //Initialize Histogram Images
    frame = cvCreateImage(cvSize(still->width, still->height), still->depth, still->nChannels);
    frame_final = cvCreateImage(cvSize(still->width, still->height), still->depth, still->nChannels);

    h_histimg_thresh = cvCreateImage( cvSize(320,240), IPL_DEPTH_8U, 3 );
    s_histimg_thresh = cvCreateImage( cvSize(320,240), IPL_DEPTH_8U, 3 );
    hsv_still = cvCreateImage( cvGetSize(still),  still->depth, still->nChannels );
    h_plane_still = cvCreateImage(cvGetSize(still), still->depth, 1);
    s_plane_still = cvCreateImage(cvGetSize(still), still->depth, 1);
    v_plane_still = cvCreateImage(cvGetSize(still), still->depth, 1);
    h_plane_orig = cvCreateImage(cvGetSize(still), still->depth, 1);
    s_plane_orig = cvCreateImage(cvGetSize(still), still->depth, 1);
    v_plane_orig = cvCreateImage(cvGetSize(still), still->depth, 1);

    planes[0] = h_plane_orig;
    planes[1] = s_plane_orig;

    frame_copy2 = cvCreateImage( cvGetSize(still), still->depth, 1 );
    frame_copy = cvCreateImage( cvGetSize(still), still->depth, 1 );

    back_project = cvCreateImage( cvGetSize(still), still->depth, 1 );
    contour_img = cvCreateImage( cvGetSize(still), still->depth, 1 );
    hsv_frame = cvCreateImage( cvGetSize(still), still->depth, still->nChannels );
    h_plane_frame = cvCreateImage(cvGetSize(still), still->depth, 1);
    s_plane_frame = cvCreateImage(cvGetSize(still), still->depth, 1);
    v_plane_frame = cvCreateImage(cvGetSize(still), still->depth, 1);
    r_plane_frame = cvCreateImage(cvGetSize(still), still->depth, 1);
    g_plane_frame = cvCreateImage(cvGetSize(still), still->depth, 1);
    b_plane_frame = cvCreateImage(cvGetSize(still), still->depth, 1);
    planes_frame[0] = h_plane_frame;
    planes_frame[1] = s_plane_frame;

    gamma_corrected = cvCreateImage( cvGetSize(still), IPL_DEPTH_32F, 3 );
    back_projection_final = cvCreateImage(cvGetSize(still), still->depth, 1);
    contours_final = cvCreateImage(cvGetSize(still), still->depth, 1);

    OUTPUT = cvCreateImage( cvSize(320,240), still->depth, 1);

    /* Histogram Variables */
    h_bins_w = 180/h_bins;
    s_bins_w = 256/s_bins;
    max_val = 0;
    hist_size[0] = h_bins;
    hist_size[1] = s_bins;
    h_hist = cvCreateHist(1, &h_bins, CV_HIST_ARRAY, &h_ranges, 1);
    s_hist = cvCreateHist(1, &s_bins, CV_HIST_ARRAY, &h_ranges, 1);
    model_hist = cvCreateHist(2, hist_size, CV_HIST_ARRAY, ranges, 1);	//2D combines H and S after thresholding
    hist_img = cvCreateImage( cvSize(h_bins*HIST_SCALE, s_bins*HIST_SCALE), 8, 3); /*2-D Hist Image */
    model_hist_copy = cvCreateHist(2, hist_size, CV_HIST_ARRAY, ranges, 1);	//2D combines H and S after thresholding 

    max_height = 0;
    max_width = 0;

    ctrl_vec_old = cvPoint2D64f(0.0,0.0);
    time_old = 0.0;
    ctrl_vec_I = cvPoint2D64f(0.0,0.0);

    last_imu_rpy[0] = 0;
    last_imu_rpy[1] = 0;
    last_imu_rpy[2] = 0;

    // TODO: set these values
    // RED
    global_data.hmax = 13;	//HUE_MAX;	    
    global_data.hmin = 7;	//HUE_MIN;	
    global_data.smax = 184;	//SAT_MAX;	 
    global_data.smin = 141;	//SAT_MIN;
    global_data.model_hist_thresh = cvCreateHist(2, hist_size, CV_HIST_ARRAY, ranges, 1);

    histograms.push_back(global_data);

    // YELLOW
    global_data.hmax = 23;	//HUE_MAX;	    
    global_data.hmin = 13;	//HUE_MIN;	
    global_data.smax = 140;	//SAT_MAX;	 
    global_data.smin = 131;	//SAT_MIN;
    global_data.model_hist_thresh = cvCreateHist(2, hist_size, CV_HIST_ARRAY, ranges, 1);

    histograms.push_back(global_data);

    // GREEN
    global_data.hmax = 38;	//HUE_MAX;	    
    global_data.hmin = 24;	//HUE_MIN;	
    global_data.smax = 135;	//SAT_MAX;	 
    global_data.smin = 97;	//SAT_MIN;
    global_data.model_hist_thresh = cvCreateHist(2, hist_size, CV_HIST_ARRAY, ranges, 1);

    histograms.push_back(global_data);
}

void keystroke(){

    int end = cvWaitKey(10);

    if( (char) end == 27){
        exit(0);
    }
    switch((char) end ){
        case 'h': 
            hue_on ^=1;
            break;
        case 's':
            sat_on ^=1;
            break;
        case 't':
            thresholding_on^=1;
            break;
        case 'm':
            model_hist_on^=1;
            break;
        case 'b':
            bound_box ^=1;
            break;
        case 'p':
            b_projection ^=1;
            break;
        case 'f':
            printf("Saving current frame\n");
            char filename [256];
            sprintf(filename, "frame-%d.jpg",(int)frame_time);
            cvSaveImage(filename,frame);
            break;
        case 'n':
            new_hist = -1;
            thresholding_on = 1;
            //printf("hit n\n");
            break;
        case 'c':
            if(vector_place < histograms.size()-1 ){
                vector_place +=1;
            }
            else{
                vector_place = 0;
            }
            printf("Histogram #: %d\n", vector_place+1);
            printf("%d, %d, %d, %d\n", histograms[vector_place].hmax, histograms[vector_place].hmin, histograms[vector_place].smax, histograms[vector_place].smin);
        default:
            ;
    }
}

void threshold(IplImage* frame, int index, Data d){	

    if(index == -1){
        d.model_hist_thresh = cvCreateHist(2, hist_size, CV_HIST_ARRAY, ranges, 1);
        cvClearHist(d.model_hist_thresh);
    }
    else{
        d = histograms[index];
    }

    //View Pure HSV image
    cvCvtColor(frame, hsv_still, CV_BGR2HSV);

    //printf("frame size %d %d \n\n", frame->width, frame->height);

    //Seperate frame into HSV planes
    cvCvtPixToPlane( hsv_still, h_plane_still, s_plane_still, v_plane_still, 0);  //hsv_still

    cvCopy(h_plane_still, h_plane_orig, 0);
    cvCopy(s_plane_still, s_plane_orig, 0);

    //Hue thresholding images
    if(hue_on){
        cvShowImage( "Hue Plane", frame_copy );
        d.hmax = temp_hmax;
        d.hmin = temp_hmin;
    }

    //Sat thresholding images
    if(sat_on){
        cvShowImage( "Sat Plane", frame_copy2 );
        d.smax = temp_smax;
        d.smin = temp_smin;
    }

    //Calc histograms
    cvCalcHist( &h_plane_orig, h_hist, 0, 0);
    cvGetMinMaxHistValue(h_hist, 0, &max_val, 0, 0);
    cvConvertScale( h_hist->bins, h_hist->bins, max_val ? 255. / max_val : 0., 0 );

    cvCalcHist(&s_plane_orig, s_hist, 0, 0);
    cvGetMinMaxHistValue(s_hist, 0, &max_val, 0, 0);
    cvConvertScale( s_hist->bins, s_hist->bins, max_val ? 255. / max_val : 0., 0 );

    //Calculate model histogram
    cvCalcHist( planes, model_hist, 0, 0);
    cvGetMinMaxHistValue(model_hist, 0, &max_val, 0, 0);
    cvConvertScale( model_hist->bins, model_hist->bins, max_val ? 255. / max_val : 0., 0 );
    cvCopyHist(model_hist, &d.model_hist_thresh);

    //Threshold h and s plane based on trackbars
    cvThreshold(h_plane_still, h_plane_still, d.hmax, d.hmax, CV_THRESH_TOZERO_INV); 
    cvThreshold(h_plane_still, h_plane_still, d.hmin, d.hmin, CV_THRESH_TOZERO);

    cvThreshold(s_plane_still, s_plane_still, d.smax, d.smax, CV_THRESH_TOZERO_INV);
    cvThreshold(s_plane_still, s_plane_still, d.smin, d.smin, CV_THRESH_TOZERO);

    //Threshold model histogram
    for( int h = 0; h<h_bins; h++){
        for( int s = 0; s<s_bins; s++){
            if(h < d.hmin/h_bins_w || h > d.hmax/h_bins_w ||  s < d.smin/s_bins_w || s > d.smax/s_bins_w){
                cvSetReal2D(d.model_hist_thresh->bins, h, s, 0.0);
            }
        }
    }		

    // Normalize Histogram
    cvNormalizeHist(d.model_hist_thresh, HIST_SCALE*255);

    // Calculate Backprojection 
    cvCalcBackProject( planes , back_project, d.model_hist_thresh);

    // Morphological operations on backprojecetion to get rid of noise 
    cvMorphologyEx(back_project, back_project, 0, 0, CV_MOP_OPEN, 1);
    cvMorphologyEx(back_project, back_project, 0, 0, CV_MOP_CLOSE, 2);

    // Show backprojection 
    cvNamedWindow("Back Projection", CV_WINDOW_AUTOSIZE);
    cvShowImage("Back Projection", back_project);
    cvNormalizeHist(d.model_hist_thresh, 1.0);

    //use h and s min/max values to isolate desired bins in histogram, reset bins outside of threshold vlaues to 0
    for(int i = 0; i < d.hmin/h_bins_w; i++){
        cvSetReal1D(h_hist->bins, i, 0.0);
    }
    for(int i = (d.hmax/h_bins_w) + 1; i < h_bins; i++){
        cvSetReal1D(h_hist->bins, i, 0.0);
    }

    for(int i = 0; i < d.smin/s_bins_w; i++){
        cvSetReal1D(s_hist->bins, i, 0.0);
    }
    for(int i = (d.smax/s_bins_w) + 1; i < s_bins; i++){
        cvSetReal1D(s_hist->bins, i, 0.0);
    }


    //Draw 2D model hist, Poplulate visualization of histogram
    if(model_hist_on){
        cvZero(hist_img);
        cvGetMinMaxHistValue(d.model_hist_thresh, 0, &max_val, 0, 0);
        for( int h = 0; h<h_bins; h++){
            for( int s = 0; s<s_bins; s++){
                float bin_val = cvQueryHistValue_2D( d.model_hist_thresh, h, s );
                int intensity = cvRound( bin_val* 255*20*255/ max_val);
                cvRectangle(hist_img, cvPoint( h*HIST_SCALE, s*HIST_SCALE), cvPoint( (h+1)*HIST_SCALE - 1, (s+1)*HIST_SCALE - 1), CV_RGB(intensity,intensity,intensity) , CV_FILLED);
            }
        }
        cvNamedWindow("Model Histogram", CV_WINDOW_AUTOSIZE);
        cvShowImage("Model Histogram", hist_img);
    }

    //Filter to get rid of noise
    if(THRESH_BIN){
        cvThreshold(h_plane_still, h_plane_still, 0, 255, CV_THRESH_BINARY);
    }
    cvCopy(h_plane_still, frame_copy, 0); //before filtering
    //cvMorphologyEx(h_plane_still, h_plane_still, 0, 0, CV_MOP_OPEN, 2);
    //cvMorphologyEx(h_plane_still, h_plane_still, 0, 0, CV_MOP_CLOSE, 1);

    if(THRESH_BIN){
        cvThreshold(s_plane_still, s_plane_still, 0, 255, CV_THRESH_BINARY);
    }
    cvCopy(s_plane_still, frame_copy2, 0); //before filtering
    //cvMorphologyEx(s_plane_still, s_plane_still, 0, 0, CV_MOP_OPEN, 2);
    //cvMorphologyEx(s_plane_still, s_plane_still, 0, 0, CV_MOP_CLOSE, 1);

    cvShowImage( "Original", frame );

    //saving new values
    if(index == -1 ){
        histograms.push_back(d);
        //Reset variables
        vector_place = histograms.size()-1;
        new_hist = 0;
    }
    else{
        histograms[index] = d;
    }

}

CvPoint2D64f calc_P_control_output(CvPoint* obj_loc){
    //Print Velocity Vector
    double sum_x = 0;
    double sum_y = 0;
    center = cvPoint2D64f((frame->width)/2, (frame->height)/2);
    avg_loc = cvPoint2D64f(0.0, 0.0);
    ctrl_vec = cvPoint2D64f(0.0, 0.0);
    double mag_scale = 0;
    double time_diff = 0;

    //calculate avg position of centers
    for(int i = 0; i <= total_obj_found-1; i++){
        sum_x = obj_loc[i].x + sum_x;
        sum_y = obj_loc[i].y + sum_y;
    }

    if(total_obj_found == 0){
        avg_loc = cvPoint2D64f(center.x, center.y);
    }
    else
        avg_loc = cvPoint2D64f(sum_x/total_obj_found, sum_y/total_obj_found);


    if(WEIGHT_CONTROL && total_obj_found != 0){
        avg_loc = cvPoint2D64f(sum_x/area_total[vector_place], sum_y/area_total[vector_place]);
    }


    //Calcualte control/velocity vector
    ctrl_vec.x = -center.x + avg_loc.x;
    ctrl_vec.y = center.y - avg_loc.y;

    image_plane_vec.x = ctrl_vec.x;
    image_plane_vec.y = ctrl_vec.y;

    return ctrl_vec;
}

void draw_arrow(IplImage* frame, CvPoint2D64f ctrl_vec, int index){

    CvScalar arrow_color;

    if(index == 1)
        arrow_color = CVX_WHITE;
    if(index == 2)
        arrow_color = CVX_BLACK;

    //Draw Arrows //
    //Draw line between points
    CvPoint	center = cvPoint((frame->width)/2, (frame->height)/2);
    //Calcualte control/velocity vector
    ctrl_vec.x = center.x + ctrl_vec.x;
    ctrl_vec.y = center.y - ctrl_vec.y;
    cvLine(frame, center, cvPoint(ctrl_vec.x, ctrl_vec.y), arrow_color, 2, 8, 0);

    if(center.x != ctrl_vec.x && center.y != ctrl_vec.y){
        //Draw arrow tip
        double angle;  
        angle = atan2( (double) center.y -ctrl_vec.y, (double) center.x - ctrl_vec.x );    
        double hypotenuse; 
        hypotenuse = sqrt( pow(center.y - ctrl_vec.y,2) + pow(center.x - ctrl_vec.x,2) );

        center.x = (int) (ctrl_vec.x + 9 * cos(angle + PI / 4)); 
        center.y = (int) (ctrl_vec.y + 9 * sin(angle + PI / 4)); 
        cvLine(frame, center, cvPoint(ctrl_vec.x, ctrl_vec.y), arrow_color, 2); 

        center.x = (int) (ctrl_vec.x + 9 * cos(angle - PI / 4)); 
        center.y = (int) (ctrl_vec.y + 9 * sin(angle - PI / 4)); 
        cvLine(frame, center, cvPoint(ctrl_vec.x, ctrl_vec.y) , arrow_color, 2 );
    }
}

CvPoint2D64f find_object(IplImage* frame, Data d, int index){

    // Convert to HSV images 
    cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
    cvCvtPixToPlane( hsv_frame, h_plane_frame, s_plane_frame, v_plane_frame, 0);
    cvCvtPixToPlane( frame, r_plane_frame, g_plane_frame, b_plane_frame, 0);

    //Calculate back projection using model hist
    cvNormalizeHist(d.model_hist_thresh, HIST_SCALE*255);
    cvCalcBackProject( planes_frame, back_project, d.model_hist_thresh);
    cvNormalizeHist(d.model_hist_thresh, 1.0);

    //Filter to get rid of noise
    cvCopy(back_project, frame_copy, 0); //before filtering
    cvMorphologyEx(back_project, back_project, 0, 0, CV_MOP_OPEN, 1);
    cvMorphologyEx(back_project, back_project, 0, 0, CV_MOP_CLOSE, 2);

    cvCopy(back_project, contour_img);

    total_obj_found = 0;
    num_obj = 10;
    obj_loc = (CvPoint*)malloc( num_obj*sizeof(CvPoint)); //freed
    weighted_obj_loc = (CvPoint*)malloc( num_obj*sizeof(CvPoint));	//freed 	

    CvSeq* c_new ;
    CvSeq* c;

    CvScalar box_color;
    CvScalar center_color;
    //Find Connected Compenents of backprojection and compute bounding boxes/centers
    if(bound_box){
        int *num = &num_obj;
        //CvBox2D* bbs = (CvBox2D*)malloc( num_obj*sizeof(CvBox2D*));
        CvRect* bbs = (CvRect*)malloc( num_obj*sizeof(CvRect));	//freed
        CvPoint* centers = (CvPoint*)malloc( num_obj*sizeof(CvPoint));	//freed
        CvMemStorage* mem_storage = cvCreateMemStorage(0);	//released
        CvSeq* contours;

        cvClearMemStorage(mem_storage);

        CvContourScanner scanner = cvStartFindContours(contour_img, mem_storage, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

        int numCont = 0;
        while( (c = cvFindNextContour(scanner)) != NULL ) {
            double len = cvContourPerimeter(c);
            double q = (contour_img->height + contour_img->width)/PERIMSCALE;

            //Perform logic on the contours identified

            //Store bounding box dimensions
            bbs[0] = cvBoundingRect(c);

            //Reject based on size of the blob
            if( len < q){
                cvSubstituteContour(scanner, NULL);
            }

            /*Reject Based on Dimensions
              if(bbs[0].height < (.2*max_height) || bbs[0].width < (.2*max_width){
              cvSubstituteContour(scanner, NULL);
              }
             */

            //Contour Passed logic, we assume it is a whale and add it to scanner
              else{				
                  //Use convex hull algorithm or not
                  if(POLY1_HULL0){
                      c_new = cvApproxPoly(c, sizeof(CvContour), mem_storage, CV_POLY_APPROX_DP, CVCONTOUR_APPROX_LEVEL, 0);

                  }
                  else{
                      c_new = cvConvexHull2(c, mem_storage, CV_CLOCKWISE, 1);

                  }
                  cvSubstituteContour(scanner, c_new);
                  numCont++;
              }
        }

        contours = cvEndFindContours(&scanner);

        IplImage *temp;	//released
        cvZero(contour_img);
        area_total[index] = 0;
        //Calculate center of mass and/or bounding rectangles for one frame for now
        if(*num != 0){
            int N = *num, numFilled = 0, i = 0;
            CvMoments moments;
            double M00, M01, M10;
            temp = cvCloneImage(contour_img);
            for(i=0, c=contours; c!=NULL; c = c->h_next, i++){
                if(i<N){
                    cvDrawContours(temp, c, CVX_WHITE, CVX_BLACK, -1, CV_FILLED, 8);
                    if( centers != NULL){
                        cvMoments(temp, &moments,1);
                        M00 = cvGetSpatialMoment(&moments, 0, 0);
                        M10 = cvGetSpatialMoment(&moments, 1, 0);
                        M01 = cvGetSpatialMoment(&moments, 0, 1);
                        pix_total = M00;
                        centers[i].x = (int)(M10/M00);
                        centers[i].y = (int)(M01/M00);

                        if(index == 1)
                            center_color = CVX_WHITE;
                        if(index == 2)
                            center_color = CVX_BLACK;

                        cvCircle(frame_final, cvPoint(centers[i].x, centers[i].y), 3, center_color, 1);

                    }
                    //Bounding rectangles 
                    if (bbs != NULL){
                        bbs[i] = cvBoundingRect(c);

                        if(index == 1)
                            box_color = CVX_WHITE;
                        if(index == 2)
                            box_color = CVX_BLACK;

                        cvRectangle(frame_final, cvPoint(bbs[i].x, bbs[i].y), cvPoint(bbs[i].x+bbs[i].width, bbs[i].y+bbs[i].height), box_color, 3);
                        //					bbs[i] = cvFitEllipse2(c);
                        //					bbs[i].angle = bbs[i].angle;
                        //					cvEllipseBox(frame2, bbs[i], CVX_RED, 3);
                        //pix_total = (bbs[i].width)*(bbs[i].height); //total bounding box area
                    }

                    cvZero(temp);
                    obj_loc[numFilled].x = centers[numFilled].x;
                    obj_loc[numFilled].y = centers[numFilled].y;

                    weighted_obj_loc[numFilled].x = (centers[numFilled].x)*pix_total;
                    weighted_obj_loc[numFilled].y = (centers[numFilled].y)*pix_total;
                    area_total[index] = area_total[index] + pix_total; //total area of all objects for a specific histogram

                    numFilled++;
                }
                cvDrawContours(contour_img, c, CVX_WHITE, CVX_BLACK, -1, CV_FILLED, 8);
            }
            //*num = numFilled;
            total_obj_found = numFilled;
            cvReleaseImage(&temp);
        }
        else{
            for( c = contours; c !=NULL; c = c->h_next){
                cvDrawContours(contour_img, c, CVX_WHITE, CVX_BLACK, -1, CV_FILLED, 8);printf("10\n");
            }
        }

        //Clean-up
        free(bbs);
        free(centers);
        cvReleaseMemStorage(&mem_storage);

    }

    //Create separate back projection and contour images for each hist
    cvCopy(back_project, back_projection_final);
    cvCopy(contour_img, contours_final);

    //Calculate velocity vector 
    ctrl_vec_out = calc_P_control_output(obj_loc);

    target_area = area_total[index];

    return ctrl_vec_out;

    //Clean-up
    free(obj_loc);
    free(weighted_obj_loc);

}

void on_trackbar_hmax(int val);

void on_trackbar_hmin(int val);

void on_trackbar_smax(int val);

void on_trackbar_smin(int val);

CvCapture* load_AVI(){
    //Load Video
    CvCapture* capture = cvCaptureFromAVI("/Users/wilselby/Desktop/DRL/Videos/Classifier Movies/Vision System Input/Boat5Edit.m4v");   //whales_long  whales09 seals.m4v Boats1.mp4 barge.m4v boats_cross.mp4 iphone.m4v 

    if(!capture){
        fprintf(stderr, "Could not initialize capturing...");
    }

    return capture;
}


/*
   static void rcv_img(const sensor_msgs::ImageConstPtr& msg){

   if(!inFunction){
   inFunction = true;
   }
   else {
   printf("Was already in function..returning from rcv_img \n");
   return;
   }

   pthread_mutex_lock(&image_lock);

   try
   {
   rcvdFrame = bridge_.imgMsgToCv(msg, "bgr8");
   }
   catch (sensor_msgs::CvBridgeException error)
   {
   ROS_ERROR("cv_bridge exception");
   return;
   }

//Fill in openCV IplImage
pthread_mutex_unlock(&image_lock);


//cvNamedWindow("Live Stream", CV_WINDOW_AUTOSIZE );
//cvShowImage("Live Stream", rcvdFrame );

//cv::imshow(WINDOW, cv_ptr->image);

//image_pub_.publish(cv_ptr->toImageMsg());

//printf("rcvd %lld \n",msg->utime);

inFunction = false;
img_rcvd = 1;

}
 */

int main( int argc, char** argv) {

    printf("Welcome to Object ID, using Opencv version %s (%d.%d.%d)\n", CV_VERSION, CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_SUBMINOR_VERSION);	

    printf( "Hot keys for manually thresholding: \n"
            "\tESC - quit the program\n"
            "\th - Thershold the Hue Plane\n"
            "\ts - Threshold the Sat Plane\n"
            "\tm - Show the 2D Model Histogram\n"
            "\tt - To Exit Threshold mode once completed\n");
    printf("\n" 
            "Hot keys for active contours: \n"
            "\tESC - quit the program\n"
            "\tb - Show the bounding box\n"
            "\tp - Show the Back Projection Image\n"
            "\tl - Toggle Beach logic\n"
            "\to - Toggle Ocean Logic\n"
            "\tn - Create a new and distinct model histogram\n"
            "\tc - Cycle through histograms\n"
            "\tt - To Exit video mode and re-create model histogram\n\n");
    //ROS
    //: it_(nh_)
    //image_pub_ = it_.advertise("myfirstimagestream", 1);
    //image_sub_ = it_.subscribe("//wide_stereo/left/image_rect_color", 1, &ImageConverter::imageCb, this);

    ros::init(argc, argv, "image_converter");
    ros::NodeHandle node_handle;
    //ros::Publisher chatter_pub = node_handle.advertise<std_msgs::String>("chatter", 1000);
    ImageConverter ic(node_handle);

    //TODO Tyler: Make sure this works.
    //Arbitrarily chose 1000 for the msg buffer. Old code: mapping_msgs::AttachedCollisionObject
    //What message type should I use? Pointcloud2? (sensor_msgs::PointCloud2) Make my own?
    // Right now I'm just using a Float64MultiArray with each x,y pair as subsequent  array elements
    // Array = x1, y1, x2, y2, ..., xN, yN;
    //ros::Publisher hist_x_y_output = n.advertise<std_msgs::String>("ctrl_vec_out_list", 1000);
    //resize from lcm

    printf("done with ic init\n");
    int counter = 0;
    //while(1);
    while(rcvdFrame == NULL || counter < 11){
        printf("\n in loop");
        printf("\n in loop");
        printf("\n in loop");
        printf("\n in loop");
        printf("\n in loop");
        printf("\n in loop");
        printf("\n kljljin loop");
        printf("\n in loop");
        printf("\n in loop");

        printf("\ncounter: %d", counter);
        ros::spinOnce();
        rcvdFrame = ic.getImage();
        if (rcvdFrame == NULL) {
            printf("null frame \n");
            continue;
        } else if (counter < 10) {
            counter++;
            printf("\n skipping because of counter");
            continue;
        } else {
            counter++;
            printf("not null frame \n");
            // TODO: put flag here or something
            //lcm_handle(lcm);	//blocks until image is read, then initializesZ

            still = cvCreateImage( cvSize(320,240), IPL_DEPTH_8U, 3);
            cvResize(rcvdFrame, still);
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            printf("\n after cvResize");
            //Initialize Variables
            initialize();
            break;
        }
        printf("After else in while loop");
    }
    printf("after the first loop \n");
    printf("after the first loop \n");
    printf("after the first loop \n");
    printf("after the first loop \n");
    printf("after the first loop \n");
    printf("after the first loop \n");
    printf("after the first loop \n");
    printf("after the first loop \n");
    printf("after the first loop \n");
    printf("after the first loop \n");
    printf("after the first loop \n");
    printf("after the first loop \n");
    printf("after the first loop \n");

    //Capture first frame in the video to initialize variables
    //capture = load_AVI();
    //rcvdFrame = cvQueryFrame(capture);

    //rcvdFrame = cvLoadImage("/Users/wilselby/Desktop/DRL/MS Thesis/Old/Object Tracking/Stills/IROS_Still.jpg"); //pgr_still.jpg IROS_Still
    //still = cvCreateImage( cvSize(rcvdFrame->width/1,rcvdFrame->height/1), IPL_DEPTH_8U, 3);
    cvResize(rcvdFrame, still);

    //Initialize Variables
    //initialize();

    int create_windows = 1;

    while(1){
        ros::spinOnce();

        //get next video frame
        //rcvdFrame  = cvQueryFrame(capture);
        rcvdFrame = ic.getImage();
        cvResize(rcvdFrame, still);

        //If using still
        //cvCopy(still,frame);

        cvCopy(frame, frame_final);


        cvResize(rcvdFrame, frame);	
        cvCopy(frame, frame_final);

        if(thresholding_on && create_windows){
            //Destroy Output Windows
            cvDestroyAllWindows();

            //Create thresholding windows
            cvNamedWindow( "Original", CV_WINDOW_AUTOSIZE );
            cvMoveWindow("Original", 200, 400);
            cvNamedWindow("Back Projection", CV_WINDOW_AUTOSIZE);
            cvMoveWindow("Back Projection",600, 400);
            cvNamedWindow( "Hue Plane", CV_WINDOW_AUTOSIZE );
            cvMoveWindow("Hue Plane",200, 0);
            cvNamedWindow( "Sat Plane", CV_WINDOW_AUTOSIZE );
            cvMoveWindow("Sat Plane",600, 0);

            //Set flag
            create_windows = 0;
        }	


        //Continue to update the model histogram as the user manually thresholds image, user hits key ('t') once finsihed thresholding
        if(thresholding_on){

            //If user wants to make a new histogram, change vector_place variable so threshold function knows to use new values and add to total number of hists
            if(new_hist == -1){

                vector_place = new_hist;
                /*
                   global_data.hmax = 142;	//HUE_MAX;	    
                   global_data.hmin = 107;	//HUE_MIN;	
                   global_data.smax = 222;	//SAT_MAX;	 
                   global_data.smin = 109;	//SAT_MIN;  
                 */
            }

            //hmax- 142 hmin- 107 smax- 222 smin- 109
            //180, 112, 222, 136

            //Set local variable to index location and load old values
            else{
                global_data = histograms[vector_place];
            }

            temp_hmax = global_data.hmax;
            temp_hmin = global_data.hmin;
            temp_smax = global_data.smax;
            temp_smin = global_data.smin;

            cvCreateTrackbar( "Hue Max", "Hue Plane", &temp_hmax, 180, on_trackbar_hmax);
            cvCreateTrackbar( "Hue Min", "Hue Plane", &temp_hmin, 180, on_trackbar_hmin);	
            cvCreateTrackbar( "Sat Max", "Sat Plane", &temp_smax, 256, on_trackbar_smax);
            cvCreateTrackbar( "Sat Min", "Sat Plane", &temp_smin, 256, on_trackbar_smin);

            while(thresholding_on){

                threshold( frame, vector_place, global_data);

                keystroke();

            } 
            printf("\nhmax- %d hmin- %d smax- %d smin- %d\n", temp_hmax, temp_hmin, temp_smax, temp_smin);
        }		

        if(!create_windows){
            //Close thresholding windows
            cvDestroyAllWindows();

            //Create Windows
            cvNamedWindow("Tracking Output", CV_WINDOW_AUTOSIZE);

            if(b_projection){
                cvNamedWindow("Back Projection", CV_WINDOW_AUTOSIZE);
            }

            if(contour_on){
                cvNamedWindow("Contours", 1);	//this is primary output image
            }	

            //Create logic trackbars
            cvCreateTrackbar( "Max Contour Size", "Tracking Output", &PERIMSCALE, 40, 0 );

            //Reset flag
            create_windows = 1;
        }

        //TODO Tyler: Make sure I can actually publish this type.
        //CvPoint2D64f ctrl_vec_out_list[histograms.size()];
        //double ctrl_vec_out_list[histograms.size()*2];
        //std_msgs::Float64MultiArray ctrl_vec_out_list;
        //Need to define multiple dimension

        // Apply Model histogram to rest of video 
        for(int ii = 0; ii< histograms.size(); ii++){
            global_data = histograms[ii];
            ctrl_vec_out = find_object(frame, global_data, ii);
            keystroke();	
            draw_arrow(frame_final, ctrl_vec_out,ii);
            printf("Object %d Control Vector: %.3f,%.3f \n",ii,ctrl_vec_out.x, ctrl_vec_out.y);
            //final_ctrl_vec.x = (ctrl_vec_out.x + ii*final_ctrl_vec.x)/(ii+1);
            //final_ctrl_vec.y = (ctrl_vec_out.y + ii*final_ctrl_vec.y)/(ii+1);

            //TODO Tyler: Just a marker.
            //ctrl_vec_out_list[ii] = ctrl_vec_out;
            //ctrl_vec_out_list[ii*2] = ctrl_vec_out.x;
            //ctrl_vec_out_list[ii*2+1] = ctrl_vec_out.y;
            //ctrl_vec_out_list.data.push_back(ctrl_vec_out.x);
            //ctrl_vec_out_list.data.push_back(ctrl_vec_out.y);
        }
        // TODO: put publisher here.  store the x and y for each iteration fo the for loop above, then publish

        //TODO Tyler: Create and populate this ctrl_vec_out_list variable.
        //This (hopefully) publishes an array of doubles, in the form x1, y1, x2, y2, ..., xN, yN.
        //hist_x_y_output.publish(ctrl_vec_out_list);

        printf("\n\n");
        //printf("Final Control Vector: %.3f,%.3f, \t%.2f \n",image_plane_vec.x, image_plane_vec.y,target_area);		

        //draw_arrow(frame_final, final_ctrl_vec);

        //send_message(lcm);

        final_ctrl_vec = cvPoint2D64f(0.0, 0.0);

        cvShowImage("Tracking Output", frame_final);

        if(b_projection){
            cvShowImage("Back Projection", back_projection_final);
        }

        if(contour_on){
            cvResize(contours_final, OUTPUT);
            cvShowImage("Contours", OUTPUT);
        }	

    }	

    //Clean-up
    cvReleaseImage(&back_project);
    cvReleaseImage(&contour_img);
    cvReleaseImage(&hsv_frame);
    cvReleaseImage(&h_plane_frame);
    cvReleaseImage(&s_plane_frame);
    cvReleaseImage(&v_plane_frame);
    cvReleaseImage(&r_plane_frame);
    cvReleaseImage(&g_plane_frame);
    cvReleaseImage(&b_plane_frame);
    cvReleaseImage(&h_histimg_thresh);
    cvReleaseImage(&s_histimg_thresh);
    cvReleaseImage(&hsv_still);
    cvReleaseImage(&h_plane_still);
    cvReleaseImage(&h_plane_orig);
    cvReleaseImage(&s_plane_orig);
    cvReleaseImage(&v_plane_orig);
    cvReleaseImage(&s_plane_still);
    cvReleaseImage(&v_plane_still);
    cvReleaseImage(&frame_copy);
    cvReleaseImage(&frame_copy2);
    cvReleaseImage(&hist_img);
    cvReleaseHist(&h_hist);
    cvReleaseHist(&s_hist);
    cvReleaseHist(&model_hist);
    cvReleaseImage(&gamma_corrected);
    cvReleaseImage(&back_projection_final);
    cvReleaseImage(&contours_final);

    cvDestroyWindow("Back Projection");
    cvDestroyWindow("Contours");
    cvDestroyWindow("Tracking Output");


    cvReleaseImage(&still);
    cvReleaseImage(&frame_final);
    cvReleaseImage(&frame);
    cvReleaseImage(&Resize);
    cvReleaseImage(&Resize2);
    cvReleaseCapture(&capture);
}	

/* Trackbar function */
void on_trackbar_hmax(int val){
    cvSetTrackbarPos("Hue Max", "Hue Plane", val);
}

void on_trackbar_hmin(int val){
    cvSetTrackbarPos("Hue Min", "Hue Plane", val);
}

void on_trackbar_smax(int val){
    cvSetTrackbarPos("Sat Max", "Sat Plane", val);
}

void on_trackbar_smin(int val){
    cvSetTrackbarPos("Sat Min", "Sat Plane", val);
}








