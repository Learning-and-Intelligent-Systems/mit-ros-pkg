#include "util.h"
#include <cstdio>
#include <cstring>

int main(int argc, char** argv) {

  char file_in[100] = "images/";
  strcat(file_in, argv[1]);

  IplImage* original = cvLoadImage(file_in);
  if (original == 0) {
    ROS_ERROR("Image not found");
  } else {
       
    IplImage* normalized = normalizeImage(original);
    
    char file_out[100];
    strncpy(file_out, file_in, strlen(file_in)-4);
    file_out[strlen(file_in)-4] = '\0';
    strcat(file_out, "_normalized.jpg");
    cvSaveImage(file_out, normalized);
    cvReleaseImage(&normalized);
    cvReleaseImage(&original);
   
  }
}
