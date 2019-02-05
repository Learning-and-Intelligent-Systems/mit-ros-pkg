#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/stereo_camera_model.h>
//#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <bingham.h>
#include <bingham/util.h>
#include "util.h"
#include <pingpong/BallState.h>
#include <pingpong/EyesState.h>
#include <pingpong/Circle.h>

//dbug (for ICRA14 bounce experiments)
#include <signal.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

// SSE instructions
#include <xmmintrin.h>
#include <emmintrin.h>
#include <mmintrin.h>


using namespace std;


typedef struct {
  list<sensor_msgs::Image> left;
  list<sensor_msgs::Image> right;
} stereo_queue_t;

//#define ROBOT_SIDE 0
//#define OPPONENT_SIDE 1

typedef struct {
  //int side;  // 0 for robot's side, 1 for opponent's side
  vector< vector<ball_t> > live_trajectories;
  vector< vector<ball_t> > dead_trajectories;
  ros::Publisher ball_pub;
  ros::Publisher trajectory_pub;
  ros::Publisher prediction_pub;
  ros::Publisher state_pub;
  ros::Publisher left_ball_image_pub;
  //ros::Publisher right_ball_image_pub;
} ball_tracker_t;


typedef struct {
  uchar bg[640*480] __attribute__ ((aligned (16)));
  uchar last[640*480] __attribute__ ((aligned (16)));
  uint w, h;  
  rect_t roi;
  sensor_msgs::Image last_msg;
  vector<ball_t> ball_detections;
  ros::Publisher pub[3];    // for debugging
} eye_memory_t;


typedef struct {
  double **vectors;  // n-by-d
  double *alpha;     // n
  double *shift;     // d
  double *scale;     // d
  double bias;
  int n;  // num vectors
  int d;  // dim of each vector
  double rbf_sigma;
} svm_t;


typedef struct {
  double B_avg[1024];
  double **V;  // principal components (n-by-1024)
  double **W;  // regression coefficients (4-by-n+1)
  int n;
  svm_t svm;   // classifier
} logo_detector_t;


// global objects
ros::Time current_time(0);
stereo_queue_t stereo_queue;
ball_tracker_t ball_tracker;
eye_memory_t left_eye;
eye_memory_t right_eye;
image_geometry::StereoCameraModel stereo_camera_model;
//cv::Point3d table_corners[4];
tf::Transform table_camera_transform, camera_table_transform;
//tf::TransformListener *tf_listener;
logo_detector_t logo_detector;
bool predict_spin = false;  //dbug
double player_rh_vel[3];
double player_rh_vel_time=0.0;
sensor_msgs::PointCloud paddle_plan;
bool have_paddle_plan = false;

//#define ICRA_EXP 1

#ifdef ICRA_EXP
//DBUG (for ICRA14 bounce experiments)
vector<sensor_msgs::Image> icra_ball_images;
vector<double> icra_ball_states;
#endif


// parameters
//const double bg_lambda = .1;
const short ball_thresh = 30; //50;
const int ball_min_size = 20;                     // pixels
const int ball_max_size = 500;                    // pixels
const double ball_stereo_y_thresh = 10;           // pixels
const double trajectory_NN_max_dist = .4;         // meters
const double ball_trajectory_NN_max_dist = .4;    // meters
const double ball_trajectory_timeout = .05;       // seconds
const double ball_min_velocity = 0.5;             // meters/second
const int ball_trajectory_min_length = 5;         // frames
//const double bg_rate = .1;                        // seconds
const bool publish_images = false;
//const double draw_rate = .1;                      // seconds
const bool publish_state = true;
const double publish_state_rate = .1; //.1;            // seconds (-1 for continuous)




//-----------------------  SVM  -----------------------//

svm_t load_svm(char *f_vectors, char *f_alpha, char *f_params)
{
  svm_t S;
  int n, d, d2, xxx;

  S.vectors = load_matrix(f_vectors, &n, &d);
  S.n = n;
  S.d = d;
  safe_calloc(S.alpha, n, double);
  safe_calloc(S.shift, d, double);
  safe_calloc(S.scale, d, double);

  double **alpha = load_matrix(f_alpha, &xxx, &n);  //TODO: check that n==S.n
  memcpy(S.alpha, alpha[0], S.n*sizeof(double));
  free_matrix2(alpha);

  double **params = load_matrix(f_params, &xxx, &d2);  //TODO: check that d==2*S.d+1
  S.bias = params[0][0];
  memcpy(S.shift, &params[0][1], S.d*sizeof(double));
  memcpy(S.scale, &params[0][1+S.d], S.d*sizeof(double));
  free_matrix2(params);

  S.rbf_sigma = 1.0;  //TODO: load this from file

  return S;
}


inline double rbf_kernel(double *x, double *y, int n, double sigma)
{
  return exp(-dist2(x,y,n) / (2.0*sigma*sigma));
}


bool svm_classify(svm_t svm, double *x)
{
  int n = svm.n;
  int d = svm.d;

  double x2[d];
  for (int i = 0; i < d; i++)
    x2[i] = svm.scale[i]*(x[i] + svm.shift[i]);

  double f = svm.bias;
  for (int i = 0; i < n; i++)
    f += svm.alpha[i]*rbf_kernel(x2, svm.vectors[i], d, svm.rbf_sigma);
  
  //printf("svm d=%d, n=%d, f = %f\n", d, n, f); //dbug

  return (f < 0.0);
}


//-----------------------  Logo Detector  ----------------------//


logo_detector_t load_logo_detector(std::string &data_folder)
{
  char f1[1024], f2[1024], f3[1024];

  logo_detector_t D;
  int n, m;

  sprintf(f1, "%s/logo_b_avg.txt", data_folder.c_str());
  double **logo_b_avg = load_matrix(f1, &n, &m);  //TODO: check if n==1 and m==1024
  memcpy(D.B_avg, logo_b_avg[0], 1024*sizeof(double));
  free_matrix2(logo_b_avg);

  sprintf(f1, "%s/logo_principal_components.txt", data_folder.c_str());
  D.V = load_matrix(f1, &n, &m);  //TODO: check if m==1024
  D.n = n;

  sprintf(f1, "%s/logo_regression_coeffs.txt", data_folder.c_str());
  D.W = load_matrix(f1, &n, &m);  //TODO: check if n==4 and m==D.n+1

  sprintf(f1, "%s/logo_svm_vectors.txt", data_folder.c_str());
  sprintf(f2, "%s/logo_svm_alpha.txt", data_folder.c_str());
  sprintf(f3, "%s/logo_svm_params.txt", data_folder.c_str());
  D.svm = load_svm(f1, f2, f3);
  
  return D;
}


/*
 * Returns true iff logo is found, and puts logo coordinates in p = {x,y,dx,dy}
 */
bool detect_logo(double *p, uchar *ball_image, logo_detector_t D)
{
  // convert ball image to double and subtract off B_avg
  double b[1024];
  for (int i = 0; i < 1024; i++)
    b[i] = ball_image[i]/256. - D.B_avg[i];

  // project into pcs (principal components) space
  int n = D.n;
  double x[n];
  matrix_vec_mult(x, D.V, b, n, 1024);

  // classify with SVM on pcs
  bool found_logo = svm_classify(D.svm, x);

  if (!found_logo)
    return false;

  // estimate logo pose in image with regression in pcs space
  double x2[n+1];
  x2[0] = 1.0;
  memcpy(&x2[1], x, n*sizeof(double));
  matrix_vec_mult(p, D.W, x2, 4, n+1);
  normalize(&p[2], &p[2], 2);

  // reconstruct ball image (from pcs)
  vec_matrix_mult(b, x, D.V, n, 1024);
  uchar B[1024];
  for (int i = 0; i < 1024; i++)
    B[i] = 256.*(b[i] + D.B_avg[i]);

  // adjust logo image position on reconstructed ball image (from pcs)
  int mx=round(p[0]), my=round(p[1]);
  for (int i = 0; i < 1024; i++)
    B[i] = (B[i] ? 256-B[i] : 0);
  image_find_local_mode(mx, my, B, 32, 32, 2, 4);

  //printf("p = [%.2f, %.2f, %.2f, %.2f]\n", p[0], p[1], p[2], p[3]); //dbug

  //dbug
  p[2] = 1.0;
  p[3] = 0.0;

  return true;
} 





void publish_ball_image(uchar *ball_image)
{
  const int w = 32;

  sensor_msgs::Image msg;
  msg.width = w;
  msg.height = w;
  msg.step = w;
  msg.data.resize(w*w);
  msg.encoding = sensor_msgs::image_encodings::MONO8;
  msg.header.stamp = current_time;
  std::copy(ball_image, ball_image+w*w, msg.data.begin());
  ball_tracker.left_ball_image_pub.publish(msg);
}


void publish_ball_image_with_logo(uchar *ball_image, int x, int y, int x2, int y2)
{
  const int w = 32;

  sensor_msgs::Image msg;
  msg.width = w;
  msg.height = w;
  msg.step = 3*w;
  msg.data.resize(3*w*w);
  msg.encoding = sensor_msgs::image_encodings::RGB8;
  msg.header.stamp = current_time;
  
  for (int i = 0; i < 1024; i++)
    msg.data[3*i] = msg.data[3*i+1] = msg.data[3*i+2] = ball_image[i];

  msg.data[3*(y*w+x)] = 255;  msg.data[3*(y*w+x)+1] = 0;  msg.data[3*(y*w+x)+2] = 0;
  msg.data[3*(y2*w+x2)] = 0;  msg.data[3*(y2*w+x2)+1] = 255;  msg.data[3*(y2*w+x2)+2] = 0;

  ball_tracker.left_ball_image_pub.publish(msg);
}



//------------------------- Ball Spin Estimation -----------------------//

bool estimate_ball_orientation_from_logo(ball_t &ball)
{
  uchar *ball_image = ball.image;

  uchar B[32*32], B2[32*32];
  memcpy(B, ball_image, 32*32);
  const int w = 32;
  const double r = 10.;

  // 1. Use imfilter to align ball image with avg. ball image.
  int dx, dy;
  uchar B_avg[1024];
  for (int i = 0; i < 1024; i++)
    B_avg[i] = 256.*logo_detector.B_avg[i];
  compute_image_alignment(dx, dy, ball_image, B_avg, w, w, 5);
  shift_image(B, ball_image, w, w, dx, dy);

  // 2. Remove pixels outside ball radius
  for (int y = 0; y < w; y++) {
    double py = y + .5 - w/2.0;
    for (int x = 0; x < w; x++) {
      double px = x + .5 - w/2.0;
      if (px*px+py*py > r*r)
	B[y*w+x] = 0;
    }
  }

  // 3. Detect the logo's presence and get an initial guess of its position
  double p[4];
  bool found_logo = detect_logo(p, B, logo_detector);
  int mx=round(p[0]), my=round(p[1]);

  if (!found_logo) {
    publish_ball_image(B);  //dbug
    memset(ball.oq, 0, 4*sizeof(double));
    return false;
  }

  publish_ball_image_with_logo(B, mx, my, mx+round(5*p[2]), my+round(5*p[3]));  //dbug
  //publish_ball_image(B);  //dbug
  
  // 5. Convert to 3-D logo position and orientation
  double px = (mx + .5 - w/2.) / r;
  double py = (my + .5 - w/2.) / r;
  double pz = -sqrt(1.0 - px*px - py*py);
  double mu[3] = {px, py, pz};
  double dp[3] = {p[2], p[3], 0.0};
  double proj_dp_mu[3];
  proj(proj_dp_mu, dp, mu, 3);
  double pc[3];
  sub(pc, dp, proj_dp_mu, 3);

  // 6. Use "mu" and "pc" to generate a quaternion orientation, q.
  double **R = new_matrix2(3,3);
  R[0][0] = mu[0];  R[0][1] = pc[0];  R[0][2] = mu[1]*pc[2] - mu[2]*pc[1];
  R[1][0] = mu[1];  R[1][1] = pc[1];  R[1][2] = mu[2]*pc[0] - mu[0]*pc[2];
  R[2][0] = mu[2];  R[2][1] = pc[2];  R[2][2] = mu[0]*pc[1] - mu[1]*pc[0];
  double q[4];
  rotation_matrix_to_quaternion(q, R);
  free_matrix2(R);

  // 7. correct for camera angle
  tf::Point b_cam = table_camera_transform(tf::Point(ball.x, ball.y, ball.z));
  double v_cam[3] = {b_cam.x(), b_cam.y(), b_cam.z()};
  normalize(v_cam, v_cam, 3);
  double a = acos(v_cam[2]);
  double q_image_to_cam[4] = {cos(a/2.0), -sin(a/2.0)*v_cam[1], sin(a/2.0)*v_cam[0], 0.0};  // v = cross([0,0,1], v_cam);
  quaternion_mult(q, q_image_to_cam, q);

  // 8. rotate into table coordinate frame
  double q_cam_to_table[4] = {0,1,0,0};
  quaternion_mult(ball.oq, q_cam_to_table, q);

  return true;
}



//bool estimate_ball_orientation_from_logo(double *q, uchar *ball_image) //, int w, double r)
/*
bool estimate_ball_orientation_from_logo(ball_t &ball) //, int w, double r)
{
  uchar *ball_image = ball.image;

  uchar B[32*32], B2[32*32];
  double D[32*32];
  memcpy(B, ball_image, 32*32);
  const int w = 32;
  const double r = 10.;
  uchar w_thresh = 50; //72;


  //dbug: publish left ball image
  sensor_msgs::Image ball_image_msg;
  ball_image_msg.width = w;
  ball_image_msg.height = w;
  ball_image_msg.step = w;
  ball_image_msg.data.resize(w*w);
  ball_image_msg.encoding = sensor_msgs::image_encodings::MONO8;
  ball_image_msg.header.stamp = current_time;


  // 2. Use imfilter to align ball image with avg. ball image.
  int dx, dy;
  compute_image_alignment(dx, dy, B, B_avg, w, w, 5);
  shift_image(B2, B, w, w, dx, dy);

  // 3. Remove pixels outside ball radius
  for (int y = 0; y < w; y++) {
    double py = y + .5 - w/2.0;
    for (int x = 0; x < w; x++) {
      double px = x + .5 - w/2.0;
      if (px*px+py*py > r*r)
	B2[y*w+x] = 0;
    }
  }


  //dbug
  std::copy(B2, B2+w*w, ball_image_msg.data.begin());
  ball_tracker.left_ball_image_pub.publish(ball_image_msg);


  // update B_avg
  //for (int i = 0; i < w*w; i++)
  //  B_avg[i] = round(.99*B_avg[i] + .01*B2[i]);


  // 4. Normalize (and invert) lighting w.r.t. avg. (aligned) ball image.
  for (int i = 0; i < w*w; i++) {
    if (B2[i] == 0)  // outside ball radius
      D[i] = 0;
    else
      D[i] = 1. - (B2[i] / (double) B_avg[i]);
  }

  //printf("max(D) = %f\n", arr_max(D, w*w)); //dbug
  //printf("B2 = [");

  // 5. Normalize lighting.
  double D_mean = sum(D, w*w) / (double)(w*w);
  for (int i = 0; i < w*w; i++) {
    if (B2[i]) {
      double p = D[i] - D_mean;
      p = MIN(MAX(p, 0.0), 1.0);
      B2[i] = (uchar)(p*255);
    }
    //printf("%d ", B2[i]); //dbug
  }
  //printf("]\n");
  
  // 6. Find weighted centroid of pixels.
  int mx, my;
  image_centroid(mx, my, B2, w, w);

  printf("image centroid = (%d, %d)\n", mx, my); //dbug

  // 7. Use mean-shift (local 5x5 grid search) on blurred ball image to find local pixel max.
  blur_image(B, B2, w, w, 5);
  image_find_local_mode(mx, my, B, w, w, 5);


  //dbug
  //uchar xxx=0; for (int i = 0; i < w*w; i++) if (B[i] > xxx) xxx = B[i];
  //printf("max(B) = %d\n", xxx);

  //printf("  logo pixel: %d (%d, %d)\n", B[my*w+mx], mx, my); //dbug

  //dbug
  std::copy(B2, B2+w*w, ball_image_msg.data.begin());
  if (B[my*w+mx] >= w_thresh)
    for (int i = 0; i < w; i++)
      ball_image_msg.data[i*w] = ball_image_msg.data[i*w+1]
	= ball_image_msg.data[i*w+w-1] = ball_image_msg.data[i*w+w-2] = 255;
  ball_tracker.left_ball_image_pub.publish(ball_image_msg);

  // 8. If local max pixel value < w_thresh, return (no logo found).
  if (B[my*w+mx] < w_thresh)
    return false;

  // 9. Mask out pixels farther than r/2 away from the logo center.
  for (int y = 0; y < w; y++)
    for (int x = 0; x < w; x++)
      if ((y-my)*(y-my) + (x-mx)*(x-mx) > r*r/4.)
	B[y*w+x] = 0;

  // 10. Compute 3D pixel locations (and weights) on ball.
  double **P = new_matrix2(w*w, 3);
  double W[w*w];
  int cnt = 0;
  for (int y = 0; y < w; y++) {
    for (int x = 0; x < w; x++) {
      int i = y*w+x;
      if (B[i] > 0) {
	double px = (x + .5 - w/2.) / r;
	double py = (y + .5 - w/2.) / r;
	double pz2 = 1 - px*px - py*py;
	if (pz2 > 0.) {
	  P[cnt][0] = px;
	  P[cnt][1] = py;
	  P[cnt][2] = -sqrt(pz2);  //fix
	  W[cnt++] = B[i];
	}
      }
    }
  }

  // 11. Compute weighted 3D logo center.
  double mu[3];
  wmean(mu, P, W, cnt, 3);
  normalize(mu, mu, 3);

  // 12. Project logo points into tangent space at "mu".
  for (int i = 0; i < cnt; i++) {
    double proj_p_mu[3];
    proj(proj_p_mu, P[i], mu, 3);
    sub(P[i], P[i], proj_p_mu, 3);
  }

  // 13. Compute weighted principal curvature direction, "pc".
  double **S = new_matrix2(3,3);
  double zeros[3] = {0., 0., 0.};
  wcov(S, P, W, zeros, cnt, 3);
  double z[3];
  double **V = new_matrix2(3,3);
  eigen_symm(z, V, S, 3);
  double *pc = V[2];
  double pcr = z[2]/z[1]; //z[3]/z[2];

  // 14. Use "mu" and "pc" to generate a quaternion orientation, q.
  double **R = S;
  R[0][0] = mu[0];  R[0][1] = pc[0];  R[0][2] = mu[1]*pc[2] - mu[2]*pc[1];
  R[1][0] = mu[1];  R[1][1] = pc[1];  R[1][2] = mu[2]*pc[0] - mu[0]*pc[2];
  R[2][0] = mu[2];  R[2][1] = pc[2];  R[2][2] = mu[0]*pc[1] - mu[1]*pc[0];
  double q[4];
  rotation_matrix_to_quaternion(q, R);

  // 15. (Optional) Use the principal curvatures ratio to make bingham distributions on q1 and q2.

  // 16. correct for camera angle
  tf::Point b_cam = table_camera_transform(tf::Point(ball.x, ball.y, ball.z));
  double v_cam[3] = {b_cam.x(), b_cam.y(), b_cam.z()};
  normalize(v_cam, v_cam, 3);
  double a = acos(v_cam[2]);
  double q_image_to_cam[4] = {cos(a/2.0), -sin(a/2.0)*v_cam[1], sin(a/2.0)*v_cam[0], 0.0};  // v = cross([0,0,1], v_cam);
  quaternion_mult(q, q_image_to_cam, q);

  // 17. rotate into table coordinate frame
  double q_cam_to_table[4] = {0,1,0,0};
  quaternion_mult(ball.oq, q_cam_to_table, q);

  //dbug
  printf("mu = [%.2f, %.2f, %.2f]\n", mu[0], mu[1], mu[2]); //dbug
  double v[3];
  quaternion_to_axis_angle(v, ball.oq);
  printf("observed ball orientation (axis-angle):  [%.2f, %.2f, %.2f]\n", v[0], v[1], v[2]);

  // cleanup
  free_matrix2(P);
  free_matrix2(S);
  free_matrix2(V);

  return true;
}
*/


void flip_quaternion_pcs(double *r, double *q)
{
  r[0] = -q[1];
  r[1] = q[0];
  r[2] = q[3];
  r[3] = -q[2];
}







inline cv::Point2d point_to_left_image(cv::Point3d point)
{
  cv::Point2d pixel = stereo_camera_model.left().project3dToPixel(point);

  return stereo_camera_model.left().unrectifyPoint(pixel);
}

inline cv::Point2d point_to_right_image(cv::Point3d point)
{
  //  printf("baseline = %.3f, Tx = %.3f, Ty = %.3f\n", stereo_camera_model.baseline(),
  //	 stereo_camera_model.right().Tx(), stereo_camera_model.right().Ty()); //dbug

  cv::Point2d pixel = stereo_camera_model.right().project3dToPixel(point);
  pixel.x += stereo_camera_model.right().Tx();
  pixel.y += stereo_camera_model.right().Ty();

  return stereo_camera_model.right().unrectifyPoint(pixel);
}




//------------------- STEREO IMAGE QUEUE ------------------//

// function declarations
void process_stereo_images(const sensor_msgs::Image &left,
			   const sensor_msgs::Image &right); 


bool order_image_stamps(sensor_msgs::Image &left, sensor_msgs::Image &right)
{
  return left.header.stamp < right.header.stamp;
}

void stereo_queue_process(stereo_queue_t &sq)
{
  sq.left.sort(order_image_stamps);
  sq.right.sort(order_image_stamps);

  while (sq.left.size() > 0 && sq.right.size() > 0) {
    if (fabs(sq.left.front().header.stamp.toSec() - sq.right.front().header.stamp.toSec()) < .000001) {
      process_stereo_images(sq.left.front(), sq.right.front());
      sq.left.pop_front();
      sq.right.pop_front();
    }
    else if (order_image_stamps(sq.left.front(), sq.right.front()) && sq.left.size() > 5)
      sq.left.pop_front();
    else if (order_image_stamps(sq.right.front(), sq.left.front()) && sq.right.size() > 5)
    sq.right.pop_front();
    else
      break;
  }

  //printf("SQ %d %d\n", sq.left.size(), sq.right.size());
}


//--------------------- BALL TRACKER ----------------------//

// is there a ball trajectory?
bool found_ball_trajectory(const ball_tracker_t &bt)
{
  for (uint i = 0; i < bt.live_trajectories.size(); i++)
    if (bt.live_trajectories[i].back().w > .5)
      return true;
  return false;
}

double ball_dist(const ball_t &b1, const ball_t &b2)
{
  double dx = b1.x - b2.x;
  double dy = b1.y - b2.y;
  double dz = b1.z - b2.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

//TODO: unrectify
double compute_ball_radius(ball_t &b)
{
  tf::Point p_cam = table_camera_transform(tf::Point(b.x, b.y, b.z));
  cv::Point2d p_left = stereo_camera_model.left().project3dToPixel(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
  cv::Point2d p_left2 =
    stereo_camera_model.left().project3dToPixel(cv::Point3d(p_cam.x() + BALL_RADIUS, p_cam.y(), p_cam.z()));
  //cv::Point2d p_right2 =
  //  stereo_camera_model.right().project3dToPixel(cv::Point3d(p_cam.x + BALL_RADIUS, p_cam.y, p_cam.z));
  double radius_left = hypot(p_left2.x - p_left.x, p_left2.y - p_left.y);
  b.r = radius_left;

  return radius_left;
}

void extract_ball_image(ball_t &b)
{
  // extract left and right ball images
  int w = ceil(3*b.r);
  uchar left_ball_image[w*w]; //, right_ball_image[w*w];
  tf::Point p_cam = table_camera_transform(tf::Point(b.x, b.y, b.z));
  cv::Point2d p1 = point_to_left_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
  cv::Point2d p2 = point_to_right_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
  crop_image(left_ball_image, left_eye.last, left_eye.w, left_eye.h, floor(p1.x - w/2), floor(p1.y - w/2), w, w);
  //crop_image(right_ball_image, right_eye.last, right_eye.w, right_eye.h, floor(p2.x - w/2), floor(p2.y - w/2), w, w);

  // resize ball images to 32x32.
  //uchar B1[32*32], B2[32*32];
  //resize_image(B1, left_ball_image, w, w, 32, 32);
  //resize_image(B2, right_ball_image, w, w, 32, 32);
  resize_image(b.image, left_ball_image, w, w, 32, 32);
}


void improve_ball_fitness(ball_t &b)
{
  //printf("improve_ball_fitness(%.3f, %.3f, %.3f) --> ", b.x, b.y, b.z); //dbug

  // extract left and right ball images
  int w = ceil(3*b.r);
  uchar left_ball_image[w*w], right_ball_image[w*w];
  tf::Point p_cam = table_camera_transform(tf::Point(b.x, b.y, b.z));
  cv::Point2d p1 = point_to_left_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
  cv::Point2d p2 = point_to_right_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
  crop_image(left_ball_image, left_eye.last, left_eye.w, left_eye.h, floor(p1.x - w/2), floor(p1.y - w/2), w, w);
  crop_image(right_ball_image, right_eye.last, right_eye.w, right_eye.h, floor(p2.x - w/2), floor(p2.y - w/2), w, w);

  // resize ball images to 32x32.
  uchar B1[32*32], B2[32*32];
  resize_image(B1, left_ball_image, w, w, 32, 32);
  resize_image(B2, right_ball_image, w, w, 32, 32);

  // use imfilter to compute alignment of ball image w.r.t. avg. ball image.
  int dx1, dy1, dx2, dy2;
  uchar B_avg[1024];
  for (int i = 0; i < 1024; i++)
    B_avg[i] = 256.*logo_detector.B_avg[i];
  compute_image_alignment(dx1, dy1, B1, B_avg, w, w, 5);
  compute_image_alignment(dx2, dy2, B2, B_avg, w, w, 5);
  
  // adjust left and right ball pixel locations
  double c = w/32.;
  cv::Point2d lp = cv::Point2d(p1.x + dx1*c, p1.y + dy1*c);
  cv::Point2d rp = cv::Point2d(p2.x + dx2*c, p2.y + dy2*c);
  //printf("  dx1 = %d, dy1 = %d, dx2 = %d, dy2 = %d, c = %f\n", dx1, dy1, dx2, dy2, c); //dbug
  //printf("  before: lp = (%.1f, %.1f), rp = (%.1f, %.1f)\n", p1.x, p1.y, p2.x, p2.y); //dbug
  //printf("  after:  lp = (%.1f, %.1f), rp = (%.1f, %.1f)\n", lp.x, lp.y, rp.x, rp.y); //dbug
  lp = stereo_camera_model.left().rectifyPoint(lp);
  rp = stereo_camera_model.right().rectifyPoint(rp);


  // recompute ball's 3d position
  double disparity = lp.x - rp.x;
  cv::Point2d p_image = lp;
  cv::Point3d p_cam_cv;
  stereo_camera_model.projectDisparityTo3d(p_image, disparity, p_cam_cv);
  tf::Point p_table = camera_table_transform(tf::Point(p_cam_cv.x, p_cam_cv.y, p_cam_cv.z));
  b.x = p_table.x();
  b.y = p_table.y();
  b.z = p_table.z();
  compute_ball_radius(b);

  //printf("(%.3f, %.3f, %.3f)\n", b.x, b.y, b.z); //dbug
}


vector<ball_t> merge_ball_detections(vector<ball_t> &left_balls, vector<ball_t> &right_balls)
{
  //double x_offset = stereo_camera_model.right().cx() - stereo_camera_model.left().cx();

  // find out how much of the bottom left and right images to black out because of the arm
  double ymax_left = 640;
  double ymax_right = 640;
  if (have_paddle_plan) {
    double dt = .01;  // from brain.cpp
    double t0 = paddle_plan.header.stamp.toSec();
    int n = paddle_plan.points.size();
    int i = round((current_time.toSec() - t0) / dt);
    if (i >= 0 && i < n) {
      double x = paddle_plan.points[i].x;
      double y = paddle_plan.points[i].y;
      double z = paddle_plan.points[i].z;
      tf::Point p_cam = table_camera_transform(tf::Point(x,y,z));
      cv::Point2d p1 = point_to_left_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
      cv::Point2d p2 = point_to_right_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
      ymax_left = p1.y - 100;
      ymax_right = p2.y - 100;
    }
  }

  vector<ball_t> balls;
  for (uint i = 0; i < left_balls.size(); i++) {
    for (uint j = 0; j < right_balls.size(); j++) {
      ball_t &lb = left_balls[i], &rb = right_balls[j];
      cv::Point2d lp = cv::Point2d(lb.x, lb.y);
      cv::Point2d rp = cv::Point2d(rb.x, rb.y);
      lp = stereo_camera_model.left().rectifyPoint(lp);
      rp = stereo_camera_model.right().rectifyPoint(rp);

      if (lp.y > ymax_left || rp.y > ymax_right)
	continue;

      if (fabs(lp.y - rp.y) < ball_stereo_y_thresh && lp.x > rp.x) {
	double disparity = lp.x - rp.x;
	cv::Point2d p_image = lp;
	cv::Point3d p_cam;
	stereo_camera_model.projectDisparityTo3d(p_image, disparity, p_cam);
	tf::Point p_table = camera_table_transform(tf::Point(p_cam.x, p_cam.y, p_cam.z));
	ball_t b;
	memset(&b, 0, sizeof(ball_t));
	b.x = p_table.x();
	b.y = p_table.y();
	b.z = p_table.z();
	b.w = 1.0;
	b.t = lb.t;
	compute_ball_radius(b);
	b.vx = b.vy = b.vz = 0;

	//printf("  lb = (%.1f, %.1f), rb = (%.1f, %.1f)\n", lb.x, lb.y, rb.x, rb.y); //dbug

	//improve_ball_fitness(b);

	if (b.z < BALL_RADIUS)
	  b.z = BALL_RADIUS;

	if (b.y > .1) //dbug
	  balls.push_back(b);
	
	//printf("ball detection %d: pos = (%.1f, %.1f, %.1f), radius = %.1f\n", balls.size(), b.x, b.y, b.z, b.r); //dbug
      }
    }
  }

  return balls;
}


void publish_ball_tracker_state(ball_tracker_t &bt)
{
  pingpong::EyesState msg;
  

  // timestamp
  msg.stamp = current_time;

  // ROI
  msg.left.roi.x = left_eye.roi.x;
  msg.left.roi.y = left_eye.roi.y;
  msg.left.roi.w = left_eye.roi.w;
  msg.left.roi.h = left_eye.roi.h;
  msg.right.roi.x = right_eye.roi.x;
  msg.right.roi.y = right_eye.roi.y;
  msg.right.roi.w = right_eye.roi.w;
  msg.right.roi.h = right_eye.roi.h;

  // table corners
  cv::Point3d corners[4];
  cv::Point2d points[4];
  table_corners_from_transform(corners, table_camera_transform);
  for (int i = 0; i < 4; i++)
    points[i] = point_to_left_image(corners[i]);
  for (int i = 0; i < 4; i++) {
    msg.left.table_corners[i].x = points[i].x;
    msg.left.table_corners[i].y = points[i].y;
  }
  for (int i = 0; i < 4; i++)
    points[i] = point_to_right_image(corners[i]);
  for (int i = 0; i < 4; i++) {
    msg.right.table_corners[i].x = points[i].x;
    msg.right.table_corners[i].y = points[i].y;
  }

  // ball trajectory
  for (uint i = 0; i < bt.live_trajectories.size(); i++) {
    if (bt.live_trajectories[i].back().w > .5) {  // found ball

      // observed ball trajectory
      for (uint j = 0; j < bt.live_trajectories[i].size(); j++) {
	ball_t &b = bt.live_trajectories[i][j];
	tf::Point p_cam = table_camera_transform(tf::Point(b.x, b.y, b.z));

	//dbug
	//tf::Point p_table = camera_table_transform(p_cam);
	//printf("p_table = (%.2f, %.2f, %.2f), b = (%.2f, %.2f, %.2f)\n", p_table.x(), p_table.y(), p_table.z(), b.x, b.y, b.z);

	cv::Point2d p1 = point_to_left_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
	cv::Point2d p2 = point_to_right_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
	pingpong::Circle c1, c2;
	c1.x = p1.x;
	c1.y = p1.y;
	c1.r = b.r;
	msg.left.ball_trajectory.push_back(c1);
	c2.x = p2.x;
	c2.y = p2.y;
	c2.r = b.r;
	msg.right.ball_trajectory.push_back(c2);
      }

      // predicted ball trajectory
      vector<ball_t> future_traj = predict_ball_trajectory(bt.live_trajectories[i].back(), 100, .01);
      for (uint j = 0; j < future_traj.size(); j++) {
	ball_t &b = future_traj[j];
	tf::Point p_cam = table_camera_transform(tf::Point(b.x, b.y, b.z));
	cv::Point2d p1 = point_to_left_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
	cv::Point2d p2 = point_to_right_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
	pingpong::Circle c1, c2;
	c1.x = p1.x;
	c1.y = p1.y;
	c1.r = b.r;
	msg.left.predicted_ball_trajectory.push_back(c1);
	c2.x = p2.x;
	c2.y = p2.y;
	c2.r = b.r;
	msg.right.predicted_ball_trajectory.push_back(c2);
      }

      // current ball position and time
      ball_t &b = bt.live_trajectories[i].back();
      tf::Point ball_pos_cam = table_camera_transform(tf::Point(b.x, b.y, b.z));  //TODO: calculate left/right separately
      msg.ball_stamp = ros::Time(b.t);
      msg.ball_pos_table.x = b.x;
      msg.ball_pos_table.y = b.y;
      msg.ball_pos_table.z = b.z;
      msg.ball_pos_cam.x = ball_pos_cam.x();
      msg.ball_pos_cam.y = ball_pos_cam.y();
      msg.ball_pos_cam.z = ball_pos_cam.z();

      // ball image
      //ball_t &b = bt.live_trajectories[i].back();
      tf::Point p_cam = table_camera_transform(tf::Point(b.x, b.y, b.z));
      cv::Point2d p1 = point_to_left_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
      cv::Point2d p2 = point_to_right_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
      int w = ceil(3*b.r);
      uchar ball_image[w*w];
      sensor_msgs::Image ball_image_msg;
      ball_image_msg.width = w;
      ball_image_msg.height = w;
      ball_image_msg.data.resize(w*w);
      ball_image_msg.encoding = sensor_msgs::image_encodings::MONO8;
      ball_image_msg.header.stamp = current_time;
      // left ball image
      crop_image(ball_image, left_eye.last, left_eye.w, left_eye.h, floor(p1.x - w/2), floor(p1.y - w/2), w, w);
      std::copy(ball_image, ball_image+w*w, ball_image_msg.data.begin());
      msg.left.ball_image = ball_image_msg;  //bt.left_ball_image_pub.publish(ball_image_msg);
      // right right ball image
      crop_image(ball_image, right_eye.last, right_eye.w, right_eye.h, floor(p2.x - w/2), floor(p2.y - w/2), w, w);
      std::copy(ball_image, ball_image+w*w, ball_image_msg.data.begin());
      msg.right.ball_image = ball_image_msg;  //bt.right_ball_image_pub.publish(ball_image_msg);

      break;
    }
  }

  bt.state_pub.publish(msg);
}

void publish_ball_tracker(ball_tracker_t &bt)
{
  //printf("publish_ball_tracker()\n"); //dbug

  // find ball trajectory
  int imax = -1;
  for (uint i = 0; i < bt.live_trajectories.size(); i++) {
    if (bt.live_trajectories[i].back().w > .5) {  // ball traj
      imax = i;
    }
  }

  //printf("  imax = %d\n", imax); //dbug
  //if (imax >= 0)
    //printf("  w = %f\n", bt.live_trajectories[imax].back().w); //dbug

  if (imax < 0 || bt.live_trajectories[imax].back().w < .5)
    return;

  ball_t &b = bt.live_trajectories[imax].back();
  pingpong::BallState msg;
  msg.t = b.t; //left_eye.last_msg.header.stamp.toSec();
  msg.ox = b.ox;
  msg.oy = b.oy;
  msg.oz = b.oz;
  msg.x = b.x;
  msg.y = b.y;
  msg.z = b.z;
  msg.vx = b.vx;
  msg.vy = b.vy;
  msg.vz = b.vz;
  msg.wx = b.wx;
  msg.wy = b.wy;
  msg.wz = b.wz;
  for (int i = 0; i < 81; i++)
    msg.cov[i] = b.cov[i];
  //double spin[4];
  //bingham_mode(spin, &b.dB);
  //msg.sw = spin[0];
  //msg.sx = spin[1];
  //msg.sy = spin[2];
  //msg.sz = spin[3];
  bt.ball_pub.publish(msg);

  if (bt.live_trajectories[imax].size() % 10 == 0)
    printf("  --> published ball (%d)!\n", bt.live_trajectories[imax].size()); //dbug

  /* copy longest live trajectory points into point cloud
  sensor_msgs::PointCloud cloud;
  cloud.header.stamp = left_eye.last_msg.header.stamp;
  cloud.header.frame_id = "pingpongtable";  //"/svstereo";

  sensor_msgs::PointCloud cloud;
  cloud.header.stamp = left_eye.last_msg.header.stamp;
  cloud.header.frame_id = "pingpongtable";  //"/svstereo";

  cloud.points.resize(bt.live_trajectories[imax].size());
  for (uint i = 0; i < bt.live_trajectories[imax].size(); i++) {
    ball_t &bi = bt.live_trajectories[imax][i];
    cloud.points[i].x = bi.ox; //bi.x;
    cloud.points[i].y = bi.oy; //bi.y;
    cloud.points[i].z = bi.oz; //bi.z;
  }

  //sensor_msgs::PointCloud cloud2;
  //tf_listener->transformPointCloud("/pingpongtable", cloud, cloud2);

  bt.trajectory_pub.publish(cloud);
  */

  // publish predicted ball trajectory
  sensor_msgs::PointCloud future_cloud;
  future_cloud.header.stamp = ros::Time(b.t); //left_eye.last_msg.header.stamp;
  future_cloud.header.frame_id = "pingpongtable";
  vector<ball_t> future_traj = predict_ball_trajectory(b, 100, .01); //1);
  future_cloud.points.resize(future_traj.size());
  for (uint i = 0; i < future_traj.size(); i++) {
    ball_t &bi = future_traj[i];
    future_cloud.points[i].x = bi.x;
    future_cloud.points[i].y = bi.y;
    future_cloud.points[i].z = bi.z;
  }

  //printf("ball time = %.3f, current_time = %.3f\n",
  //	 future_cloud.header.stamp.toSec(), ros::Time::now().toSec()); //dbug

  bt.prediction_pub.publish(future_cloud);

#ifdef ICRA_EXP
  //dbug: publish left ball image
  sensor_msgs::Image ball_image_msg;
  int w = ceil(3*b.r);
  ball_image_msg.width = w;
  ball_image_msg.height = w;
  ball_image_msg.step = w;
  ball_image_msg.data.resize(w*w);
  ball_image_msg.encoding = sensor_msgs::image_encodings::MONO8;
  ball_image_msg.header.stamp = ros::Time(b.t); //current_time;
  uchar ball_image[w*w];
  tf::Point p_cam = table_camera_transform(tf::Point(b.ox, b.oy, b.oz));
  cv::Point2d p1 = point_to_left_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
  crop_image(ball_image, left_eye.last, left_eye.w, left_eye.h, floor(p1.x - w/2), floor(p1.y - w/2), w, w);
  std::copy(ball_image, ball_image+w*w, ball_image_msg.data.begin());
  //std::copy(b.image, b.image+w*w, ball_image_msg.data.begin());
  bt.left_ball_image_pub.publish(ball_image_msg);

  icra_ball_images.push_back(ball_image_msg);
  icra_ball_states.push_back(b.t);
  icra_ball_states.push_back(p_cam.x());
  icra_ball_states.push_back(p_cam.y());
  icra_ball_states.push_back(p_cam.z());
  icra_ball_states.push_back(b.x);
  icra_ball_states.push_back(b.y);
  icra_ball_states.push_back(b.z);
#endif
}


bool in_table_region(const ball_t &b)
{
  //printf("in_table_region(b.x = %.3f, b.y = %.3f)\n", b.x, b.y); //dbug

  double x_eps = 0;
  double y_eps = 0;

  return (b.x > x_eps && b.x < TABLE_WIDTH-x_eps && b.y > y_eps && b.y < TABLE_LENGTH-y_eps);
}

bool trajectory_in_table_region(vector<ball_t> &traj)
{
  //printf("trajectory_in_table_region(traj.size = %d)\n", traj.size()); //dbug

  int n = traj.size();
  if (n < ball_trajectory_min_length)
    return false;
  for (int i = n-1; i >= n-ball_trajectory_min_length; i--) {
    //printf("i = %d\n", i); //dbug
    if (!in_table_region(traj[i]))
      return false;
  }
  return true;

  /*
  uint cnt = 0;
  for (uint i = 0; i < traj.size(); i++)
    if (in_table_region(tf::Point(traj[i].x, traj[i].y, traj[i].z)))
      if (++cnt >= 3)
	return true;

  return false;
  */
}

double trajectory_ballistic_score(vector<ball_t> &traj)
{
  uint n = traj.size();
  if (n < 5)
    return 0;
  
  uint p = MIN(n-3, 10);

  // compute velocity (in table coordinates) at the last p trajectory points
  double vx[p], vy[p], vz[p];
  for (uint i = 0; i < p; i++) {
    vx[i] = traj[n-p+i].vx;
    vy[i] = traj[n-p+i].vy;
    vz[i] = traj[n-p+i].vz;
  }

  //double vx_mean = sum(vx,p) / (double)p;
  double vy_mean = sum(vy,p) / (double)p;

  //printf("vx_mean = %.2f, vy_mean = %.2f\n", vx_mean, vy_mean);

  // min velocity check
  if (fabs(vy_mean) < ball_min_velocity)  //(vx_mean*vx_mean + vy_mean*vy_mean < ball_min_velocity*ball_min_velocity)
    return 0;

  // check direction of motion across the table
  if (vy_mean < 0 && traj[0].y < .4*TABLE_LENGTH)
    return 0;
  if (vy_mean > 0 && traj[0].y > .6*TABLE_LENGTH)
    return 0;

  return 1;

  /*
  printf("vx = [");
  for (uint i = 0; i < p; i++)
    printf("%.2f ", vx[i]);
  printf("\n");
  printf("vy = [");
  for (uint i = 0; i < p; i++)
    printf("%.2f ", vy[i]);
  printf("\n");

  double d = 0;
  for (uint i = 0; i < p-1; i++) {
    double dvx = vx[i+1]-vx[i];
    double dvy = vy[i+1]-vy[i];
    d += dvx*dvx + dvy*dvy;
  }
  d /= (double)(p-1);

  double mean_d_given_ball = 1;
  double mean_d_given_notball = 4;

  double c1 = 1/mean_d_given_ball;
  double c2 = 1/mean_d_given_notball;
  double p_d_given_ball = c1*exp(-c1*d);
  double p_d_given_notball = c2*exp(-c2*d);

  double score = p_d_given_ball / (p_d_given_ball + p_d_given_notball);

  printf(" ---> d = %.2f, score = %.2f\n", d, score);

  return score;
  */
}

void classify_live_trajectories(ball_tracker_t &bt)
{
  for (uint i = 0; i < bt.live_trajectories.size(); i++) {
    ball_t &b = bt.live_trajectories[i].back();
    bool on_table = trajectory_in_table_region(bt.live_trajectories[i]);
    if (!on_table) {
      b.w = 0;
      continue;
    }
    double s1 = 1.0; //trajectory_ballistic_score(bt.live_trajectories[i]);
    b.w = s1;
  }
}




void kill_stale_trajectories(ball_tracker_t &bt)
{
  bool found_ball = found_ball_trajectory(bt);
  uint living = 0;
  for (uint i = 0; i < bt.live_trajectories.size(); i++) {
    ball_t &b = bt.live_trajectories[i].back();
    bool is_returning = (b.vy > 0 && b.y > 1.5);
    double delay = current_time.toSec() - b.t;
    bool is_ball = b.w > .5;
    if (is_ball && (delay > ball_trajectory_timeout)) // || (is_returning && (b.y > 2.2 || delay > .02))))
      bt.dead_trajectories.push_back(bt.live_trajectories[i]);
    else if (!is_ball && (found_ball || delay > ball_trajectory_timeout))
      continue;
    else if (living < i)
      bt.live_trajectories[living++] = bt.live_trajectories[i];
    else
      living++;
  }
  bt.live_trajectories.resize(living);

  /* print live trajectories
  for (uint i = 0; i < bt.live_trajectories.size(); i++) {
    ball_t &b = bt.live_trajectories[i].back();
    printf("(%.3f, %.3f, %.3f, w=%.2f) ", b.x, b.y, b.z, b.w);
  }
  printf("\n");
  */
}


void update_ball_trajectories(ball_tracker_t &bt, vector<ball_t> &ball_detections)
{
  if (ball_detections.size() == 0)
    return;

  //printf("==========================\n");

  //fit_circles_to_stereo_ball_detections(ball_detections);

  bool found_ball = found_ball_trajectory(bt);

  int num_traj = MAX(bt.live_trajectories.size(), 1);
  bool assigned[num_traj];
  memset(assigned, 0, num_traj*sizeof(bool));

  // compute predicted ball positions for each live trajectory
  vector<ball_t> predictions;
  for (uint i = 0; i < bt.live_trajectories.size(); i++) {
    ball_t &b = bt.live_trajectories[i].back();
    //double dt = ball_detections[0].t - b.t;
    //vector<ball_t> p = predict_ball_trajectory(b, 1, dt);
    //predictions.push_back(p.back());
    predictions.push_back(b);
  }

  double t0=0, t1=0, t2=0, t3=0;

  // NN search to assign detections to trajectories
  for (uint i = 0; i < ball_detections.size(); i++) {
    ball_t &new_ball = ball_detections[i];
    if (!in_table_region(new_ball))
      continue;

    t0 = get_time_ms(); //dbug

    extract_ball_image(new_ball);

    t1 = get_time_ms(); //dbug

    estimate_ball_orientation_from_logo(new_ball); //new_ball.oq, new_ball.image);

    t2 = get_time_ms(); //dbug

    double dmin = 1000;
    int jmin = -1;
    for (uint j = 0; j < predictions.size(); j++) {
      double d = ball_dist(new_ball, predictions[j]);
      if (d < dmin) {
	dmin = d;
	jmin = j;
      }
    }
    //printf("dmin = %.3f\n", dmin);
    bool is_NN_ball_traj = (jmin >= 0 && bt.live_trajectories[jmin].back().w > .5);
    if ((!is_NN_ball_traj && dmin < trajectory_NN_max_dist) ||
    	(is_NN_ball_traj && dmin < ball_trajectory_NN_max_dist)) {
    //if (dmin < ball_trajectory_NN_max_dist) {

      //printf("before filter: new_ball: pos=(%.3f, %.3f, %.3f), vel=(%.3f, %.3f, %.3f)\n",
      //     new_ball.x, new_ball.y, new_ball.z, new_ball.vx, new_ball.vy, new_ball.vz);

      filter_ball(new_ball, bt.live_trajectories[jmin].back());

      //printf("after filter: new_ball: pos=(%.3f, %.3f, %.3f), vel=(%.3f, %.3f, %.3f)\n",
      //     new_ball.x, new_ball.y, new_ball.z, new_ball.vx, new_ball.vy, new_ball.vz);

      if (!assigned[jmin]) {  // add to existing trajectory
	bt.live_trajectories[jmin].push_back(new_ball);
	assigned[jmin] = true;
      }
      else {  // clone existing trajectory
	printf(" --> cloning!\n");
	vector<ball_t> new_trajectory = bt.live_trajectories[jmin];
	new_trajectory.pop_back();
	new_trajectory.push_back(new_ball);
	bt.live_trajectories.push_back(new_trajectory);
      }
    }
    else if (!found_ball) {  // create new trajectory
      if (player_rh_vel_time > current_time.toSec() - 0.1) {
	if (player_rh_vel[2] > 0.1) { //0.1  // topspin
	  new_ball.wx = 1.2 / .005;
	  //new_ball.wy = .2*player_rh_vel[0] / .005;
	}
 	else {  // underspin
	  new_ball.wx = -1.2 / .005;
	  new_ball.wy = -.4*player_rh_vel[0] / .005;
	}	
      }
      filter_ball_init(new_ball);
      vector<ball_t> new_trajectory;
      new_trajectory.push_back(new_ball);
      bt.live_trajectories.push_back(new_trajectory);
    }

    t3 = get_time_ms(); //dbug
  }

  //dbug
  if (t3-t0 > 10)
    printf("Warning: update_ball_trajectories() took %.1f ms (%.1f, %.1f, %.1f)\n", t3-t0, t1-t0, t2-t1, t3-t2);
}


void update_ball_tracker(ball_tracker_t &bt)
{
  kill_stale_trajectories(bt);
  vector<ball_t> ball_detections = merge_ball_detections(left_eye.ball_detections, right_eye.ball_detections);
  update_ball_trajectories(bt, ball_detections);
  classify_live_trajectories(bt);
}



//--------------------- IMAGE PROCESSING ------------------//

void threshold_image(uchar *ball, const sensor_msgs::Image &msg, eye_memory_t &eye)
{
  //double t0 = ros::Time::now().toSec();  //dbug
    
  int w = eye.w;
  int rx = eye.roi.x;
  int ry = eye.roi.y;
  int rw = eye.roi.w;
  int rh = eye.roi.h;

  int xmax = rx+rw;
  int ymax = ry+rh;

  for (int y = ry; y < ymax; y++)
    std::copy(msg.data.begin()+y*w+rx, msg.data.begin()+y*w+rx+rw, ball+(y-ry)*rw);

  uchar *last = eye.last, *bg = eye.bg;
  int cnt = 0;
  for (int y = ry; y < ymax; y++)
    for (int x = rx; x < xmax; x+=16, cnt+=16) {
      int i = y*w+x;
      __m128i a = _mm_load_si128((__m128i *)(ball+cnt));
      __m128i b = _mm_load_si128((__m128i *)(last+i));
      __m128i c = _mm_load_si128((__m128i *)(bg+i));
      __m128i d = _mm_subs_epu8(a, b);
      b = _mm_subs_epu8(a, c);
      a = _mm_adds_epu8(b, d);  // a = ball[cnt] - last[i] + ball[cnt] - bg[i];
      b = _mm_set1_epi8(ball_thresh);
      c = _mm_max_epu8(a, b);
      b = _mm_cmpeq_epi8(a, c);  // b = (a > ball_thresh ? 255 : 0);
      _mm_store_si128((__m128i *)(ball+cnt), b);  // ball[cnt] = b;
    }

  //printf("Processed ROI in %f sec.\n", ros::Time::now().toSec() - t0);  //dbug
}


void process_image(const sensor_msgs::Image &msg, eye_memory_t &eye, bool first)
{
  // update system time
  if (msg.header.stamp > current_time) {
    //printf("dt = %.3f sec\n", msg.header.stamp.toSec() - current_time.toSec()); //dbug
    current_time = msg.header.stamp;
  }

  if (first) {
    first = false;
    eye.w = msg.width;
    eye.h = msg.height;
    eye.roi.x = 0;
    eye.roi.y = 0;
    eye.roi.w = eye.w;
    eye.roi.h = eye.h;
    eye.last_msg = msg;
    //eye.bg = new uchar[eye.w * eye.h];
    //eye.last = new uchar[eye.w * eye.h];

    if (msg.data.size() != eye.w*eye.h) {
      ROS_ERROR("msg.data.size() != eye.w*eye.h");
      exit(1);
    }

    // copy image into eye.bg and eye.last
    for (uint i = 0; i < msg.data.size(); i++)
      eye.bg[i] = eye.last[i] = msg.data[i];

    return;
  }

  //uint n = msg.data.size();
  //int w = eye.w;
  //int h = eye.h;
  int rx = eye.roi.x;
  int ry = eye.roi.y;
  int rw = eye.roi.w;
  int rh = eye.roi.h;
  int rn = rw*rh;

  uchar ball[rn] __attribute__ ((aligned (16)));
  threshold_image(ball, msg, eye);

  /*dbug
  FILE *fp = fopen("ball.m", "w");
  fprintf(fp, "ball_image = [");
  for (int i = 0; i < rh; i++) {
    for (int j = 0; j < rw; j++)
      fprintf(fp, "%u,", ball[i*rw+j]);
    fprintf(fp, "; ...\n");
  }
  fprintf(fp, "];\n");
  fclose(fp);
  */

  //double t0 = ros::Time::now().toSec();  //dbug
  connected_components_t CC;
  int num_comp = connected_components_sse(CC, ball, rw, rh);
  //printf("Computed connected components (SSE) in %f sec.\n", ros::Time::now().toSec() - t0);  //dbug

  // get balls within a range of sizes
  eye.ball_detections.resize(0);
  for (int c = 0; c < num_comp; c++) {
    int cnum = CC.cluster_size[c];
    if (cnum >= ball_min_size && cnum <= ball_max_size) {
      ball_t b;
      b.t = current_time.toSec();
      b.x = CC.cluster_x[c] + rx;
      b.y = CC.cluster_y[c] + ry;
      eye.ball_detections.push_back(b);
    }
  }

  //dbug
  //printf("CC = [");
  //for (int i = 0; i < CC.cluster_x.size(); i++)
  //  printf("%.1f,%.1f,%d; ", CC.cluster_x[i], CC.cluster_y[i], CC.cluster_size[i]);
  //printf("];\n");

  //t0 = ros::Time::now().toSec();  //dbug

  // update previous image & background image
  if (publish_images)
    eye.last_msg = msg;
  else
    eye.last_msg.header.stamp = msg.header.stamp;

  std::copy(msg.data.begin(), msg.data.end(), eye.last);
  uint n = eye.w*eye.h;
  for (uint i = 0; i < n; i+=16) {  // assuming eye.w is a multiple of 16
    __m128i a = _mm_load_si128((__m128i *)(eye.last+i));
    __m128i b = _mm_load_si128((__m128i *)(eye.bg+i));
    __m128i c = _mm_avg_epu8(_mm_avg_epu8(a,b), b);  // bg[i] = .75*bg[i] + .25*last[i]
    _mm_store_si128((__m128i *)(eye.bg+i), c);  // ball[cnt] = b;
  }

  //printf("Updated previous and background images in %f sec.\n", ros::Time::now().toSec() - t0);  //dbug
}

void broadcast_table(ros::Time stamp)
{
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(table_camera_transform, stamp, "/svstereo", "/pingpongtable"));
}

void update_image_roi()
{
  uint roi_padding = 80;  // pixels

  // is there a ball trajectory?
  bool found_ball = found_ball_trajectory(ball_tracker);

  rect_t max_roi_left, max_roi_right;
  max_roi_left.x = 0;
  max_roi_left.y = 0;
  max_roi_left.w = left_eye.w;
  max_roi_left.h = left_eye.h;
  max_roi_right.x = 0;
  max_roi_right.y = 0;
  max_roi_right.w = right_eye.w;
  max_roi_right.h = right_eye.h;

  vector<cv::Point2d> left_points, right_points;
  if (!found_ball) {
    /*
    int n = 8;
    tf::Point corners[n];  // z=0 : (NW, NE, SE, SW), z=1 : (NW, NE, SE, SW)
    corners[0] = table_camera_transform(tf::Point(0, TABLE_LENGTH, 0));
    corners[1] = table_camera_transform(tf::Point(TABLE_WIDTH, TABLE_LENGTH, 0));
    corners[2] = table_camera_transform(tf::Point(TABLE_WIDTH, 0, 0));
    corners[3] = table_camera_transform(tf::Point(0, 0, 0));
    corners[4] = table_camera_transform(tf::Point(0, TABLE_LENGTH, .5));
    corners[5] = table_camera_transform(tf::Point(TABLE_WIDTH, TABLE_LENGTH, .5));
    corners[6] = table_camera_transform(tf::Point(TABLE_WIDTH, 0, .5));
    corners[7] = table_camera_transform(tf::Point(0, 0, .5));
    */
    int n = 4;
    tf::Point corners[n];  // z=0 : (NW, NE, SE, SW)
    corners[0] = table_camera_transform(tf::Point(.2*TABLE_WIDTH, .8*TABLE_LENGTH, 0));
    corners[1] = table_camera_transform(tf::Point(.8*TABLE_WIDTH, .8*TABLE_LENGTH, 0));
    corners[2] = table_camera_transform(tf::Point(.8*TABLE_WIDTH, .2*TABLE_LENGTH, 0));
    corners[3] = table_camera_transform(tf::Point(.2*TABLE_WIDTH, .2*TABLE_LENGTH, 0));

    for (uint i = 0; i < n; i++) {
      tf::Point p_cam = corners[i];
      cv::Point2d p1 = point_to_left_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
      cv::Point2d p2 = point_to_right_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
      left_points.push_back(p1);
      right_points.push_back(p2);
    }
  }
  else {
    for (uint i = 0; i < ball_tracker.live_trajectories.size(); i++) {
      ball_t &b = ball_tracker.live_trajectories[i].back();
      bool is_ball = (b.w > .5);
      if (is_ball) {
	tf::Point p_cam = table_camera_transform(tf::Point(b.x, b.y, b.z));
	cv::Point2d p1 = point_to_left_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
	cv::Point2d p2 = point_to_right_image(cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));

	//printf("ball left image pos: (%.0f, %.0f), right image pos: (%.0f, %.0f)\n", p1.x, p1.y, p2.x, p2.y); //dbug

	left_points.push_back(p1);
	right_points.push_back(p2);
      }
    }	
  }

  left_eye.roi = roi_from_points(left_points, roi_padding, max_roi_left);
  right_eye.roi = roi_from_points(right_points, roi_padding, max_roi_right);

  // align rois with 16-byte boundaries (for vectorized CPU optimizations)
  int block = 16;
  left_eye.roi.x = block*(left_eye.roi.x/block);
  left_eye.roi.y = block*(left_eye.roi.y/block);
  left_eye.roi.w = block*(left_eye.roi.w/block);
  left_eye.roi.h = block*(left_eye.roi.h/block);

  right_eye.roi.x = block*(right_eye.roi.x/block);
  right_eye.roi.y = block*(right_eye.roi.y/block);
  right_eye.roi.w = block*(right_eye.roi.w/block);
  right_eye.roi.h = block*(right_eye.roi.h/block);

  /*dbug
  if (left_eye.roi.w < 200)
    printf("left roi = (%d, %d, %d, %d), right roi = (%d, %d, %d, %d)\n",
	   left_eye.roi.x, left_eye.roi.y, left_eye.roi.w, left_eye.roi.h,
	   right_eye.roi.x, right_eye.roi.y, right_eye.roi.w, right_eye.roi.h);
  */
}

void process_stereo_images(const sensor_msgs::Image &left,
			   const sensor_msgs::Image &right)
{
  //cout << "============================" << endl;

  /*
  static int cnt = 0;
  if (cnt < 100) {
    if (cnt%10==0) {
      printf("fit_table()\n"); //dbug
      fit_table(table_camera_transform, left, right, stereo_camera_model);
      camera_table_transform = table_camera_transform.inverse();
    }
    cnt++;
  }
  */

  // make sure its a new pair of images!
  if (left.header.stamp.toSec() < current_time.toSec() + .001)
    return;

  // broadcast table-camera coordinate transform
  broadcast_table(left.header.stamp);
  
  //dbug
  //double dt = left.header.stamp.toSec() - current_time.toSec();
  //if (dt > .015)
  //  printf("Warning: dt = %f\n", dt);

  //printf("image stamp = %f\n", left.header.stamp.toSec()); //dbug

  double t0 = ros::Time::now().toSec();  //dbug

  // do basic image processing on each image
  static bool first = true;
  process_image(left, left_eye, first);
  process_image(right, right_eye, first);
  first = false;

  double t1 = ros::Time::now().toSec();  //dbug

  //printf("Processed images in %f sec.\n", ros::Time::now().toSec() - t0);  //dbug
  //t0 = ros::Time::now().toSec();  //dbug

  // update ball tracker based on eye memories
  update_ball_tracker(ball_tracker);

  // update image ROIs
  update_image_roi();

  double t2 = ros::Time::now().toSec();  //dbug

  /* draw live trajectories
  if (publish_images) {
    static double last_draw_time = 0;
    double now = ros::Time::now().toSec();
    if (now - last_draw_time > draw_rate) {
      draw_ball_tracker(ball_tracker);
      last_draw_time = now;
    }
  }
  */

  if (publish_state) {
    static double last_publish_state_time = 0;
    double now = ros::Time::now().toSec();
    if (now - last_publish_state_time > publish_state_rate) {
      publish_ball_tracker_state(ball_tracker);
      last_publish_state_time = now;
    }
  }

  double t3 = ros::Time::now().toSec();  //dbug
   
  // publish ball tracker
  publish_ball_tracker(ball_tracker);

  double t4 = ros::Time::now().toSec();  //dbug

  //dbug
  if (t4-t0 > .015)
    printf("Warning: process_stereo_images() took %.3f sec. (%.3f, %.3f, %.3f, %.3f)\n", t4-t0, t1-t0, t2-t1, t3-t2, t4-t3);

  //printf("Processed stereo images in %f sec.\n", ros::Time::now().toSec() - t0);  //dbug
}



//---------------------- CALLBACKS -----------------//

void left_image_callback(const sensor_msgs::Image &msg)
{
  //cout << "L " << msg.header.stamp << endl;
  
  stereo_queue.left.push_back(msg);
  stereo_queue_process(stereo_queue);
}

void right_image_callback(const sensor_msgs::Image &msg)
{
  //cout << "R " << msg.header.stamp << endl;

  stereo_queue.right.push_back(msg);
  stereo_queue_process(stereo_queue);
}


void right_hand_velocity_callback(const geometry_msgs::PointStamped &msg)
{
  player_rh_vel[0] = msg.point.x;
  player_rh_vel[1] = msg.point.y;
  player_rh_vel[2] = msg.point.z;
  player_rh_vel_time = msg.header.stamp.toSec();

  printf("Got right hand velocity message %.3f sec. ago: (%.2f, %.2f, %.2f)\n",
  	 current_time.toSec() - msg.header.stamp.toSec(), msg.point.x, msg.point.y, msg.point.z); //dbug
}


void paddle_plan_callback(const sensor_msgs::PointCloud &msg)
{
  paddle_plan = msg;
  have_paddle_plan = true;
}


//---------------------- GET CAMERA MODEL --------------------//

void get_camera_model()
{
  ROS_INFO("Waiting for camera model...");

  sensor_msgs::CameraInfoConstPtr cinfo_left_ptr =
    ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/svstereo/left/camera_info");

  sensor_msgs::CameraInfoConstPtr cinfo_right_ptr =
    ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/svstereo/right/camera_info");

  stereo_camera_model.fromCameraInfo(cinfo_left_ptr, cinfo_right_ptr);

  ROS_INFO("Got camera model.");
}


void get_table()
{
  table_camera_transform =
    tf::Transform(tf::Quaternion(0.999951,-0.009017,-0.002392,0.003181), tf::Point(-0.516968,1.059134,1.808742));
  //tf::Transform(tf::Quaternion(0.999915,-0.011315,-0.004410,0.004755), tf::Point(-0.515778,1.060738,1.788701));
  //tf::Transform(tf::Quaternion(0.999970,-0.006918,-0.001725,0.002961), tf::Point(-0.540300,1.064571,1.779986));
    //tf::Transform(tf::Quaternion(0.999994,-0.002821,0.000909,0.002013), tf::Point(-0.556238,1.048042,1.804106));
    //tf::Transform(tf::Quaternion(0.999989,-0.001008,0.004013,0.002277), tf::Point(-0.553282,1.039629,1.776915));
    //tf::Transform(tf::Quaternion(0.999973,-0.006779,0.002474,0.001569), tf::Point(-0.537627,1.048767,1.792372));
    //tf::Transform(tf::Quaternion(0.999954,-0.001743,-0.007902,-0.005179), tf::Point(-0.553724,1.053008,1.821833));
    //tf::Transform(tf::Quaternion(0.999988,-0.003173,-0.002212,-0.003110), tf::Point(-0.552538,1.059923,1.776718));
  //tf::Transform(tf::Quaternion(0.999975,-0.006711,-0.000696,0.002034), tf::Point(-0.538369,1.067960,1.814124));
  camera_table_transform = table_camera_transform.inverse();
}




/*
void load_avg_ball_image(std::string &data_folder)
{
  char file[1024];
  sprintf(file, "%s/avg_ball_image.txt", data_folder.c_str());

  int n,m;
  double **X = load_matrix(file, &n, &m);

  if (X==NULL) {
    printf("ERROR: Failed to load avg ball image file from %s\n", file);
    exit(1);
  }
  if (n!=32 || m!=32) {
    printf("ERROR: avg ball image has the wrong dimensions\n");
    exit(1);
  }

  for (int i = 0; i < 32*32; i++)
    B_avg[i] = X[0][i];

  free_matrix2(X);
}
*/

/*
void get_table()
{

  //double table_min_depth = 3.0;               // meters
  //double table_max_depth = 8.0;               // meters
  //double min_disparity = stereo_camera_model.getDisparity(table_max_depth);
  //double max_disparity = stereo_camera_model.getDisparity(table_min_depth);
  //printf("disparity range = [%.2f, %.2f]\n", min_disparity, max_disparity);

  // get table corners in 3D camera coordinates
  for (uint i = 0; i < 4; i++) {
    double x = table_corners_left[i][0];
    double y = (table_corners_left[i][1] + table_corners_right[i][1]) / 2.0;
    double disparity = table_corners_left[i][0] - table_corners_right[i][0];
    cv::Point2d p_image(x, y);
    stereo_camera_model.projectDisparityTo3d(p_image, disparity, table_corners[i]);
  }

  // get table coordinate frame (origin is at SW corner, X is east, Y is north, and Z is up)
  cv::Point3d &NW = table_corners[0];
  //cv::Point3d &NE = table_corners[1];
  cv::Point3d &SE = table_corners[2];
  cv::Point3d &SW = table_corners[3];
  table_camera_transform.setOrigin(tf::Vector3(SW.x, SW.y, SW.z));
  double **R = new_matrix2(3,3);
  R[0][0] = SE.x - SW.x;
  R[1][0] = SE.y - SW.y;
  R[2][0] = SE.z - SW.z;
  R[0][1] = NW.x - SW.x;
  R[1][1] = NW.y - SW.y;
  R[2][1] = NW.z - SW.z;
  R[0][2] = R[1][0]*R[2][1] - R[2][0]*R[1][1];
  R[1][2] = R[2][0]*R[0][1] - R[0][0]*R[2][1];;
  R[2][2] = R[0][0]*R[1][1] - R[1][0]*R[0][1];;
  double d[3];
  for (uint i = 0; i < 3; i++)
    d[i] = sqrt(R[0][i]*R[0][i] + R[1][i]*R[1][i] + R[2][i]*R[2][i]);
  for (uint i = 0; i < 3; i++)
    for (uint j = 0; j < 3; j++)
      R[i][j] /= d[j];
  double q[4];
  rotation_matrix_to_quaternion(q, R);
  free_matrix2(R);
  table_camera_transform.setRotation(tf::Quaternion(q[1], q[2], q[3], q[0]));

  camera_table_transform = table_camera_transform.inverse();
}
*/

/*
void test_connected_components()
{
  sensor_msgs::Image C, I;
  I.width = 5;
  I.height = 3;
  I.data.resize(I.width*I.height);
  rect_t roi;
  roi.x = 0;
  roi.y = 0;
  roi.w = I.width;
  roi.h = I.height;

  int pixels[10][2] = {{0,0}, {0,2}, {0,4}, {1,0}, {1,1}, {1,2}, {1,4}, {2,2}, {2,3}, {2,4}};
  for (int i = 0; i < 10; i++)
    I.data[pixels[i][0]*I.width + pixels[i][1]] = 10+i;

  connected_components(C, I, roi);
}
*/



#ifdef ICRA_EXP

void save_image(char *filename, const sensor_msgs::Image &image_msg)
{
  static sensor_msgs::CvBridge img_bridge;

  //printf("w = %d, h = %d\n", image_msg.width, image_msg.height); //dbug

  if (!img_bridge.fromImage(image_msg, "bgr8"))
    ROS_ERROR("Unable to convert %s image to bgr8", image_msg.encoding.c_str());
  IplImage *image = img_bridge.toIpl();
  if (image) {
    cvSaveImage(filename, image);
    ROS_INFO("Saved image %s", filename);
  }
}

void sigint_handler(int signum)
{
  // save ball images
  int n = icra_ball_images.size();
  for (int i = 0; i < n; i++) {
    char filename[128];
    sprintf(filename, "left%04d.png", i);
    save_image(filename, icra_ball_images[i]);
    //sprintf(filename, "right%04d.png", cnt);
    //save_image(filename, msg->right.ball_image);
  }

  printf("Opening ball_state.txt...");
  FILE *ball_state_fp = fopen("ball_state.txt", "w");
  if (ball_state_fp == NULL) {
    ROS_ERROR("Failed to open ball_state.txt.\n");
    exit(1);
  }
  printf("done.\n");

  // save ball timestamp and positions
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < 7; j++)
      fprintf(ball_state_fp, "%f ", icra_ball_states[i*7+j]);
    fprintf(ball_state_fp, "\n");
  }

  printf("Closing ball_state...");
  fclose(ball_state_fp);
  printf("done.\n");

  exit(0);
}

#endif


//----------------------- MAIN --------------------//

int main(int argc, char *argv[])
{
  //test_connected_components();  //dbug

  // init libbingham
  bingham_init();

  // init ROS
  ros::init(argc, argv, "pingpongeyes");
  ros::NodeHandle nh;

  std::string data_folder;
  if (nh.getParam("/eyes/data_folder", data_folder))
    ROS_INFO("Got param 'data_folder': %s", data_folder.c_str());
  else
    ROS_ERROR("Failed to get param 'data_folder'");

#ifdef ICRA_EXP
  signal(SIGINT, sigint_handler);
#endif

  get_camera_model();
  get_table();
  logo_detector = load_logo_detector(data_folder);

  ros::Subscriber sub_left_image = nh.subscribe("/svstereo/left/image_raw", 3, left_image_callback);
  //ros::Subscriber sub_left_image = nh.subscribe("/svstereo/left/image_rect", 10, left_image_callback);
  //left_eye.pub[0] = nh.advertise<sensor_msgs::Image>("/svstereo/left/eyes", 1);
  //left_eye.pub[1] = nh.advertise<sensor_msgs::Image>("/svstereo/left/eyes1", 1);
  //left_eye.pub[2] = nh.advertise<sensor_msgs::Image>("/svstereo/left/eyes2", 1);

  ros::Subscriber sub_right_image = nh.subscribe("/svstereo/right/image_raw", 3, right_image_callback);
  //ros::Subscriber sub_right_image = nh.subscribe("/svstereo/right/image_rect", 10, right_image_callback);
  //right_eye.pub[0] = nh.advertise<sensor_msgs::Image>("/svstereo/right/eyes", 1);
  //right_eye.pub[1] = nh.advertise<sensor_msgs::Image>("/svstereo/right/eyes1", 1);
  //right_eye.pub[2] = nh.advertise<sensor_msgs::Image>("/svstereo/right/eyes2", 1);

  ros::Subscriber sub_rh_vel = nh.subscribe("right_hand_velocity", 1, right_hand_velocity_callback);
  ros::Subscriber sub_paddle_plan = nh.subscribe("paddle_plan", 1, paddle_plan_callback);

  ball_tracker.ball_pub = nh.advertise<pingpong::BallState>("ball_state", 10);
  ball_tracker.state_pub = nh.advertise<pingpong::EyesState>("eyes_state", 1);
  ball_tracker.trajectory_pub = nh.advertise<sensor_msgs::PointCloud>("ball_trajectory", 1);
  ball_tracker.prediction_pub = nh.advertise<sensor_msgs::PointCloud>("ball_trajectory_prediction", 1);
  ball_tracker.left_ball_image_pub = nh.advertise<sensor_msgs::Image>("/svstereo/left/ball_image", 10);
  //ball_tracker.right_ball_image_pub = nh.advertise<sensor_msgs::Image>("/svstereo/right/ball_image", 10);
  //ball_tracker.side = OPPONENT_SIDE;

  //tf_listener = new tf::TransformListener;
  
  ros::spin();

  return 0;
}








//---------------------------  UNUSED  -------------------------//


/*
 *   +
 *  +++  
 *0+++++0
 *  +++
 *   +
 *
 */


/*
double ball_image_fitness(sensor_msgs::Image &I, double x, double y, double r)
{
  // cut out ball image & normalize
  uint r_pixels = 2*ceil(r/2);  // round up to the nearest even integer
  uint w = 3*r_pixels + 1;      // width of ball image (always odd)
  uint x0 = 3*r_pixels/2;       // center pixel (x-coordinate) of ball image
  uint y0 = 3*r_pixels/2;       // center pixel (y-coordinate) of ball image
  double ball_image[w][w];
  double mu=0, sigma=0;
  for (uint yi = 0; yi < w; yi++) {
    for (uint xi = 0; xi < w; xi++) {
      uint yi2 = y - y0 + yi;
      uint xi2 = x - x0 + xi;
      ball_image[yi][xi] = I.data[yi2*I.width + xi2];
      mu += ball_image[yi][xi];
    }
  }
  mu /= (double)(w*w);
  for (uint yi = 0; yi < w; yi++) {
    for (uint xi = 0; xi < w; xi++) {
      double d = ball_image[yi][xi] - mu;
      sigma += d*d;
    }
  }
  sigma = sqrt(sigma / (double)(w*w));
  for (uint yi = 0; yi < w; yi++)
    for (uint xi = 0; xi < w; xi++)
      ball_image[yi][xi] = (ball_image[yi][xi] - mu) / sigma;
  
  // compute sum of all normalized ball image pixels inside of a circle of radius r
  // (equivalent to dot product b/w the normalized ball image and a disk of radius r)
  double score = 0;
  int cnt = 0;
  for (uint yi = 0; yi < w; yi++) {
    for (uint xi = 0; xi < w; xi++) {
      double dy = y0 - yi;
      double dx = x0 - xi;
      if (dx*dx + dy*dy < r*r) {
	score += ball_image[yi][xi];
	cnt++;
      }
    }
  }
  score /= (double)cnt;

  return score;
}

double ball_image_fitness2(sensor_msgs::Image &I, double x, double y, double r)
{
  // cut out ball image
  int r_pixels = 2*ceil(r/2);  // round up to the nearest even integer
  int w = 3*r_pixels + 1;      // width of ball image (always odd)
  int x0 = 3*r_pixels/2;       // center pixel (x-coordinate) of ball image
  int y0 = 3*r_pixels/2;       // center pixel (y-coordinate) of ball image
  double ball_image[w][w];
  for (int yi = 0; yi < w; yi++) {
    for (int xi = 0; xi < w; xi++) {
      int yi2 = y - y0 + yi;
      int xi2 = x - x0 + xi;
      ball_image[yi][xi] = I.data[yi2*I.width + xi2];
    }
  }
  
  // compute sum of all ball image pixels inside of a circle of radius r
  // (equivalent to dot product b/w the ball image and a disk of radius r)
  //printf("===== x0=%d, y0=%d, r=%.2f (%d) =====\n", x0, y0, r, r_pixels);
  double score = 0;
  int cnt = 0;
  for (int yi = 0; yi < w; yi++) {
    for (int xi = 0; xi < w; xi++) {
      double dy = y0 - yi;
      double dx = x0 - xi;
      if (dx*dx + dy*dy < r*r) {
	score += ball_image[yi][xi];
	cnt++;
	//printf("x");
      }
      //else
      //	printf(".");
    }
    //printf("\n");
  }
  //printf("========== cnt=%d ==========\n\n", cnt);
  //score /= (double)cnt;

  return score;
}

// Assumes (b.x, b.y) are in image coordinates
double fit_ball_image(sensor_msgs::Image &I, ball_t &b, double r)
{
  double max_score = 0;
  int best_dx = 0;
  int best_dy = 0;

  int window_radius = ceil(r);
  int window_step = ceil(window_radius / 5.0);

  for (int dx = -window_radius; dx <= window_radius; dx += window_step) {
    for (int dy = -window_radius; dy <= window_radius; dy += window_step) {
      double score = ball_image_fitness2(I, b.x + dx, b.y + dy, r);
      if (score > max_score) {
	max_score = score;
	best_dx = dx;
	best_dy = dy;
      }
    }
  }

  b.x += best_dx;
  b.y += best_dy;

  return max_score;
}
*/

/*
double fit_ball_stereo2(sensor_msgs::Image &left_image, sensor_msgs::Image &right_image, ball_t &b)
{
  cv::Point2d p_left = stereo_camera_model.left().project3dToPixel(cv::Point3d(b.x, b.y, b.z));
  cv::Point2d p_right = stereo_camera_model.right().project3dToPixel(cv::Point3d(b.x, b.y, b.z));
  cv::Point2d p_left2 =
    stereo_camera_model.left().project3dToPixel(cv::Point3d(b.x + BALL_RADIUS, b.y, b.z));
  cv::Point2d p_right2 =
    stereo_camera_model.right().project3dToPixel(cv::Point3d(b.x + BALL_RADIUS, b.y, b.z));
  double radius_left = hypot(p_left2.x - p_left.x, p_left2.y - p_left.y);
  double radius_right = hypot(p_right2.x - p_right.x, p_right2.y - p_right.y);
 
  // fit left ball
  ball_t b_left = b;
  b_left.x = round(p_left.x);
  b_left.y = round(p_left.y);
  double score_left = fit_ball_image(left_image, b_left, radius_left);

  // fit right ball
  ball_t b_right = b;
  b_right.x = round(p_right.x);
  b_right.y = round(p_right.y);
  double score_right = fit_ball_image(right_image, b_right, radius_right);

  double disparity = b_left.x - b_right.x;
  //printf("disparity = %.3f\n", disparity); //dbug
  cv::Point2d p_image_left(b_left.x, b_left.y);
  cv::Point3d new_ball_pos;
  stereo_camera_model.projectDisparityTo3d(p_image_left, disparity, new_ball_pos);
  b.x = new_ball_pos.x;
  b.y = new_ball_pos.y;
  b.z = new_ball_pos.z;
  b.r = radius_left;
  b.lx = b_left.x;
  b.ly = b_left.y;
  b.rx = b_right.x;
  b.ry = b_right.y;

  return score_left + score_right;
}

double fit_ball_stereo(sensor_msgs::Image &left_image, sensor_msgs::Image &right_image, ball_t &b)
{
  cv::Point2d p_left = stereo_camera_model.left().project3dToPixel(cv::Point3d(b.x, b.y, b.z));
  cv::Point2d p_right = stereo_camera_model.right().project3dToPixel(cv::Point3d(b.x, b.y, b.z));
 
  cv::Point2d p_left2 =
    stereo_camera_model.left().project3dToPixel(cv::Point3d(b.x + BALL_RADIUS, b.y, b.z));
  cv::Point2d p_right2 =
    stereo_camera_model.right().project3dToPixel(cv::Point3d(b.x + BALL_RADIUS, b.y, b.z));
  double radius_left = hypot(p_left2.x - p_left.x, p_left2.y - p_left.y);
  double radius_right = hypot(p_right2.x - p_right.x, p_right2.y - p_right.y);

  double max_score = 0;
  int best_dx = 0;
  int best_dy = 0;
  int x0_left = p_left.x;
  int y0_left = p_left.y;
  int x0_right = p_right.x;
  int y0_right = p_right.y;

  for (int dx = -5; dx <= 5; dx++) {
    for (int dy = -5; dy <= 5; dy++) {
      double left_score = ball_image_fitness2(left_image, p_left.x + dx, p_left.y + dy, radius_left);
      double right_score = ball_image_fitness2(right_image, p_right.x + dx, p_right.y + dy, radius_right);
      double score = left_score + right_score;
      if (score > max_score) {
	max_score = score;
	best_dx = dx;
	best_dy = dy;
      }
    }
  }
 
  //printf("radii = [%f, %f], score = %.2f\n", radius_left, radius_right, max_score);

  double disparity = x0_left - x0_right;
  cv::Point2d p_image_left(x0_left + best_dx, y0_left + best_dy);
  cv::Point3d new_ball_pos;
  stereo_camera_model.projectDisparityTo3d(p_image_left, disparity, new_ball_pos);
  b.x = new_ball_pos.x;
  b.y = new_ball_pos.y;
  b.z = new_ball_pos.z;
  b.r = radius_left;
  b.lx = x0_left + best_dx;
  b.ly = y0_left + best_dy;
  b.rx = x0_right + best_dx;
  b.ry = y0_right + best_dy;

  return max_score;
}

void fit_circles_to_stereo_ball_detections(vector<ball_t> &balls)
{
  //printf("circle scores = [");
  for (uint i = 0; i < balls.size(); i++) {
    double score = fit_ball_stereo2(left_eye.change, right_eye.change, balls[i]);
    balls[i].w = score;
    balls[i].circle_score = score;
    //printf("%.2f, ", score);
  }
  //printf("]\n");
}
*/


/*
double trajectory_median_z_accel(vector<ball_t> &traj)
{
  uint n = traj.size()-2;
  double z_accel[n];
  for (uint i = 0; i < n; i++) {
    tf::Point p0 = camera_table_transform(tf::Point(traj[i].x, traj[i].y, traj[i].z));
    tf::Point p1 = camera_table_transform(tf::Point(traj[i+1].x, traj[i+1].y, traj[i+1].z));
    tf::Point p2 = camera_table_transform(tf::Point(traj[i+2].x, traj[i+2].y, traj[i+2].z));
    double z0 = p0.z();
    double z1 = p1.z();
    double z2 = p2.z();
    double t0 = traj[i].t.toSec();
    double t1 = traj[i+1].t.toSec();
    double t2 = traj[i+2].t.toSec();
    double v1 = (z1-z0)/(t1-t0);
    double v2 = (z2-z1)/(t2-t1);
    z_accel[i] = 2*(v2-v1)/(t2-t0);
  }

  int i = qselect(z_accel, n, n/2);
  return z_accel[i];
}
*/

/* Classify live trajectories with Bayes rule:
 *
 * p(onTable=1 | isBall=1) = .7
 * p(onTable=1 | isBall=0) = .05
 *
 * B := isBall
 * O := onTable
 *
 * p(B|O) = P(O|B)*P(B) / P(O)
 *
void classify_live_trajectories(ball_tracker_t &bt)
{
  double initial_prior = .3, on_table_true_pos = .7, on_table_false_pos = .05;
  for (uint i = 0; i < bt.live_trajectories.size(); i++) {
    ball_t &b = bt.live_trajectories[i].back();
    uint n = bt.live_trajectories[i].size();
    if (n >= 5) {
      double p_B_true_prior = initial_prior;  //bt.live_trajectories[i][n-2].w;
      bool on_table = trajectory_in_table_region(bt.live_trajectories[i]);
      double p_B_true = (on_table ? on_table_true_pos : 1 - on_table_true_pos) * p_B_true_prior;
      double p_B_false = (on_table ? on_table_false_pos : 1 - on_table_false_pos) * (1 - p_B_true_prior);
      b.w = p_B_true / (p_B_true + p_B_false);

      // add in gravity classification -- TOO NOISY
      double z_accel = trajectory_median_z_accel(bt.live_trajectories[i]);
      //printf("median z accel = %.2f\n", z_accel);
    }
    else
      b.w = initial_prior;
  }
}
*/

//----------------------- BALL DETECTION --------------------//

/*
vector<ball_t> balls_from_connected_components(ushort *C, int w, int h, int ncomp, ros::Time stamp)
{
  //uint w = C.width;
  //uint h = C.height;

  // compute non-zero pixel indices
  uint num_pixels = 0;
  for (int i = 0; i < w*h; i++)
    if (C[i])
      num_pixels++;
  uint idx[num_pixels];
  num_pixels = 0;
  for (int i = 0; i < w*h; i++)
    if (C[i])
      idx[num_pixels++] = i;

  printf("num_pixels = %d\n", num_pixels);

  // get num components
  //uint num_comp = 0;
  //for (uint i = 0; i < num_pixels; i++)
  //  if (C[idx[i]] > num_comp)
  //    num_comp = C[idx[i]];
  uint num_comp = ncomp;

  // compute component sizes and centroids
  uint comp_sizes[num_comp+1];
  double comp_x[num_comp+1];
  double comp_y[num_comp+1];
  for (uint c = 1; c <= num_comp; c++)
    comp_sizes[c] = comp_x[c] = comp_y[c] = 0;
  for (uint i = 0; i < num_pixels; i++) {
    uint c = C[idx[i]];
    uint x = idx[i] % w;
    uint y = (idx[i] - x) / w;
    comp_sizes[c]++;
    comp_x[c] += x;
    comp_y[c] += y;
  }
  for (uint c = 1; c <= num_comp; c++) {
    comp_x[c] /= (double)comp_sizes[c];
    comp_y[c] /= (double)comp_sizes[c];
    //printf("comp %d (%.0f, %.0f, %d)\n", c, comp_x[c], comp_y[c], comp_sizes[c]);
  }

  // get balls within a range of sizes
  vector<ball_t> balls;
  for (uint c = 1; c <= num_comp; c++) {
    if (comp_sizes[c] >= ball_min_size && comp_sizes[c] <= ball_max_size) {
      ball_t b;
      b.t = stamp;
      b.x = comp_x[c];
      b.y = comp_y[c];
      balls.push_back(b);
    }
  }
  
  return balls;
}
*/

/*
void draw_table_left(cv_bridge::CvImagePtr cv_ptr)
{
  cv::Point3d corners[4];
  cv::Point2d points[4];
  table_corners_from_transform(corners, table_camera_transform);
  table_corners_to_left_image(points, corners);
  for (int i = 0; i < 4; i++)
    cv::line(cv_ptr->image, points[i], points[(i+1)%4], GREEN);
}

void draw_table_right(cv_bridge::CvImagePtr cv_ptr)
{
  cv::Point3d corners[4];
  cv::Point2d points[4];
  table_corners_from_transform(corners, table_camera_transform);
  table_corners_to_right_image(points, corners);
  for (int i = 0; i < 4; i++)
    cv::line(cv_ptr->image, points[i], points[(i+1)%4], GREEN);
}

void draw_roi(cv_bridge::CvImagePtr cv_ptr, rect_t roi)
{
  //printf("roi = (%d, %d, %d, %d)\n", roi.x, roi.y, roi.w, roi.h);

  cv::Point2d p1(roi.x, roi.y);
  cv::Point2d p2(roi.x + roi.w, roi.y);
  cv::Point2d p3(roi.x + roi.w, roi.y + roi.h);
  cv::Point2d p4(roi.x, roi.y + roi.h);
  cv::line(cv_ptr->image, p1, p2, RED);
  cv::line(cv_ptr->image, p2, p3, RED);
  cv::line(cv_ptr->image, p3, p4, RED);
  cv::line(cv_ptr->image, p4, p1, RED);
}

void draw_ball_tracker(ball_tracker_t &bt)
{
  // convert to opencv
  cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
  try {
    cv_ptr_left =
      cv_bridge::toCvCopy(left_eye.last_msg, sensor_msgs::image_encodings::BGR8);
    cv_ptr_right =
      cv_bridge::toCvCopy(right_eye.last_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // draw table
  draw_table_left(cv_ptr_left);
  draw_table_right(cv_ptr_right);

  //dbug: draw ROIs
  draw_roi(cv_ptr_left, left_eye.roi);
  draw_roi(cv_ptr_right, right_eye.roi);

  // is there a ball trajectory?
  bool found_ball = false;
  for (uint i = 0; i < bt.live_trajectories.size(); i++) {
    if (bt.live_trajectories[i].back().w > .5) {
      found_ball = true;
      break;
    }
  }

  // draw live trajectories
  for (uint i = 0; i < bt.live_trajectories.size(); i++) {
    //if (found_ball && bt.live_trajectories[i].back().w < .5)
    //  continue;
    cv::Scalar color = GREEN;
    if (bt.live_trajectories[i].size() < 5)
      color = GREEN;
    else if (bt.live_trajectories[i].back().w < .5)  //(bt.live_trajectories[i].size() < 10)
      color = BLUE;
    else
      color = RED;
    for (uint j = 0; j < bt.live_trajectories[i].size(); j++) {
      ball_t &b = bt.live_trajectories[i][j];
      cv::Point2d p1 =
	stereo_camera_model.left().project3dToPixel(cv::Point3d(b.x, b.y, b.z));
      cv::Point2d p2 =
	stereo_camera_model.right().project3dToPixel(cv::Point3d(b.x, b.y, b.z));

      // draw a circle
      //if (b.circle_score < 4)
      //	color = GREEN;
      //else if (b.circle_score < 5)
      //	color = BLUE;
      //else
      //	color = RED;
      //cv::circle(cv_ptr_left->image, cv::Point(b.lx, b.ly), ceil(b.r), GREEN, 1);
      //cv::circle(cv_ptr_right->image, cv::Point(b.rx, b.ry), ceil(b.r), GREEN, 1);      
      //cv::circle(cv_ptr_left->image, cv::Point(p1.x, p1.y), ceil(b.r), RED, 1);
      //cv::circle(cv_ptr_right->image, cv::Point(p2.x, p2.y), ceil(b.r), RED, 1);

      cv::circle(cv_ptr_left->image, cv::Point(p1.x, p1.y), ceil(b.r), color, 1);
      cv::circle(cv_ptr_right->image, cv::Point(p2.x, p2.y), ceil(b.r), color, 1);
    }
  }

  // convert to ROS
  sensor_msgs::Image I1, I2;
  cv_ptr_left->toImageMsg(I1);
  cv_ptr_right->toImageMsg(I2);

  // publish trajectory image
  left_eye.pub[0].publish(I1);
  right_eye.pub[0].publish(I2);
}
*/

