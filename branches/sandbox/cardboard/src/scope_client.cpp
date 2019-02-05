#include <ros/ros.h>
// Figure out how to put bingham as the library

#include <iostream>

#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <sensor_msgs/ChannelFloat32.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "cardboard/Scope.h"

extern "C" {
#include <bingham/util.h>
#include <bingham/olf.h>
}

using namespace std;

// This came from furniture package // <-- Move it to that scope util I planned a while ago?
template <class PointT>
  void get_color(PointT p, int &c1, int &c2, int &c3) {
    uint32_t c123 = *reinterpret_cast<int*>(&p.rgb);
    c1 = (int) (c123 >> 16) & 0x0000ff;
    c2 = (int) (c123 >> 8)  & 0x0000ff;
    c3 = (int) (c123)       & 0x0000ff;
  }


typedef struct {
  double **coeffs;
  int **inliers;
  int *inliers_len;
  int num_planes;
} planes_struct_t;

void find_planes(planes_struct_t &plane_info, double **cloud, int orig_num_points) {
  int num_points = orig_num_points;
  double **tmp_cloud;
  tmp_cloud = new_matrix2(num_points, 3);
  memcpy(tmp_cloud[0], cloud[0], num_points * 3 * sizeof(double));
  int min_inliers = num_points / 30;
  double inlier_dist = 0.01; // 0.003;
  int num_samples = 500;
  
  plane_info.num_planes = 0;
  int *indices;
  safe_malloc(indices, num_points, int);
  for (int i = 0; i < num_points; ++i) {
    indices[i] = i;
  }

  plane_info.coeffs = new_matrix2(30, 4);
  // NOTE(sanja): If this has terrifingly bad performance, it's probably because of the clunky inliers...
  plane_info.inliers = (int **) malloc(30 * sizeof(int *));
  safe_malloc(plane_info.inliers_len, 30, int);
  int planes_alloced = 30;
    
  while (num_points > 0) {
    double cmax[4];
    int nmax = 0;
    for (int i = 0; i < num_samples; ++i) {
      // sample 3 points
      int point_indices[3];
      do {
	for (int j = 0; j < 3; ++j) {
	  point_indices[j] = irand(num_points);
	}
      } while(!(point_indices[0] != point_indices[1] && point_indices[0] != point_indices[2] && point_indices[1] != point_indices[2]));
      
      // Compute plane coefficients
      double diff1[3], diff2[3];
      sub(diff1, tmp_cloud[point_indices[1]], tmp_cloud[point_indices[0]], 3);
      sub(diff2, tmp_cloud[point_indices[2]], tmp_cloud[point_indices[0]], 3);
      double n[3];
      cross(n, diff1, diff2);
      normalize(n, n, 3);
      double c[4];
      c[0] = n[0]; c[1] = n[1]; c[2] = n[2]; c[3] = -dot(n, tmp_cloud[point_indices[0]], 3);
      
      // Compute num inliers
      int n_in = 0;
      for (int j = 0; j < num_points; ++j) {
        double d = c[3];
	//for (int k = 0; k < 3; ++k) {
	//d += c[k] * tmp_cloud[j][k];
	//}
	d += dot(c, tmp_cloud[j], 3);
	d = fabs(d);
	if (d < inlier_dist) ++n_in;
      }
      if (n_in > nmax) {
	nmax = n_in;
	for (int j = 0; j < 4; ++j) {
	  cmax[j] = c[j];
	}
      }
    }

    if (nmax >= min_inliers) {
      ++plane_info.num_planes;
      if (plane_info.num_planes == planes_alloced) {
	planes_alloced <<= 1;
	add_rows_matrix2(&plane_info.coeffs, planes_alloced >> 1, 4, planes_alloced);
	safe_realloc(plane_info.inliers, planes_alloced, int*);
	safe_realloc(plane_info.inliers_len, planes_alloced, int);
      }
      
      // Unroll loop for better performance
      plane_info.coeffs[plane_info.num_planes-1][0] = cmax[0];
      plane_info.coeffs[plane_info.num_planes-1][1] = cmax[1];
      plane_info.coeffs[plane_info.num_planes-1][2] = cmax[2];
      plane_info.coeffs[plane_info.num_planes-1][3] = cmax[3];

      safe_malloc(plane_info.inliers[plane_info.num_planes-1], nmax, int);

      int insert = 0;
      plane_info.inliers_len[plane_info.num_planes-1] = 0;
      for (int j = 0; j < num_points; ++j) {
	double d = cmax[3];
	//for (int k = 0; k < 3; ++k) {
	//  d += cmax[k] * tmp_cloud[j][k];
	//}
	d += dot(cmax, tmp_cloud[j], 3);
	d = fabs(d);
	if (d < inlier_dist) {
	  plane_info.inliers[plane_info.num_planes-1][plane_info.inliers_len[plane_info.num_planes-1]++] = indices[j];
	} else {
	  tmp_cloud[insert][0] = tmp_cloud[j][0];
	  tmp_cloud[insert][1] = tmp_cloud[j][1];
	  tmp_cloud[insert][2] = tmp_cloud[j][2];
	  indices[insert] = indices[j];
	  ++insert;
	}
      }
      num_points = insert;
    } else {
      break;
    }
  }
  ROS_INFO("Out of loop\n");

  for (int i = 0; i < plane_info.num_planes; ++i)
    printf("Coeffs: %lf %lf %lf %lf\n", plane_info.coeffs[i][0], plane_info.coeffs[i][1], plane_info.coeffs[i][2], plane_info.coeffs[i][3]);


  free_matrix2(tmp_cloud);
  free(indices);

  safe_realloc(plane_info.coeffs[0], plane_info.num_planes * 4, double);
  safe_realloc(plane_info.coeffs, plane_info.num_planes, double*);

  safe_realloc(plane_info.inliers, plane_info.num_planes, int*);
  safe_realloc(plane_info.inliers_len, plane_info.num_planes, int);
 
}

void find_turntable(double x[3], double n[3], pcl::PointCloud<pcl::PointXYZRGB> pcd) {
  double **cloud;
  cloud = new_matrix2(pcd.points.size()/10, 3);
  int num_points = 0;
  for (int i = 0; i < pcd.points.size()/10; ++i) {
    cloud[num_points][0] = pcd.points[10*i].x;
    cloud[num_points][1] = pcd.points[10*i].y;
    cloud[num_points][2] = pcd.points[10*i].z;
    if (!(isnan(cloud[num_points][0]) || isnan(cloud[num_points][1]) || isnan(cloud[num_points][2]))) {
      ++num_points;
    }
  }  

  safe_realloc(cloud[0], num_points * 3, double);
  safe_realloc(cloud, num_points, double*);

  planes_struct_t plane_info;
  find_planes(plane_info, cloud, num_points); // Find_planes allocates all the necessary memory... Hopefully.

  // Find the turntable plane
  double max_theta = M_PI/10; //20;
  int iturn = -1;
  for (int i = 0; i < plane_info.num_planes; ++i) {
    double dtheta;
    double d = dot(plane_info.coeffs[0], plane_info.coeffs[i], 3);
    dtheta = fabs(acos(d));
    dtheta = min(dtheta, M_PI - dtheta);
    if (dtheta > max_theta) {
      plane_info.coeffs[i][3] = 1000000000;
    }
    if (iturn == -1 || fabs(plane_info.coeffs[i][3]) < fabs(plane_info.coeffs[iturn][3])) {
      iturn = i;
    }
  }

  printf("iturn = %d\n", iturn);
 
  // Get turntable normal and centroid
  int sz = plane_info.inliers_len[iturn];
  double W[sz];
  for (int i = 0; i < sz; ++i) {
    W[i] = 0;
    W[i] = dot(cloud[plane_info.inliers[iturn][i]], cloud[plane_info.inliers[iturn][i]], 3);
    W[i] = exp(-W[i]);
  }
  normalize_pmf(W, W, sz);
  x[0] = x[1] = x[2] = 0;
  for (int i = 0; i < sz; ++i) {
    for (int j = 0; j < 3; ++j) {
      x[j] += cloud[plane_info.inliers[iturn][i]][j] * W[i];
    }
  }  

  // Refine normal estimate
  double DX[3];
  int idx;
  vector<int> I;
  for (int i = 0; i < sz; ++i) {
    idx = plane_info.inliers[iturn][i];
    sub(DX, cloud[idx], x, 3);
    if (dot(DX, DX, 3) < 0.25) {
      I.push_back(idx);
    }
  }

  double **C;
  C = new_matrix2(3, 3);
  // Find covariance matrix
  // Call eigen_symm on the matrix
  // Take the first row
  double **cloud_inliers = new_matrix2(I.size(), 3);
  reorder_rows(cloud_inliers, cloud, &I[0], I.size(), 3);
  double mu[3];
  mean(mu, cloud_inliers, I.size(), 3);
  double **cov_mat = new_matrix2(3, 3);
  cov(cov_mat, cloud_inliers, mu, I.size(), 3);
  double z[3];
  double t0 = get_time_ms();
  eigen_symm(z, C, cov_mat, 3);
  printf("EigenSymm in %.3f seconds\n", (get_time_ms() - t0) / 1000.0);
  free_matrix2(cov_mat);

  n[0] = C[0][0];
  n[1] = C[0][1];
  n[2] = C[0][2];

  if (dot(x, n, 3) > 0) {
    mult(n, n, -1, 3);
  }

  free_matrix2(C);

  free_matrix2(plane_info.coeffs);
  for (int i = 0; i < plane_info.num_planes; ++i)
    free(plane_info.inliers[i]);
  free(plane_info.inliers);
  free(plane_info.inliers_len);
}

void get_tabletop_pcd(pcl::PointCloud<pcl::PointXYZRGB> &pcd_out, double table_params[4], int* &I, int &n_I, const pcl::PointCloud<pcl::PointXYZRGB> pcd_in, float radius, float delta) {
  n_I = 0;
  
  double x[3], n[3];
  find_turntable(x, n, pcd_in);
  
  printf("x = %lf %lf %lf\n", x[0], x[1], x[2]);
  printf("n = %lf %lf %lf\n", n[0], n[1], n[2]);

  int num_points = 0;
  /*sensor_msgs::ChannelFloat32 r, g, b;
  r.name = "red";
  g.name = "green";
  b.name = "blue";*/
  
  table_params[0] = n[0];
  table_params[1] = n[1];
  table_params[2] = n[2];
  table_params[3] = -dot(x, n, 3);

  /*for (int i = 0; i < pcd_in.channels.size(); ++i) {
    sensor_msgs::ChannelFloat32 ch;
    ch.name = pcd_in.channels[i].name;
    pcd_out.channels.push_back(ch);
    }*/

  pcd_out.header = pcd_in.header;
  safe_malloc(I, pcd_in.points.size(), int);

  double **N_proj = new_matrix2(3, 3);
  outer_prod(N_proj, n, n, 3, 3);
  double **eye = new_identity_matrix2(3);
  matrix_sub(N_proj, eye, N_proj, 3, 3);

  double M_proj[3];
  vec_matrix_mult(M_proj, x, N_proj, 3, 3);

  for (int i = 0; i < pcd_in.points.size(); ++i) {
    if (isnan(pcd_in.points[i].x) || isnan(pcd_in.points[i].y) || isnan(pcd_in.points[i].z))
      continue;
    double pt_proj[3];
    double pt[3];
    pt[0] = pcd_in.points[i].x; pt[1] = pcd_in.points[i].y; pt[2] = pcd_in.points[i].z;
    vec_matrix_mult(pt_proj, pt, N_proj, 3, 3);

    if ((dot(pt, n, 3) > (dot(x, n, 3) - delta)) && (dist2(pt_proj, M_proj, 3) < radius * radius)) {
      ++num_points;
      pcd_out.points.push_back(pcd_in.points[i]);
      I[n_I++] = i;
    }
  }
  pcd_out.width = pcd_out.points.size();
  pcd_out.height = 1;

  //safe_realloc(I, pcd_out.points.size(), int);
  free_matrix2(N_proj);
  free_matrix2(eye);
}

void crop_pcd_image(pcl::PointCloud<pcl::PointXYZRGB> &pcd_cropped, pcl::PointCloud<pcl::PointXYZRGB> pcd, int *I, int n_I, int padding) {
  int **XI, **YI;
  int w = pcd.width;
  int h = pcd.height;
  XI = new_matrix2i(h, w);
  YI = new_matrix2i(h, w);

  // C++ stores matrices row-wise
  for (int i = 0; i < h; ++i) {
    for (int j = 0; j < w; ++j) {
      YI[i][j] = i;
      XI[i][j] = j;
    }
  }
  
  printf("Original w = %d, h = %d\n", w, h);

  int minXI, maxXI, minYI, maxYI;
  minXI = 10000000;
  maxXI = -10000000;
  minYI = 10000000;
  maxYI = -10000000;
    
  for (int i = 0; i < n_I; ++i) {
    if ((*XI)[I[i]] < minXI) // Pointers ftw... Syntax abusing, ftw^2
      minXI = (*XI)[I[i]];
    if ((*XI)[I[i]] > maxXI)
      maxXI = (*XI)[I[i]];
    if ((*YI)[I[i]] < minYI)
      minYI = (*YI)[I[i]];
    if ((*YI)[I[i]] > maxYI)
      maxYI = (*YI)[I[i]];
  }

  int x0, x1, y0, y1;
  
  printf("minXI = %d, maxXI = %d, minYI = %d, maxYI = %d\n", minXI, maxXI, minYI, maxYI);

  x0 = MAX(0, minXI - padding);
  x1 = MIN(w-1, maxXI + padding);
  y0 = MAX(0, minYI - padding);
  y1 = MIN(h-1, maxYI + padding);
  
  /*y0 = MAX(0, minXI - padding);
  y1 = MIN(w-1, maxXI + padding);
  x0 = MAX(0, minYI - padding);
  x1 = MIN(h-1, minYI + padding);*/

  int h2 = y1 - y0 + 1;
  int w2 = x1 - x0 + 1;

  printf("x0 = %d, x1 = %d, y0 = %d, y1 = %d\n", x0, x1, y0, y1);

  pcd_cropped.width = w2;
  pcd_cropped.height = h2;
  pcd_cropped.header = pcd.header;

  for (int i = 0 ; i < pcd.points.size(); ++i) {
    if (((*XI)[i] >=x0) && ((*XI)[i] <= x1) && ((*YI)[i] >= y0) && ((*YI)[i] <= y1)) {
      pcd_cropped.points.push_back(pcd.points[i]);
    }
  }

  pcd_cropped.width = w2;
  pcd_cropped.height = h2;
  pcd_cropped.header = pcd.header;

  printf("w = %d, h = %d, total = %d\n", w2, h2, pcd_cropped.points.size());

  free_matrix2i(XI);
  free_matrix2i(YI);
}

void filter_pcd(pcl::PointCloud<pcl::PointXYZRGB> &pcd_out, pcl::PointCloud<pcl::PointXYZRGB> pcd_in) {
  pcd_out.header = pcd_in.header;
  for (int i = 0; i < pcd_in.points.size(); ++i) {
    if (isFinite(pcd_in.points[i])) {
      pcd_out.points.push_back(pcd_in.points[i]);
    }
  } 
  pcd_out.width = pcd_out.points.size();
  pcd_out.height = 1;
}

void crop_pcd(pcl::PointCloud<pcl::PointXYZRGB> &pcd_objects, pcl::PointCloud<pcl::PointXYZRGB> &pcd_cropped, pcl::PointCloud<pcl::PointXYZRGB> &pcd_cropped_and_filtered, 
	      pcl::PointCloud<pcl::PointXYZRGB> pcd, float radius, float delta) {

  double table_params[4];
  int *I;
  int n_I;
  get_tabletop_pcd(pcd_objects, table_params, I, n_I, pcd, radius, delta);
  printf("r = %lf, d = %lf\n", radius, delta);
  
  crop_pcd_image(pcd_cropped, pcd, I, n_I, 10);
  filter_pcd(pcd_cropped_and_filtered, pcd_cropped);
}

void transform_ros_cloud(sensor_msgs::PointCloud &cloud, double *x, double *q) {
  double **R = new_matrix2(3,3);
  quaternion_to_rotation_matrix(R,q);
  int i;
  for (i = 0; i < cloud.points.size(); i++) {
    double pt[3];
    pt[0] = cloud.points[i].x; pt[1] = cloud.points[i].y; pt[2] = cloud.points[i].z;
    matrix_vec_mult(pt, R, pt, 3, 3);
    if (x != NULL)
      add(pt, pt, x, 3);
    cloud.points[i].x = pt[0]; cloud.points[i].y = pt[1]; cloud.points[i].z = pt[2];
  }
  free_matrix2(R);
}

int main(int argc, char **argv) {

  ROS_INFO("Started client!");

  // init ROS
  ros::init(argc, argv, "scope_client");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<cardboard::Scope>("scope_service");
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("located_cloud", 1000);
  
  cardboard::Scope srv;
  srv.request.model_name = argv[1];

  string topic = "/camera/depth_registered/points";

  //  while (1) {
    ROS_INFO("Waiting for stuff");
    
    pcl::PointCloud<pcl::PointXYZRGB> pcd_cropped;
    pcl::PointCloud<pcl::PointXYZRGB> pcd_objects;
    pcl::PointCloud<pcl::PointXYZRGB> pcd_cropped_and_filtered;

    pcl::PointCloud<pcl::PointXYZRGB> pcd;

    int num_clouds = 5; // TODO: Make this a launch parameter

    double **pts;
    pts = new_matrix2(640 * 480, 6);
    
    double t0 = get_time_ms();
    for (int i = 0; i < num_clouds; ++i) {
      sensor_msgs::PointCloud2ConstPtr cloud_blob_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic);
      pcl::PointCloud<pcl::PointXYZRGB> tmp_pcd;
      pcl::fromROSMsg(*cloud_blob_ptr, tmp_pcd);
      if (i == 0)
	pcl::fromROSMsg(*cloud_blob_ptr, pcd);
      
      for (int j = 0; j < tmp_pcd.points.size(); ++j) {
	pts[j][0] += tmp_pcd.points[j].x;
	pts[j][1] += tmp_pcd.points[j].y;
	pts[j][2] += tmp_pcd.points[j].z;
	// TODO: Average colors as well (I don't want to deal with packing, doubles and other ugly stuff. Sorry, Jared :)
      }      
    }
    mult(pts[0], pts[0], 1.0 / (double) num_clouds, 640 * 480 * 6);
    for (int i = 0; i < pcd.points.size(); ++i) {    
      pcd.points[i].x = pts[i][0];
      pcd.points[i].y = pts[i][1];
      pcd.points[i].z = pts[i][2];
    }
    
    printf("Averaged pointclouds in %.3f seconds\n", (get_time_ms() - t0) / 1000.0);

    t0 = get_time_ms();
    // TODO: Make the params for crop_pcd launch params.
    ROS_INFO("About to crop");
    crop_pcd(pcd_objects, pcd_cropped, pcd_cropped_and_filtered, pcd, 0.5, -0.01);
    printf("Cropped in %.3f seconds\n", (get_time_ms() - t0) / 1000.0);

    sensor_msgs::PointCloud2 pcd_cropped_ros, pcd_cropped_and_filtered_ros, pcd_objects_ros;
    pcl::toROSMsg(pcd_cropped, pcd_cropped_ros);
    pcl::toROSMsg(pcd_cropped_and_filtered, pcd_cropped_and_filtered_ros);
    pcl::toROSMsg(pcd_objects, pcd_objects_ros);
    srv.request.cloud_bg = pcd_cropped_ros;
    srv.request.cloud_objects = pcd_objects_ros;
    srv.request.true_pose = ""; // This is the unlabeled scene
    
    // dbug
    /*pcl::io::savePCDFile("uncropped.pcd", pcd);
      ROS_INFO("Saved uncropped point cloud.");
      pcl::io::savePCDFile("objects_cropped.pcd", pcd_objects);
      ROS_INFO("Saved objects point cloud.");
      pcl::io::savePCDFile("bg_cropped.pcd", pcd_cropped);
      ROS_INFO("Saved cropped point cloud.");*/
    
    ROS_INFO("Calling server");
    
    if (client.call(srv)) {
      ROS_INFO("Response received");      
      // TODO: make publishing to RViz optional (add a switch to the launch file)
      // Publish data to RViz
      string model_path = "/home/sanja/papers/icra13/data/kinect_scans/"; //NOTE(sanja): I am unsure why the hell will this not work as "~/...", but it will be moved to a launch file anyway
      // load model data
      pcd_t *model_pcd;
      string full_path = model_path + srv.request.model_name + "/occ_model.pcd";
      char *fp_file;
      safe_malloc(fp_file, full_path.length(), char);
      strcpy(fp_file, full_path.c_str());
      model_pcd = load_pcd(fp_file);
      free(fp_file);

      double **transformed = new_matrix2(model_pcd->num_points, 3);
      for (int i = 0; i < srv.response.object_samples.objects.size(); ++i) {
	sensor_msgs::PointCloud object_transformed;
	
	double x[3], q[4];
	x[0] = srv.response.object_samples.objects[i].pose.position.x;
	x[1] = srv.response.object_samples.objects[i].pose.position.y;
	x[2] = srv.response.object_samples.objects[i].pose.position.z;
	
	q[0] = srv.response.object_samples.objects[i].pose.orientation.x;
	q[1] = srv.response.object_samples.objects[i].pose.orientation.y;
	q[2] = srv.response.object_samples.objects[i].pose.orientation.z;
	q[3] = srv.response.object_samples.objects[i].pose.orientation.w;
	
	//transform_ros_cloud(object_transformed, x, q);
	transform_cloud(transformed, model_pcd->points, model_pcd->num_points, x, q);
	object_transformed.header = pcd_objects_ros.header;
	for (int j = 0; j < model_pcd->num_points; ++j) {
	  geometry_msgs::Point32 pt;
	  pt.x = transformed[j][0]; pt.y = transformed[j][1]; pt.z = transformed[j][2];
	  object_transformed.points.push_back(pt);
	}
	
	cloud_pub.publish(object_transformed);
	char action;
	printf("i = %d ", i);
	scanf("%c", &action);
	if (action == 's')
	  break;
      }
      free_matrix2(transformed);
      //pcd_free(model_pcd);
    }
    //  }
  
  return 0;
}
