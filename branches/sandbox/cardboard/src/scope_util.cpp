#include <cardboard/scope_util.h>

void load_true_pose(string pose_file, simple_pose_t *true_pose) {
  FILE *f = fopen(pose_file.c_str(), "r");
  if (f == NULL) {
    fprintf(stderr, "Error loading true pose file: %s\n", pose_file.c_str());
    return;
  }
  char sbuf[1024];
  char *s = sbuf;
  if (fgets(s, 1024, f)) {
    //s = sword(s, " ", 1);
    int i;
    for (i = 0; i < 3; ++i) {
      s = sword(s, " ", 1);
      true_pose->X[i] = atof(s);
    }
    for (i = 0; i < 4; ++i) {
      s = sword(s, " ", 1);
      true_pose->Q[i] = atof(s);
    }
  }
  fclose(f);
}

void edge_image(sensor_msgs::Image &E, const sensor_msgs::Image &I)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(I, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat1b E_cv;
  //I.assignTo(I2, DataType<uchar>::type);
  E_cv.create(cv_ptr->image.size());
  cv::Canny(cv_ptr->image, E_cv, 20, 40);

  E = I;
  uint w = E.width, h = E.height;
  for (uint y = 0; y < h; y++)
    for (uint x = 0; x < w; x++)
      E.data[y*w+x] = E_cv(y,x);
}

void pclxyzrgb_to_bingham_pcd(pcd_t &pcd_out, pcl::PointCloud<pcl::PointXYZRGB> pcd_in) {
  pcd_out.num_channels = 6;
  
  /*pcd_out.channels = (char **) malloc(pcd_out.num_channels * sizeof(char *));
  for (int i = 0; i < 6; ++i) {
    safe_calloc(pcd_out.channels[i], 30, char);
    }*/

  sprintf(pcd_out.channels[0], "x");
  sprintf(pcd_out.channels[1], "y");
  sprintf(pcd_out.channels[2], "z");
  sprintf(pcd_out.channels[3], "red");
  sprintf(pcd_out.channels[4], "green");
  sprintf(pcd_out.channels[5], "blue");
  
  pcd_out.num_points = pcd_in.points.size();
  pcd_out.data = new_matrix2(pcd_out.num_channels, pcd_out.num_points);

  pcd_out.points = new_matrix2(pcd_out.num_points, 3);
  pcd_out.colors = new_matrix2(pcd_out.num_points, 3);
  pcd_out.lab = new_matrix2(pcd_out.num_points, 3);
  
  for (int i = 0; i < pcd_out.num_points; ++i) {
    pcd_out.data[0][i] = pcd_in.points[i].x;
    pcd_out.data[1][i] = pcd_in.points[i].y;
    pcd_out.data[2][i] = pcd_in.points[i].z;
    pcd_out.points[i][0] = pcd_out.data[0][i];
    pcd_out.points[i][1] = pcd_out.data[1][i];
    pcd_out.points[i][2] = pcd_out.data[2][i];

    uint32_t rgb = *reinterpret_cast<int*>(&pcd_in.points[i].rgb);
    uint8_t r = (rgb >> 16) & 0x0000ff;
    uint8_t g = (rgb >> 8) & 0x0000ff;
    uint8_t b = (rgb) & 0x0000ff;
    pcd_out.data[3][i] = (double) r;
    pcd_out.data[4][i] = (double) g;
    pcd_out.data[5][i] = (double) b;

    pcd_out.colors[i][0] = pcd_out.data[3][i];
    pcd_out.colors[i][1] = pcd_out.data[4][i];
    pcd_out.colors[i][2] = pcd_out.data[5][i];
    rgb2lab(pcd_out.lab[i], pcd_out.colors[i]);
  }   
}

void pcd_add_normals_idx(pcd_t &pcd_out, pcl::PointCloud<pcl::PointNormal> pcd_in, vector<int> I) {
  int old_n = pcd_out.num_channels;
  pcd_out.num_channels += 3;
  /*pcd_out.channels = (char **) realloc(pcd_out.channels, pcd_out.num_channels * sizeof(char *));

  for (int i = 0; i < 3; ++i) {
    safe_calloc(pcd_out.channels[i + old_n], 30, char);
    }*/

  // NOTE(sanja): Ask Jared if null-termination is necessary
  sprintf(pcd_out.channels[old_n], "nx");
  sprintf(pcd_out.channels[old_n + 1], "ny");
  sprintf(pcd_out.channels[old_n + 2], "nz");
  
  add_rows_matrix2(&pcd_out.data, old_n, pcd_out.num_points, pcd_out.num_channels);
  pcd_out.normals = new_matrix2(pcd_out.num_points, 3);
  for (int i = 0; i < I.size(); ++i) { //pcd_out.num_points; ++i) {
    pcd_out.data[old_n][I[i]] = pcd_in.points[i].normal_x;
    pcd_out.data[old_n + 1][I[i]] = pcd_in.points[i].normal_y;
    pcd_out.data[old_n + 2][I[i]] = pcd_in.points[i].normal_z;
    
    pcd_out.normals[I[i]][0] = pcd_out.data[old_n][I[i]];
    pcd_out.normals[I[i]][1] = pcd_out.data[old_n + 1][I[i]];
    pcd_out.normals[I[i]][2] = pcd_out.data[old_n + 2][I[i]];
  }
}

void pcd_add_normals(pcd_t &pcd_out, pcl::PointCloud<pcl::PointNormal> pcd_in) {
  int old_n = pcd_out.num_channels;
  pcd_out.num_channels += 3;
  /*pcd_out.channels = (char **) realloc(pcd_out.channels, pcd_out.num_channels * sizeof(char *));

  for (int i = 0; i < 3; ++i) {
    safe_calloc(pcd_out.channels[i + old_n], 30, char);
    }*/

  // NOTE(sanja): Ask Jared if null-termination is necessary
  pcd_out.channels[old_n] = "nx\0";
  pcd_out.channels[old_n + 1] = "ny\0";
  pcd_out.channels[old_n + 2] = "nz\0";
  
  add_rows_matrix2(&pcd_out.data, old_n, pcd_out.num_points, pcd_out.num_channels);
  pcd_out.normals = new_matrix2(pcd_out.num_points, 3);
  for (int i = 0; i < pcd_out.num_points; ++i) {
    pcd_out.data[old_n][i] = pcd_in.points[i].normal_x;
    pcd_out.data[old_n + 1][i] = pcd_in.points[i].normal_y;
    pcd_out.data[old_n + 2][i] = pcd_in.points[i].normal_z;
    
    pcd_out.normals[i][0] = pcd_out.data[old_n][i];
    pcd_out.normals[i][1] = pcd_out.data[old_n + 1][i];
    pcd_out.normals[i][2] = pcd_out.data[old_n + 2][i];
  }
}



void pcd_add_curvature(pcd_t &pcd_out, pcl::PointCloud<pcl::PrincipalCurvatures> pcl_in) {
  int old_n = pcd_out.num_channels;
  pcd_out.num_channels += 5;
  /*pcd_out.channels = (char **) realloc(pcd_out.channels, pcd_out.num_channels * sizeof(char *));

  for (int i = 0; i < 5; ++i) {
    safe_calloc(pcd_out.channels[i + old_n], 30, char);
    }*/

  sprintf(pcd_out.channels[old_n], "pcx");
  sprintf(pcd_out.channels[old_n + 1], "pcy");
  sprintf(pcd_out.channels[old_n + 2], "pcz");
  sprintf(pcd_out.channels[old_n + 3], "pc1");
  sprintf(pcd_out.channels[old_n + 4], "pc2");
  
  add_rows_matrix2(&pcd_out.data, old_n, pcd_out.num_points, pcd_out.num_channels);
  pcd_out.principal_curvatures = new_matrix2(pcd_out.num_points, 3);
  
  for (int i = 0; i < pcd_out.num_points; ++i) {
    pcd_out.data[old_n][i] = pcl_in.points[i].principal_curvature[0];
    pcd_out.data[old_n + 1][i] = pcl_in.points[i].principal_curvature[1];
    pcd_out.data[old_n + 2][i] = pcl_in.points[i].principal_curvature[2];
    pcd_out.data[old_n + 3][i] = pcl_in.points[i].pc1;
    pcd_out.data[old_n + 4][i] = pcl_in.points[i].pc2;
    
    pcd_out.principal_curvatures[i][0] = pcd_out.data[old_n][i];
    pcd_out.principal_curvatures[i][1] = pcd_out.data[old_n + 1][i];
    pcd_out.principal_curvatures[i][2] = pcd_out.data[old_n + 2][i];
  }
  pcd_out.pc1 = pcd_out.data[old_n + 3];
  pcd_out.pc2 = pcd_out.data[old_n + 4];
}

void pcd_add_fpfhs(pcd_t &pcd_out, pcl::PointCloud<pcl::FPFHSignature33> pcl_in) {
  int old_n = pcd_out.num_channels;
  pcd_out.num_channels += 33;
  pcd_out.fpfh_length = 33;
  //pcd_out.channels = (char **) realloc(pcd_out.channels, pcd_out.num_channels * sizeof(char *));

  for (int i = 0; i < 33; ++i) {
    //safe_calloc(pcd_out.channels[i + old_n], 30, char);
    sprintf(pcd_out.channels[old_n + i], "f%d", i+1);
  }
  
  add_rows_matrix2(&pcd_out.data, old_n, pcd_out.num_points, pcd_out.num_channels);
  pcd_out.fpfh = new_matrix2(pcd_out.num_points, 33);
  
  for (int i = 0; i < pcd_out.num_points; ++i) {
    for (int j = 0; j < 33; ++j) {
      pcd_out.data[old_n + j][i] = pcl_in.points[i].histogram[j];
    }
    for (int j = 0; j < 33; ++j) {
      pcd_out.fpfh[i][j] = pcd_out.data[old_n + j][i];
    }
  }
}

void dense_pcd_find_knn(int *nn_idx, double *nn_d2, int query_idx, int query_len,
		       double **data, int h, int w, int search_radius_pixels, int k)
{
  int query_i = floor(query_idx / w);
  int query_j = floor(query_idx % w);
  int r = search_radius_pixels;
  int last = 0;
  double dist, tmp_d2;
  int tmp_idx;
  for (int i = MAX(0, query_i - r); i <= MIN(h - 1, query_i + r); ++i) {
    for (int j = MAX(0, query_j - r); j <= MIN(w - 1, query_j + r); ++j) {
      dist = 0;
      int data_idx = query_i * w + query_j;
      for (int kk = 0; kk < query_len; ++kk)
	dist+= (data[data_idx][kk] - data[query_idx][kk]) * (data[data_idx][kk] - data[query_idx][kk]);
      if (last == k && dist >= nn_d2[last-1])
	continue;
      if (last == k && dist >= nn_d2[last-1])
	continue;
      if (last < k)
	++last;
      if (last < k || (last == k && nn_d2[last-1] > dist)) {
	nn_d2[last-1] = dist;
	nn_idx[last-1] = data_idx;
      }
      for (int jj = last-1; jj > 0; --jj) {
	if (nn_d2[jj] < nn_d2[jj-1]) {
	  tmp_d2 = nn_d2[jj]; nn_d2[jj] = nn_d2[jj-1]; nn_d2[jj-1] = tmp_d2;
	  tmp_idx = nn_idx[jj]; nn_idx[jj] = nn_idx[jj-1]; nn_idx[jj-1] = tmp_idx;
	}
      }
    }
  }
}

void remove_outliers_pt_norm_pc(pcd_t &pcd, int *outliers) { // NOTE(sanja): terrible name to signal that this function is a gross hack
  int n = pcd.num_points;
  int insert = 0;
  for (int i = 0; i < n; ++i) {
    if (!outliers[i]) {
      memcpy(pcd.points[insert], pcd.points[i], 3 * sizeof(double));
      memcpy(pcd.normals[insert], pcd.normals[i], 3 * sizeof(double));
      memcpy(pcd.principal_curvatures[insert], pcd.principal_curvatures[i], 3 * sizeof(double));
      ++insert;
    }
  }
  safe_realloc(pcd.points[0], 3 * insert, double);
  pcd.points = (double **) realloc(pcd.points, insert * sizeof(double *));
  safe_realloc(pcd.normals[0], 3 * insert, double);
  pcd.normals = (double **) realloc(pcd.normals, insert * sizeof(double *));
  safe_realloc(pcd.principal_curvatures[0], 3 * insert, double);
  pcd.principal_curvatures = (double **) realloc(pcd.principal_curvatures, insert * sizeof(double *));
  pcd.num_points = insert;

  insert = 0;
  for (int i = 0; i < pcd.num_channels; ++i) {
    for (int j = 0; j < n; ++j) {
      if (!outliers[j]) {
	pcd.data[0][insert++] = pcd.data[0][i * pcd.num_points + j];
      }
    }
  }
  safe_realloc(pcd.data[0], pcd.num_points * pcd.num_channels, double);
  for (int i = 1; i < pcd.num_channels; ++i) {
    pcd.data[i] = &pcd.data[0][i * pcd.num_points]; // NOTE(sanja): This might be mildly sketchy
  }
  pcd.pc1 = pcd.data[pcd_channel(&pcd, "pc1")];
  pcd.pc2 = pcd.data[pcd_channel(&pcd, "pc1")];

}

void pcd_compute_principal_curvatures_find_negative_curvature_points(pcd_t &pcd, int* &I, int &n_I, double* &C_thin, int h, int w) {
  double t_tmp = 0.0;
  double t_eig_sym = 0.0;
  double t_knn = 0.0;
  double t_proj = 0.0;

  double curv_thresh = 40.0;

  int knn = 30;
  int n = pcd.num_points;
  double pcs[n][3], pc1[n], pc2[n];
  int outliers[n];
  memset(outliers, 0, n * sizeof(int));

  double nn_d2[2*knn];
  int nn_idx[2*knn];
  int mask[2*knn];
  int nn_cnt;

  int mask_neg[n];
  memset(mask_neg, 0, n * sizeof(int));
  n_I = 0;

  int C_idx_left[n], C_idx_right[n];
  safe_calloc(C_thin, n, double);

  for (int i = 0; i < n; ++i) {
    if (isnan(pcd.points[i][0]) || isnan(pcd.points[i][1]) || isnan(pcd.points[i][2]))
      continue;
    memset(mask, 0, 2 * knn * sizeof(int));
    nn_cnt = 0;
    t_tmp = get_time_ms();
    dense_pcd_find_knn(nn_idx, nn_d2, i, 3, pcd.points, h, w, 5, 2*knn);
    for (int j = 1; j < 2*knn; ++j) {
      if (dot(pcd.normals[nn_idx[j]], pcd.normals[i], 3) > 0) { // && nn_d2[j] < 0.01) {  // Less than 10cm away (nn_d2 is squared distance) <-- I must have hallucinated this...
	mask[j] = 1;
	++nn_cnt;
	if (nn_cnt == knn) {
	  break;
	}
      }
    }
    t_knn += get_time_ms() - t_tmp;
    
    if (nn_cnt < 3) {
      outliers[i] = 1;
      continue;
    }

    // project knn normals into plane orthogonal to normals[i]
    double **tmp = new_matrix2(3, 3);
    outer_prod(tmp, pcd.normals[i], pcd.normals[i], 3, 3);
    double **eye = new_identity_matrix2(3);
    matrix_sub(tmp, eye, tmp, 3, 3);
    transpose(tmp, tmp, 3, 3);
    free_matrix2(eye);
    
    double N[3];
    vec_matrix_mult(N, pcd.normals[i], tmp, 3, 3);
    free_matrix2(tmp);

    double **X = new_matrix2(3, 3);
    outer_prod(X, N, N, 3, 3);
    mult(X[0], X[0], 1.0/(double) nn_cnt, 9);
    double z[3];
    double **V = new_matrix2(3, 3);

    t_tmp = get_time_ms();
    eigen_symm(z, V, X, 3);
    t_eig_sym += get_time_ms() - t_tmp;

    pcs[i][0] = V[2][0]; pcs[i][1] = V[2][1]; pcs[i][2] = V[2][2];
    pc1[i] = sqrt(z[2]); pc2[i] = sqrt(z[1]);

    // Project KNN points & normals onto principal curvatures
    t_tmp = get_time_ms();
    double nn_proj_points[2*knn];
    double nn_proj_normals[2*knn];

    double angles[2 * knn];
    for (int j = 0; j < 2*knn; ++j) {
      if (mask[j]) { // This is OK because we terminate early once we get k neighbors that we like
	double diff[3];
	sub(diff, pcd.points[nn_idx[j]], pcd.points[i], 3);
	nn_proj_points[j] = dot(diff, pcs[i], 3);
	nn_proj_normals[j] = dot(pcd.normals[nn_idx[j]], pcs[i], 3);
	angles[j] = acos(abs(nn_proj_points[j]) / norm(diff, 3));
      } else { // This is OK because zeros won't affect dot products
	nn_proj_points[j] = 0;
	nn_proj_normals[j] = 0;
      }
    }    
    t_proj += get_time_ms() - t_tmp;
    
    // Compute correlation between projected points and normals
    C_thin[i] = dot(nn_proj_points, nn_proj_normals, 2*knn) / dot(nn_proj_points, nn_proj_points, 2*knn);
    mask_neg[i] = (C_thin[i] < -curv_thresh ? 1 : 0);
    if (mask_neg[i]) {
      ++n_I;
      int i_left = -1, i_right = -1;
      for (int j = 0; j < 2*knn; ++j) {
	if (!mask[j])
	  continue;

	if (nn_proj_points[j] < 0 && angles[j] < M_PI/4.0) {
	  if (i_left == -1 || nn_proj_points[i_left] < nn_proj_points[j])
	    i_left = j;
	}
	if (nn_proj_points[j] > 0 && angles[j] < M_PI/4.0) {
	  if (i_right == -1 || nn_proj_points[i_right] > nn_proj_points[j])
	    i_right = j;
	}
      }
      C_idx_left[i] = (i_left == -1 ? -1 : nn_idx[i_left]);
      C_idx_right[i] = (i_right == -1 ? -1 : nn_idx[i_right]);
    }
  }
  printf("EigenSymm in %.3f seconds\n", t_eig_sym / 1000.0);
  printf("KNN in %.3f seconds\n", t_knn / 1000.0);
  printf("Projecting in %.3f seconds\n", t_proj / 1000.0);
  
  int old_n = pcd.num_channels;
  pcd.num_channels += 1; //6;
  /*pcd.channels = (char **) realloc(pcd.channels, pcd.num_channels * sizeof(char *));
    
    safe_calloc(pcd.channels[old_n], 30, char);*/
  sprintf(pcd.channels[old_n + 0], "pcs_curvature");

  add_rows_matrix2(&pcd.data, old_n, pcd.num_points, pcd.num_channels);
  
  for (int i = 0; i < pcd.num_points; ++i) {
    pcd.data[old_n][i] = C_thin[i];
  }
  pcd.pcs_curvature = pcd.data[old_n];

  for (int i = 0; i < n; ++i) {
    if (mask_neg[i]) {
      bool left_is_bigger = C_idx_left[i] != -1 && (abs(C_thin[i]) <= abs(C_thin[C_idx_left[i]]));
      bool right_is_bigger = C_idx_right[i] != -1 && (abs(C_thin[i]) <= abs(C_thin[C_idx_right[i]]));
      if (left_is_bigger || right_is_bigger) {
	mask_neg[i] = 0;
	--n_I;
      }
    }
  }

  int cnt = 0;
  int insert = 0;
  if (n_I > 0) {
    safe_malloc(I, n_I, int);
    for (int i = 0; i < n; ++i) {
      if (mask_neg[i]) {
	I[cnt++] = i;
	C_thin[insert++] = C_thin[i];
      }
    }
  }
  printf("n_I = %d\n", n_I);
  if (n_I)
    safe_realloc(C_thin, n_I, double);
  else {
    free(C_thin);
    C_thin = 0;
  }

  // Add everything to the cloud
  old_n = pcd.num_channels;
  pcd.num_channels += 5;
  /*pcd.channels = (char **) realloc(pcd.channels, pcd.num_channels * sizeof(char *));
  
  for (int i = 0; i < 5; ++i) {
    safe_calloc(pcd.channels[i + old_n], 30, char);
    }*/
  sprintf(pcd.channels[old_n + 0], "pcx");
  sprintf(pcd.channels[old_n + 1], "pcy");
  sprintf(pcd.channels[old_n + 2], "pcz");
  sprintf(pcd.channels[old_n + 3], "pc1");
  sprintf(pcd.channels[old_n + 4], "pc2");

  add_rows_matrix2(&pcd.data, old_n, pcd.num_points, pcd.num_channels);
  pcd.principal_curvatures = new_matrix2(pcd.num_points, 3);

  for (int i = 0; i < pcd.num_points; ++i) {
    pcd.data[old_n + 0][i] = pcs[i][0];
    pcd.data[old_n + 1][i] = pcs[i][1];
    pcd.data[old_n + 2][i] = pcs[i][2];
    
    pcd.principal_curvatures[i][0] = pcd.data[old_n + 0][i];
    pcd.principal_curvatures[i][1] = pcd.data[old_n + 1][i];
    pcd.principal_curvatures[i][2] = pcd.data[old_n + 2][i];

    pcd.data[old_n + 3][i] = pc1[i];
    pcd.data[old_n + 4][i] = pc2[i];
  }

  remove_outliers_pt_norm_pc(pcd, outliers);
}

void DFS(int **out, int **in, int n, int m, int i, int j, int steps, int comp, int stepI[], int stepJ[]) {
  out[i][j] = comp;
  for (int s = 0; s < steps; ++s) {
    int newI = i + stepI[s];
    int newJ = j + stepJ[s];
    if ((newI >= 0) && (newI < n) && (newJ >= 0) && (newJ < m))
      if (out[i][j] == -1 && in[i][j] > 0)
	DFS(out, in, n, m, newI, newJ, steps, comp, stepI, stepJ);
  }
}

int bwlabel(int **out, int **in, int n, int m, int steps) {
  int stepI[8] = {-1, 0, 1, 0, 0, 0, 0, 0};
  int stepJ[8] = {0, 1, 0, -1, 0, 0, 0, 0};
  if (steps == 8) {
    // Set up diagonal steps
    stepI[4] = -1; stepI[5] = -1; stepI[6] = 1; stepI[7] = 1;
    stepJ[4] = -1; stepI[5] = 1; stepI[6] = 1; stepI[7] = -1;
  }

  for (int i = 0; i < n; ++i) 
    for (int j = 0; j < m; ++j)
      out[i][j] = -1;

  int comp = 0;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      if (out[i][j] == -1 && in[i][j] > 0) {
	++comp; 
	DFS(out, in, n, m, i, j, steps, comp, stepI, stepJ);
      }
    }
  }
  return comp;
}

bool compare_idx_depth(const pair<int, double>&i, const pair<int, double>&j) {
  return i.second > j.second;
}

void compute_range_edges(int* &edge_idx, int &n_edge_idx, range_image_t *range_image, int fill_background, double dthresh) {
  double **ZI = range_image->image;
  
  double practical_infty = 100000000;

  int w = range_image->w;
  int h = range_image->h;
  
  if (fill_background) {
    int **background = new_matrix2i(w, h);
    for (int i = 0; i < w; ++i)
      for (int j = 0; j < h; ++j)
	background[i][j] = (ZI[i][j] < 0 ? 1 : 0);

    int **L = new_matrix2i(w, h);
    int num_comps = bwlabel(L, background, w, h, 8);
    
    int comp_cnts[num_comps];
    memset(comp_cnts, 0, num_comps * sizeof(int));
    for (int i = 0; i < w; ++i)
      for (int j = 0; j < h; ++j)
	++comp_cnts[L[i][j]];

    for (int i = 0; i < w; ++i)
      for (int j = 0; j < h; ++j) {
	if (L[i][j] > -1 && comp_cnts[L[i][j]] > 20)
	  L[i][j] = practical_infty;
	else if (ZI[i][j] < 0)
	  ZI[i][j] = 0;
      }
  }
  
  for (int i = 0; i < w; ++i) {
    ZI[i][0] = practical_infty;
    ZI[i][h - 1] = practical_infty;
  }
  for (int i = 0; i < h; ++i) {
    ZI[0][i] = practical_infty;
    ZI[w - 1][i] = practical_infty;
  }
  
  // Fill holes in depth image
  int num_zero = 0;
  for (int i = 0; i < w; ++i) {
    for (int j = 0; j < h; ++j) {
      if (ZI[i][j] == 0)
	++num_zero;
    }
  }  
  // W ---> j - 1
  // E ---> j + 1
  // N ---> i - 1
  // S ---> i + 1
  while (num_zero > 0) {
    vector<pair<int, double> > nn_depths;
    for (int i = 1; i < w-1; ++i) {
      for (int j = 1; j < h-1; ++j) {
	if (ZI[i][j] == 0) {
	  double field = MAX(MAX(ZI[i-1][j], ZI[i+1][j]), MAX(ZI[i][j-1], ZI[i][j+1]));
	  nn_depths.push_back(pair<int, double>(i * h + j, field));
	}
      }
    }
    sort(nn_depths.begin(), nn_depths.end(), compare_idx_depth); // Sorts descending
    for (int i = 0; i < MIN(nn_depths.size(), 10); ++i) {
      ZI[0][nn_depths[i].first] = nn_depths[i].second;
      if (nn_depths[i].second != 0)
	--num_zero;
      if (num_zero == 0)
	break;
    }
  }

  // Compute filled depth image edges (and orientation)
  vector<int> edge_idx_vec; // We only care about edge_idx even though MATLAB code finds more stuff
  for (int i = 1; i < w-1; ++i) {
    for (int j = 1; j < h-1; ++j) {
      bool take = false;
      take = take || (ZI[i][j-1] - ZI[i][j] > dthresh);
      take = take || (ZI[i][j+1] - ZI[i][j] > dthresh);
      take = take || (ZI[i-1][j] - ZI[i][j] > dthresh);
      take = take || (ZI[i+1][j] - ZI[i][j] > dthresh);
      
      if (take && (range_image->idx[i][j] > -1)) {
	edge_idx_vec.push_back(i * h + j);
	if (range_image->idx[i][j] != range_image->idx[0][i*h + j]) // dbug
	  printf("ri->idx = %d, ri2->idx = %d, val = %d\n", range_image->idx[i][j], range_image->idx[0][i*h + j], i * h + j);
      }
    }
  }

  safe_calloc(edge_idx, edge_idx_vec.size(), int);
  n_edge_idx = edge_idx_vec.size();
  memcpy(edge_idx, &edge_idx_vec[0], sizeof(int) * n_edge_idx);

}

void compute_range_edges_for_pcd(pcd_t &pcd, int* &I, int &n_edge_idx, range_image_t *range_image, double dthresh) {
 
  int *edge_idx;
  compute_range_edges(edge_idx, n_edge_idx, range_image, 1, dthresh);
  
  safe_calloc(I, n_edge_idx, int);
  for (int i = 0; i < n_edge_idx; ++i)
    I[i] = range_image->idx[0][edge_idx[i]];
    
}

void add_pcd_edge_features(pcd_t &pcd, int *I_neg_curv, int n_neg_curv, double *C, double vp[7]) {
  double curv_thresh = 40.0;
  int *I_edge;
  int n_edge;
  range_image_t *range_image = pcd_to_range_image(&pcd, vp, M_PI/1000.0, 4);  // NEXT(sanja): Add blurring portion from MATLAB that's lacking in C
  compute_range_edges_for_pcd(pcd, I_edge, n_edge, range_image, .03);
  
  int w = range_image->w;
  int h = range_image->h;
  
  // Find image edges
  vector<int> I_image;
  sensor_msgs::Image red_image, green_image;
  red_image.width = w;
  red_image.height = h;
  red_image.data.resize(w*h);
  red_image.encoding = sensor_msgs::image_encodings::MONO8;

  green_image.width = w;
  green_image.height = h;
  green_image.data.resize(w*h);
  green_image.encoding = sensor_msgs::image_encodings::MONO8;

  uchar r_img_arr[pcd.num_points], g_img_arr[pcd.num_points];
  for (int i = 0; i < pcd.num_points; ++i) {
    r_img_arr[i] = (uchar) pcd.colors[i][0];
    g_img_arr[i] = (uchar) pcd.colors[i][2];
  }
  std::copy(r_img_arr, r_img_arr+w*h, red_image.data.begin());
  std::copy(g_img_arr, g_img_arr+w*h, green_image.data.begin());
  sensor_msgs::Image edge_red, edge_green;
  edge_image(edge_red, red_image);
  edge_image(edge_green, green_image);  
  for (int i = 0; i < w * h; ++i) {
    if (edge_red.data[i] || edge_green.data[i])
      I_image.push_back(i);
  }

  int n_image = I_image.size();
  int n = pcd.num_points;
  double **edge_data = new_matrix2(n, 3);
  // From MATLAB code: I1 is I_edge, I2 is I_neg_curv, I3 is I_image
  // NOTE(sanja): I have a terrible feeling something here is transposed.
  for (int i = 0; i < n_edge; ++i) {
    edge_data[I_edge[i]][0] = 1;
  }  
  for (int i = 0; i < n_neg_curv; ++i) {
    edge_data[I_neg_curv[i]][1] = MIN(1, .4 * abs(C[i]) / curv_thresh);
  }  
  for (int i = 0; i < n_image; ++i) {
    edge_data[I_image[i]][2] = 1;
  }  

  // Add everything to the cloud
  int old_n = pcd.num_channels; // NOTE(sanja): I am now realizing that this is a misnomer throughout a code (I used n for num_points). Should be fixed once there is time.
  pcd.num_channels += 3;
  /*safe_realloc(pcd.channels, pcd.num_channels, char*);
  
  safe_calloc(pcd.channels[old_n], 30, char);
  safe_calloc(pcd.channels[old_n + 1], 30, char);
  safe_calloc(pcd.channels[old_n + 2], 30, char);*/
  
  sprintf(pcd.channels[old_n + 0], "range_edge");
  sprintf(pcd.channels[old_n + 1], "curv_edge");
  sprintf(pcd.channels[old_n + 2], "img_edge");

  add_rows_matrix2(&pcd.data, old_n, pcd.num_points, pcd.num_channels);
  
  for (int i = 0; i < pcd.num_points; ++i) {
    pcd.data[old_n + 0][i] = edge_data[i][0];
    pcd.data[old_n + 1][i] = edge_data[i][1];
    pcd.data[old_n + 2][i] = edge_data[i][2];
  } 
  pcd.range_edge = pcd.data[old_n];
  pcd.curv_edge = pcd.data[old_n + 1];
  pcd.img_edge = pcd.data[old_n + 2];
}

void filter_pcl(pcl::PointCloud<pcl::PointXYZ> &pcd_out, vector<int> &I, pcl::PointCloud<pcl::PointXYZ> pcd_in) {
  pcd_out.header = pcd_in.header;
  for (int i = 0; i < pcd_in.points.size(); ++i) {
    if (isFinite(pcd_in.points[i])) {
      pcd_out.points.push_back(pcd_in.points[i]);
      I.push_back(i);
    }
  } 
  pcd_out.width = pcd_out.points.size();
  pcd_out.height = 1;
}

void filter_pcd_in_place(pcd_t &pcd) {
  int outliers[pcd.num_points];
  memset(outliers, 0, pcd.num_points * sizeof(int));
  
  for (int i = 0; i < pcd.num_points; ++i) {
    if (isnan(pcd.points[i][0]) || isnan(pcd.points[i][1]) || isnan(pcd.points[i][2])) {
      outliers[i] = 1;
    }
  } 
  remove_outliers_pt_norm_pc(pcd, outliers);
}

void find_all_the_features(pcd_t &pcd_bg_full, pcd_t &pcd_objects_full, cardboard::Scope::Request &req) {
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_bg_full;
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_objects_full;
  pcl::fromROSMsg(req.cloud_bg, pcl_cloud_bg_full);
  pcl::fromROSMsg(req.cloud_objects, pcl_cloud_objects_full);
  pcd_bg_full.quaternions = NULL;

  safe_calloc(pcd_bg_full.channels, 100, char*);
  for (int i = 0; i < 100; ++i)
    safe_calloc(pcd_bg_full.channels[i], 30, char);

  double t0 = get_time_ms();
  pclxyzrgb_to_bingham_pcd(pcd_bg_full, pcl_cloud_bg_full);
  printf("Converted to Bingham PCD in %.3f seconds\n", (get_time_ms() - t0) / 1000.0);
  
  // Calculate all the features
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_bg;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_objects;
  pcl::fromROSMsg(req.cloud_bg, pcl_cloud_bg);
  pcl::fromROSMsg(req.cloud_objects, pcl_cloud_objects);
  
  pcl::PointCloud<pcl::PointNormal> pcl_cloud_bg_normals;

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_bg_filtered;
  vector<int> I_filtered;

  filter_pcl(pcl_cloud_bg_filtered, I_filtered, pcl_cloud_bg);
  cardboard::compute_normals(pcl_cloud_bg_filtered, pcl_cloud_bg_normals, 0.015); // NOTE(sanja): Hardcoded radius...
  t0 = get_time_ms();
  pcd_add_normals_idx(pcd_bg_full, pcl_cloud_bg_normals, I_filtered); // NOTE(sanja): God, this is awful.
  printf("Found normals in %.3f seconds\n", (get_time_ms() - t0) / 1000.0);

  t0 = get_time_ms();
  int *I_neg;
  int n_neg;
  double *C_thin;
  pcd_compute_principal_curvatures_find_negative_curvature_points(pcd_bg_full, I_neg, n_neg, C_thin, pcl_cloud_bg_full.height, pcl_cloud_bg_full.width);
  printf("Found principal curvatures in %.3f seconds\n", (get_time_ms() - t0) / 1000.0);
  filter_pcd_in_place(pcd_bg_full);

  pcd_bg_full.quaternions = new_matrix2(pcd_bg_full.num_points, 4);
  compute_orientation_quaternions(pcd_bg_full.quaternions, pcd_bg_full.normals, pcd_bg_full.principal_curvatures, pcd_bg_full.num_points);

  t0 = get_time_ms();
  double vp[7];
  vp[0] = pcl_cloud_bg_full.sensor_origin_[0];
  vp[1] = pcl_cloud_bg_full.sensor_origin_[1];
  vp[2] = pcl_cloud_bg_full.sensor_origin_[2];
  vp[3] = pcl_cloud_bg_full.sensor_orientation_.w();
  vp[4] = pcl_cloud_bg_full.sensor_orientation_.x();
  vp[5] = pcl_cloud_bg_full.sensor_orientation_.y();
  vp[6] = pcl_cloud_bg_full.sensor_orientation_.z();
  add_pcd_edge_features(pcd_bg_full, I_neg, n_neg, C_thin, vp);
  
  pcl::PointCloud<pcl::PointNormal> pcl_objects_normals;
  cardboard::compute_normals(pcl_cloud_objects, pcl_objects_normals, 0.015); // NOTE(sanja): Hardcoded radius...
  pcl::PointCloud<pcl::PrincipalCurvatures> pcl_pcs_cloud;
  cardboard::compute_principal_curvatures(pcl_cloud_objects, pcl_objects_normals, pcl_pcs_cloud, .015);
  pcl::PointCloud<pcl::FPFHSignature33> pcl_fpfh_cloud;
  cardboard::compute_fpfhs(pcl_cloud_objects, pcl_objects_normals, pcl_fpfh_cloud, .03);
  printf("Calculated objects features in %.3f seconds\n", (get_time_ms() - t0) / 1000.0);

  t0 = get_time_ms();

  safe_calloc(pcd_objects_full.channels, 100, char*);
  for (int i = 0; i < 100; ++i)
    safe_calloc(pcd_objects_full.channels[i], 30, char);


  pclxyzrgb_to_bingham_pcd(pcd_objects_full, pcl_cloud_objects_full);
  pcd_add_normals(pcd_objects_full, pcl_objects_normals);
  pcd_add_curvature(pcd_objects_full, pcl_pcs_cloud);
  pcd_add_fpfhs(pcd_objects_full, pcl_fpfh_cloud);

  pcd_objects_full.quaternions = new_matrix2(pcd_objects_full.num_points, 4);
  compute_orientation_quaternions(pcd_objects_full.quaternions, pcd_objects_full.normals, pcd_objects_full.principal_curvatures, pcd_objects_full.num_points);
  printf("Converted objects cloud to PCD in %.3f seconds\n", (get_time_ms() - t0) / 1000.0);

  //save_pcd("objects.pcd", pcd_objects_full);
}
