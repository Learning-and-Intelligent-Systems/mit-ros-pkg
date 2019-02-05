#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <bingham/util.h>
#include <flann/flann.h>
#include <lapacke.h>



//---------------------- nn_forest_3d ----------------------//


struct nn_forest_3d {
  kdtree_t **trees;
  int **idx;
  int *cnt;
  int dims[3];
  double gmin[3];
  double res[3];
};



inline int nn_forest_cell_index(nn_forest_3d *forest, double *x)
{
  int ci[3];
  for (int i = 0; i < 3; i++)
    ci[i] = (int)floor((x[i] - forest->gmin[i]) / forest->res[i]);
  
  int d1 = forest->dims[1];
  int d2 = forest->dims[2];

  return ci[0]*d1*d2 + ci[1]*d2 + ci[2];
}

nn_forest_3d *build_nn_forest(double **X, int n, int d)
{
  // allocate forest (kdtree grid)
  nn_forest_3d *forest;
  safe_calloc(forest, 1, nn_forest_3d);
  for (int i = 0; i < 3; i++) {
    forest->dims[i] = 10;
    forest->gmin[i] = -M_PI;
    forest->res[i] = M_PI/5.;
  }
  forest->dims[1] = 1000;
  forest->gmin[1] = -M_PI;
  forest->res[1] = M_PI/500.;
  

  int grid_size = forest->dims[0] * forest->dims[1] * forest->dims[2];
  safe_calloc(forest->trees, grid_size, kdtree_t*);
  safe_calloc(forest->idx, grid_size, int*);
  safe_calloc(forest->cnt, grid_size, int);

  // get the grid cell counts
  for (int i = 0; i < n; i++) {
    int c = nn_forest_cell_index(forest, X[i]);
    forest->cnt[c]++;
  }
  
  // allocate grid indices
  int num_cells = 0;
  for (int c = 0; c < grid_size; c++) {
    if (forest->cnt[c] > 0) {
      safe_calloc(forest->idx[c], forest->cnt[c], int);
      num_cells++;
    }
  }
  printf("num_cells = %d\n", num_cells);  //dbug

  // get the indices of X in each grid cell
  memset(forest->cnt, 0, grid_size*sizeof(int));  // reset cell counts
  for (int i = 0; i < n; i++) {
    int c = nn_forest_cell_index(forest, X[i]);
    int j = forest->cnt[c];
    forest->idx[c][j] = i;
    forest->cnt[c]++;
  }

  // make kdtrees for each non-empty grid cell
  int n2 = imax(forest->cnt, grid_size);
  double **X2 = new_matrix2(n2,d);
  for (int c = 0; c < grid_size; c++) {
    if (forest->cnt[c] > 0) {
      reorder_rows(X2, X, forest->idx[c], forest->cnt[c], d);
      forest->trees[c] = kdtree(X2, forest->cnt[c], d);
    }
  }
  free_matrix2(X2);
  
  printf("max cell cnt = %d\n", n2);  //dbug

  return forest;
}


int nn_forest_search(nn_forest_3d *forest, double *x)
{
  int c = nn_forest_cell_index(forest, x);

  if (forest->trees[c] == NULL)
    return -1;

  int ci = kdtree_NN(forest->trees[c], x);

  return forest->idx[c][ci];
}


//-------------------------------------------------------------------------//


/*
// create a new n-by-m 2d matrix of doubles on the stack
double **new_matrix2_stack(double *data, double **ptrs, int n, int m)
{
  int i;
  double *raw = data;
  double **X = ptrs;
  //safe_calloc(raw, n*m, double);
  //safe_malloc(X, n, double*);

  for (i = 0; i < n; i++)
    X[i] = raw + m*i;

  return X;
}
*/

// create a new n-by-m 2d matrix of floats
float **new_matrix2f(int n, int m)
{
  if (n*m == 0) return NULL;
  int i;
  float *raw, **X;
  safe_calloc(raw, n*m, float);
  safe_malloc(X, n, float*);

  for (i = 0; i < n; i++)
    X[i] = raw + m*i;

  return X;
}

// free a 2d matrix of floats
void free_matrix2f(float **X)
{
  if (X == NULL) return;
  free(X[0]);
  free(X);
}



void weighted_regression(double *b, double **X, double *y, double *w, int n, int d)
{
  /*
  static double Xtw_data[30*8];
  static double *Xtw_ptrs[30];
  static double XtwX_data[8*8];
  static double *XtwX_ptrs[8];
  static int first = 1;
  if (first) {
    first = 0;
    new_matrix2_stack(Xtw_data, Xtw_ptrs, 30, 8);
    new_matrix2_stack(XtwX_data, XtwX_ptrs, 8, 8);
  }
  */

  //double **Xtw = Xtw_ptrs;
  double **Xtw = new_matrix2(d,n);
  transpose(Xtw,X,n,d);
  for (int i = 0; i < d; i++)
    for (int j = 0; j < n; j++)
      Xtw[i][j] *= w[j];

  //double **XtwX = XtwX_ptrs;
  double **XtwX = new_matrix2(d,d);
  matrix_mult(XtwX,Xtw,X,d,n,d);

  double *Xtwy = b;
  matrix_vec_mult(Xtwy,Xtw,y,d,n);

  int pivots[d];
  LAPACKE_dgesv(LAPACK_COL_MAJOR, d, 1, XtwX[0], d, pivots, Xtwy, d);

  free_matrix2(Xtw);
  free_matrix2(XtwX);
}


double llsample_kdtree(kdtree_t *X_kdtree, double *Y, double *x)
{
  int nn_idx = kdtree_NN(X_kdtree, x);

  double y = Y[nn_idx];

  return y;
}


double llsample_forest(nn_forest_3d *X_forest, double *Y, double *x)
{
  int nn_idx = nn_forest_search(X_forest, x);

  double y = Y[nn_idx];

  return y;
}


/*
void find_nn(flann_index_t X_index, double *x, int *nn_idx, double *nn_d2, int knn,
	     FLANNParameters *flann_params)
{
  flann_find_nearest_neighbors_index_double(X_index, x, 1, nn_idx, nn_d2, knn, flann_params);
}
*/

double llsample_flann_single(float *x, double *Y, flann_index_t X_index, FLANNParameters *flann_params)
{
  int knn = 1;
  int nn_idx[knn];
  float nn_d2[knn];
  flann_find_nearest_neighbors_index(X_index, x, 1, nn_idx, nn_d2, knn, flann_params);

  double y = Y[nn_idx[0]];

  return y;
}


double llsample_flann(double **X, double *Y, double *x, double r, int n, int d,
		      flann_index_t X_index, FLANNParameters *flann_params)
{
  int knn = 30;

  double r2 = r*r;

  int nn_idx[knn];
  double nn_d2[knn];
  flann_find_nearest_neighbors_index_double(X_index, x, 1, nn_idx, nn_d2, knn, flann_params);
  //find_nn(X_index, x, nn_idx, nn_d2, knn, flann_params);
  //memset(nn_idx, 0, knn*sizeof(int));
  //memset(nn_d2, 0, knn*sizeof(double));

  double logw[knn];
  for (int i = 0; i < knn; i++) {
    logw[i] = -nn_d2[i] / r2;
  }
  //printf("nn_d2[0] = %f, dist2[0] = %f\n", nn_d2[0], dist2(X[nn_idx[0]], x, d));
  //printf("nn_d2[1] = %f, dist2[1] = %f\n", nn_d2[1], dist2(X[nn_idx[1]], x, d));

  //double cutoff = log(.02);  // truncate weights less than 2% of the max
  double offset = logw[0]; //max(logw, knn);
  double w[knn];
  for (int i = 0; i < knn; i++) {
    w[i] = exp(logw[i] - offset);
    //double dw = logw[i] - offset;
    //w[i] = (dw < cutoff ? 0.0 : exp(dw));
  }

  double y = 0;
  double wtot = 0.0;
  for (int i = 0; i < knn; i++) {
    //if (w[i] != 0.0) {
    y += w[i]*Y[nn_idx[i]];
    wtot += w[i];
    //}
  }

  return y / wtot;
}

double llsample(double **X, double *Y, double *x, double r, int n, int d)
{
  double r2 = r*r;

  double logw[n];
  for (int i = 0; i < n; i++)
    logw[i] = -dist2(X[i], x, d) / r2;

  double cutoff = log(.02);  // truncate weights less than 2% of the max
  double offset = max(logw, n);
  double w[n];
  for (int i = 0; i < n; i++) {
    double d = logw[i] - offset;
    w[i] = (d < cutoff ? 0.0 : exp(d));
  }

  double y = 0;
  double wtot = 0.0;
  for (int i = 0; i < n; i++) {
    if (w[i] != 0.0) {
      y += w[i]*Y[i];
      wtot += w[i];
    }
  }

  return y / wtot;
}


/*
 * Locally-Weighted Regression.  Note that b will contain (d+1) regression coefficients.
 * Returns y = dot([1,x],b)
 */
double lwr(double *b, double **X, double *Y, double *x, double r, int n, int d)
{
  double r2 = r*r;

  double logw[n];
  for (int i = 0; i < n; i++)
    logw[i] = -dist2(X[i], x, d) / r2;

  double cutoff = log(.02);  // truncate weights less than 2% of the max
  double offset = max(logw, n);
  double w[n];
  int n2 = 0;
  for (int i = 0; i < n; i++) {
    double d = logw[i] - offset;
    if (d < cutoff)
      w[i] = 0.0;
    else {
      w[i] = exp(d);
      n2++;
    }
  }
  double **X2 = new_matrix2(n2,d+1);
  double Y2[n2];
  double w2[n2];
  n2 = 0;
  for (int i = 0; i < n; i++) {
    if (w[i] != 0.0) {
      X2[n2][0] = 1;
      memcpy(&X2[n2][1], X[i], d*sizeof(double));
      Y2[n2] = Y[i];
      w2[n2] = w[i];
      n2++;
    }
  }

  weighted_regression(b, X2, Y2, w2, n2, d+1);

  free_matrix2(X2);

  double y = b[0] + dot(&b[1], x, d);

  return y;
}

/*
 * Locally-Weighted Regression (with FLANN).  Note that b will contain (d+1) regression coefficients.
 * Returns y = dot([1,x],b)
 */
double lwr_flann(double *b, double **X, double *Y, double *x, double r, int n, int d,
		 flann_index_t X_index, FLANNParameters *flann_params)
{
  //double X2_data[30*8];
  //double *X2_ptrs[30];
  //new_matrix2_stack(X2_data, X2_ptrs, 30, 8);

  int knn = 30;

  double r2 = r*r;

  int nn_idx[knn];
  double nn_d2[knn];
  flann_find_nearest_neighbors_index_double(X_index, x, 1, nn_idx, nn_d2, knn, flann_params);

  double logw[knn];
  for (int i = 0; i < knn; i++)
    logw[i] = -nn_d2[i] / r2;
  double offset = max(logw, knn);

  //double **X2 = X2_ptrs;
  double **X2 = new_matrix2(knn,d+1);
  double Y2[knn];
  double w[knn];
  for (int i = 0; i < knn; i++) {
    X2[i][0] = 1;
    memcpy(&X2[i][1], X[nn_idx[i]], d*sizeof(double));
    Y2[i] = Y[nn_idx[i]];
    w[i] = exp(logw[i] - offset);
  }

  weighted_regression(b, X2, Y2, w, knn, d+1);

  free_matrix2(X2);

  double y = b[0] + dot(&b[1], x, d);

  return y;
}


/*
double ylookup(double **X, double *Y, double *x, double r, int n, int d,
	       kdtree_t *X_kdtree, flann_index_t X_index, FLANNParameters *flann_params)
{

}
*/

void simulate(double **Q_sim, double **dQ_sim, double *T, double *q0, double *dq0, double **U,
	      int n, int d, float ***X, double **Y, int xn, flann_index_t *X_indices, FLANNParameters *flann_params)
//	      kdtree_t **X_kdtrees, nn_forest_3d **X_forests)
{
  double r = .05;  // kernel width

  double q[d], dq[d], ddq[d];
  
  memcpy(q, q0, d*sizeof(double));
  memcpy(dq, dq0, d*sizeof(double));
  memcpy(Q_sim[0], q0, d*sizeof(double));
  memcpy(dQ_sim[0], dq0, d*sizeof(double));

  for (int i = 1; i < n; i++) {
    double dt = T[i] - T[i-1];

    double b[2*d+2];
    float query[d];
    for (int j = 0; j < d; j++)
      query[j] = q[j];
    for (int j = 0; j < d; j++)
      query[d+j] = dq[j];
    for (int j = 0; j < d; j++) {
      query[2*d] = U[i-1][j];
      //ddq[j] = lwr(b, X[j], Y[j], query, r, xn, 2*d+1);
      //ddq[j] = lwr_flann(b, X[j], Y[j], query, r, xn, 2*d+1, X_indices[j], flann_params);
      //ddq[j] = llsample(X[j], Y[j], query, r, xn, 2*d+1);
      //ddq[j] = llsample_flann(X[j], Y[j], query, r, xn, 2*d+1, X_indices[j], flann_params);
      ddq[j] = llsample_flann_single(query, Y[j], X_indices[j], flann_params);
      //llsample_kdtree(X_kdtrees[j], Y[j], query);
      //llsample_forest(X_forests[j], Y[j], query);
    }

    for (int j = 0; j < d; j++) {
      q[j] += dt*dq[j];
      dq[j] += dt*ddq[j];
    }

    memcpy(Q_sim[i], q, d*sizeof(double));
    memcpy(dQ_sim[i], dq, d*sizeof(double));
  }
}


float ***build_lookup_tables(double **Q, double **dQ, double **U, int n, int d)
{
  float ***X;
  safe_malloc(X, d, float**);  // build one lookup table for each joint

  for (int j = 0; j < d; j++) {  // j for joint
    X[j] = new_matrix2f(n, 2*d+1);
    for (int i = 0; i < n; i++) {
      for (int k = 0; k < d; k++)
	X[j][i][k] = Q[i][k];
      for (int k = 0; k < d; k++)
	X[j][i][d+k] = dQ[i][k];
      X[j][i][2*d] = U[i][j];
    }
  }

  return X;
}









void test_lapacke()
{
  printf("testing lapacke...\n");

  int n = 3, d = 2;
  double **X = new_matrix2(n,d);
  double x[] = {1,2, 1,3, 1,4};
  memcpy(X[0], x, n*d*sizeof(double));

  double y[] = {3,4,10};
  double w[] = {1,1,10};
  double b[d];

  weighted_regression(b, X, y, w, n, d);

  printf("b = [%.4f, %.4f]\n", b[0], b[1]);
}


int main(int argc, char *argv[])
{
  test_lapacke();

  printf("loading data...\n");

  // load swing data
  int n,d;
  double **T_mat = load_matrix("T.txt", &n, &d);
  double *T = T_mat[0];
  double **Q = load_matrix("Q.txt", &n, &d);
  double **dQ = load_matrix("dQ.txt", &n, &d);
  double **ddQ = load_matrix("ddQ.txt", &n, &d);
  double **U = load_matrix("U.txt", &n, &d);

  n = 4096; //dbug
  printf("n = %d\n", n);

  // just model the first 3 joints
  float ***X = build_lookup_tables(Q, dQ, U, n, 3);
  double **Y = new_matrix2(3,n);
  transpose(Y, ddQ, n, 3);

  // build flann indices
  printf("Building FLANN indices\n");
  flann_set_distance_type(FLANN_DIST_EUCLIDEAN, 0);
  FLANNParameters flann_params = DEFAULT_FLANN_PARAMETERS;
  flann_params.algorithm = FLANN_INDEX_KDTREE_SINGLE;  //FLANN_INDEX_KDTREE;
  //flann_params.trees = 8;
  //flann_params.checks = 64;
  float speedup;
  flann_index_t X_indices[3];
  for (int j = 0; j < 3; j++)
    X_indices[j] = flann_build_index(X[j][0], n, 7, &speedup, &flann_params);
    //X_indices[j] = flann_build_index_double(X[j][0], n, 7, &speedup, &flann_params);

  /*
  // build kdtrees
  printf("Building kdtrees\n");
  kdtree_t *X_kdtrees[3];
  for (int j = 0; j < 3; j++)
    X_kdtrees[j] = kdtree(X[j], n, 7);

  printf("Building forests\n");
  nn_forest_3d *X_forests[3];
  for (int j = 0; j < 3; j++)
    X_forests[j] = build_nn_forest(X[j], n, 7);
  */

  // simulate first second of data
  int NT = 4000; //500;
  double **Q_sim = new_matrix2(NT, 3);
  double **dQ_sim = new_matrix2(NT, 3);

  printf("simulating trajectory...\n");

  double t0 = get_time_ms();
  int num_sims = 10;
  for (int i = 0; i < num_sims; i++)
    simulate(Q_sim, dQ_sim, T, Q[i], dQ[i], U, NT, 3, X, Y, n, X_indices, &flann_params); //, X_kdtrees, X_forests);
  double t = get_time_ms();

  printf("simulated %dx%d timesteps in %.4f ms\n", num_sims, NT, t-t0);

  printf("q_final: (%.2f, %.2f, %.2f)\n", Q_sim[NT-1][0], Q_sim[NT-1][1], Q_sim[NT-1][2]);
  printf("dq_final: (%.2f, %.2f, %.2f)\n", dQ_sim[NT-1][0], dQ_sim[NT-1][1], dQ_sim[NT-1][2]);

  return 0;
}
