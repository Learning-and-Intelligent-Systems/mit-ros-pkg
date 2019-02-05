#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <bingham/util.h>
#include <flann/flann.h>


double llsample_kdtree(kdtree_t *X_kdtree, double *Y, double *x)
{
  int nn_idx = kdtree_NN(X_kdtree, x);

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


void simulate(double **Q_sim, double **dQ_sim, double *T, double *q0, double *dq0, double **U,
	      int n, int d, double ***X, double **Y, int xn,
	      flann_index_t *X_indices, FLANNParameters *flann_params, kdtree_t **X_kdtrees)
{
  double r = .05;  // kernel width

  double q[d], dq[d], ddq[d];
  
  memcpy(q, q0, d*sizeof(double));
  memcpy(dq, dq0, d*sizeof(double));
  memcpy(Q_sim[0], q0, d*sizeof(double));
  memcpy(dQ_sim[0], dq0, d*sizeof(double));

  for (int i = 1; i < n; i++) {
    double dt = T[i] - T[i-1];

    double query[d];
    memcpy(&query[0], q, d*sizeof(double));
    memcpy(&query[d], dq, d*sizeof(double));
    for (int j = 0; j < d; j++) {
      query[2*d] = U[i-1][j];
      //ddq[j] = llsample(X[j], Y[j], query, r, xn, 2*d+1);
      ddq[j] = llsample_flann(X[j], Y[j], query, r, xn, 2*d+1, X_indices[j], flann_params);
      //llsample_kdtree(X_kdtrees[j], Y[j], query);
    }

    for (int j = 0; j < d; j++) {
      q[j] += dt*dq[j];
      dq[j] += dt*ddq[j];
    }

    memcpy(Q_sim[i], q, d*sizeof(double));
    memcpy(dQ_sim[i], dq, d*sizeof(double));
  }
}


double ***build_lookup_tables(double **Q, double **dQ, double **U, int n, int d)
{
  double ***X;
  safe_malloc(X, d, double**);  // build one lookup table for each joint

  for (int j = 0; j < d; j++) {  // j for joint
    X[j] = new_matrix2(n, 2*d+1);
    for (int i = 0; i < n; i++) {
      memcpy(&X[j][i][0], Q[i], d*sizeof(double));
      memcpy(&X[j][i][d], dQ[i], d*sizeof(double));
      X[j][i][2*d] = U[i][j];
    }
  }

  return X;
}



int main(int argc, char *argv[])
{
  printf("loading data...\n");

  // load swing data
  int n,d;
  double **T_mat = load_matrix("T.txt", &n, &d);
  double *T = T_mat[0];
  double **Q = load_matrix("Q.txt", &n, &d);
  double **dQ = load_matrix("dQ.txt", &n, &d);
  double **ddQ = load_matrix("ddQ.txt", &n, &d);
  double **U = load_matrix("U.txt", &n, &d);

  // just model the first 3 joints
  double ***X = build_lookup_tables(Q, dQ, U, n, 3);
  double **Y = new_matrix2(3,n);
  transpose(Y, ddQ, n, 3);

  //dbug
  //n = 1000;

  // build flann indices
  printf("Building FLANN indices\n");
  flann_set_distance_type(FLANN_DIST_EUCLIDEAN, 0);
  FLANNParameters flann_params = DEFAULT_FLANN_PARAMETERS;
  flann_params.algorithm = FLANN_INDEX_KDTREE;
  flann_params.trees = 8;
  flann_params.checks = 64;
  float speedup;
  flann_index_t X_indices[3];
  for (int j = 0; j < 3; j++)
    X_indices[j] = flann_build_index_double(X[j][0], n, 7, &speedup, &flann_params);

  //n = 1000; //dbug

  // build kdtrees
  printf("Building kdtrees\n");
  kdtree_t *X_kdtrees[3];
  for (int j = 0; j < 3; j++)
    X_kdtrees[j] = kdtree(X[j], n, 7);

  // simulate first second of data
  int NT = 500;
  double **Q_sim = new_matrix2(NT, 3);
  double **dQ_sim = new_matrix2(NT, 3);

  printf("simulating trajectory...\n");

  double t0 = get_time_ms();
  //for (int i = 0; i < 100; i++)
  simulate(Q_sim, dQ_sim, T, Q[0], dQ[0], U, NT, 3, X, Y, n, X_indices, &flann_params, X_kdtrees);
  double t = get_time_ms();

  printf("simulated %d timesteps in %.4f ms\n", NT, t-t0);

  return 0;
}
