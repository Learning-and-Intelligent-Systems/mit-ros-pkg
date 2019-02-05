 #include <math.h>
 #include <stdlib.h>
 #include <stdio.h>
 #include <string.h>
 #include <vector>
 #include <bingham/util.h>
 //#include <flann/flann.h>
 #include "dynamics.h"

 // warning: keep this at the bottom of the includes!
 #include <lapacke.h>


 using namespace std;
 using namespace fastwam;
 //using namespace pingpong;




 static void nn_map_find_knn(int *nn_idx, double *nn_d2, nn_map_t *map, double *x, int k)
 {
   double *d2;
   safe_calloc(d2, map->n, double);
   for (int i = 0; i < map->n; i++)
     d2[i] = dist2(x, &map->X[i*map->dx], map->dx);
   mink(d2, nn_idx, map->n, k);
   reorder(nn_d2, d2, nn_idx, k);
   free(d2);
 }



 //TODO: add to libbingham
 static void weighted_regression(double *b, double **X, double *y, double *w, int n, int d)
 {
   double **Xtw = new_matrix2(d,n);
   transpose(Xtw,X,n,d);
   for (int i = 0; i < d; i++)
     for (int j = 0; j < n; j++)
       Xtw[i][j] *= w[j];

   double **XtwX = new_matrix2(d,d);
   matrix_mult(XtwX,Xtw,X,d,n,d);

   double *Xtwy = b;
   matrix_vec_mult(Xtwy,Xtw,y,d,n);

   int pivots[d];
   LAPACKE_dgesv(LAPACK_COL_MAJOR, d, 1, XtwX[0], d, pivots, Xtwy, d);

   free_matrix2(Xtw);
   free_matrix2(XtwX);
 }


 static void weighted_ridge_regression(double *b, double **X, double *y, double *w, double lambda, int n, int d)
 {
   double **Xtw = new_matrix2(d,n);
   transpose(Xtw,X,n,d);
   for (int i = 0; i < d; i++)
     for (int j = 0; j < n; j++)
       Xtw[i][j] *= w[j];

   double **XtwX = new_matrix2(d,d);
   matrix_mult(XtwX,Xtw,X,d,n,d);

   double *Xtwy = b;
   matrix_vec_mult(Xtwy,Xtw,y,d,n);

   for (int i = 0; i < d; i++)
     XtwX[i][i] += lambda;

   int pivots[d];
   LAPACKE_dgesv(LAPACK_COL_MAJOR, d, 1, XtwX[0], d, pivots, Xtwy, d);

   free_matrix2(Xtw);
   free_matrix2(XtwX);
 }


 static void iterative_weighted_ridge_regression(double *b, double **X, double *y, double *w, double lambda, int n, int d)
 {
   weighted_ridge_regression(b, X, y, w, lambda, n, d);

   double r = 5;
   double r2 = r*r;

   int iter, max_iter = 20;
   for (iter = 0; iter < max_iter; iter++) {
     int i;
     double points_d2[n], weights[n];
     for (i = 0; i < n; i++) {
       double y_proj = b[0] + dot(&b[1], X[i], d);
       double bn = norm(&b[1],d);
       points_d2[i] = (y_proj - y[i])*(y_proj - y[i]) / (bn*bn + 1);
       weights[i] = exp(-points_d2[i] / r2) * w[i];
     }
     weighted_ridge_regression(b, X, y, weights, lambda, n, d);
   }
 }


 static void build_nn_map(nn_map_t *map, double *X, double *Y, int n, int dx, int dy)
 {
   // set map data pointers
   map->X = X;
   map->Y = Y;
   map->n = n;
   map->dx = dx;
   map->dy = dy;

   // create flann index
   //flann_set_distance_type(FLANN_DIST_EUCLIDEAN, 0);
   //map->params = DEFAULT_FLANN_PARAMETERS;
   //map->params.algorithm = FLANN_INDEX_KDTREE_SINGLE;
   //float speedup;
   //map->index = flann_build_index_double(X, n, dx, &speedup, &map->params);
 }


 //TODO: Fix memory leak!
 static void nn_map_add_data(nn_map_t *map, double *X, double *Y, int n, int dx, int dy)
 {
   //dbug: check for NaN
   for (int i = 0; i < n*dx; i++)
     if (isnan(X[i]))
       printf("X[%d] = %f\n", i, X[i]);
   for (int i = 0; i < n*dy; i++)
     if (isnan(Y[i]))
       printf("Y[%d] = %f\n", i, Y[i]);


   if (map->n == 0) {
     build_nn_map(map, X, Y, n, dx, dy);
     return;
   }

   // append data matrices
   int n2 = n + map->n;
   double **X2 = new_matrix2(n2, dx);
   memcpy(X2[0], map->X, dx*map->n*sizeof(double));
   memcpy(X2[map->n], X, dx*n*sizeof(double));
   double **Y2 = new_matrix2(n2, dy);
   memcpy(Y2[0], map->Y, dy*map->n*sizeof(double));
   memcpy(Y2[map->n], Y, dy*n*sizeof(double));

   // free old map data
   //if (map->X) {
   //  free(map->X);
   //  free(map->Y);
   //}
   //flann_free_index(map->index, &map->params);

   // set new map data pointers
   map->X = X2[0];
   map->Y = Y2[0];
   map->n = n2;

   /*dbug
   double v[dx];
   memset(v, 0, dx*sizeof(double));
   for (int i = 0; i < n2; i++)
     for (int j = 0; j < dx; j++)
       v[j] += X2[i][j]*X2[i][j];
   mult(v, v, 1/(double)n2, dx);
   //variance(v, X2, n2, dx);
   printf("std(X2) = [ ");
   for (int i = 0; i < dx; i++)
     printf("%.2f ", sqrt(v[i]));
   printf("]\n");
   */

   // create new flann index
   //flann_set_distance_type(FLANN_DIST_EUCLIDEAN, 0);
   //map->params = DEFAULT_FLANN_PARAMETERS;
   //map->params.algorithm = FLANN_INDEX_KDTREE_SINGLE;
   //float speedup;
   //map->index = flann_build_index_double(map->X, n2, dx, &speedup, &map->params);
 }


 /*
  * Find the nearest neighbor to x in map and compute its corresponding y-value.
  * Returns the index of x's nearest neighbor.
  */
 static int nn_map_lookup(double *y, nn_map_t *map, double *x)
 {
   int nn_idx;
   double nn_d2 = 1e16;

   //flann_find_nearest_neighbors_index_double(map->index, x, 1, &nn_idx, &nn_d2, 1, &map->params);

   for (int i = 0; i < map->n; i++) {
     double d2 = dist2(x, &map->X[i*map->dx], map->dx);
     if (d2 < nn_d2) {
       nn_d2 = d2;
       nn_idx = i;
     }
   }

   if (y)
     memcpy(y, &map->Y[nn_idx * map->dy], map->dy*sizeof(double));

   return nn_idx;
 }


 /*
  * Find the k-nearest neighbors to x in map and compute a weighted average of their corresponding y-values.
  */
 static void knn_map_lookup_smooth(double *y, nn_map_t *map, double *x, int k, double r)
 {
   //dbug: check for NaN
   for (int j = 0; j < map->dx; j++) {
     if (isnan(x[j])) {
       printf("x[%d] = %f\n", j, x[j]);
       exit(1);
     }
   }


   double r2 = r*r;

   int nn_idx[k];
   double nn_d2[k];
   nn_map_find_knn(nn_idx, nn_d2, map, x, k);
   //flann_find_nearest_neighbors_index_double(map->index, x, 1, nn_idx, nn_d2, k, &map->params);

   if (nn_idx[0] < 0 || nn_idx[0] >= map->n) {
     printf("Error: nn_idx[0] is bad!\n");
     exit(1);
   }
   for (int i = 1; i < k; i++)
     if (nn_idx[i] < 0 || nn_idx[i] >= map->n)
       k = i;

   double logw[k];
   for (int i = 0; i < k; i++)
     logw[i] = -nn_d2[i] / r2;
   double logw_max = arr_max(logw, k);
   double w[k];
   for (int i = 0; i < k; i++)
     w[i] = exp(logw[i] - logw_max);
   normalize_pmf(w, w, k);

   memset(y, 0, map->dy*sizeof(double));
   for (int i = 0; i < k; i++) {
     //printf("nn_idx[%d] = %d\n", i, nn_idx[i]); //dbug
     for (int j = 0; j < map->dy; j++) {
       //printf("  j = %d\n", j); //dbug
       y[j] += map->Y[nn_idx[i]*map->dy + j] * w[i];
     }
   }
 }


 /*
  * Sparsify the rows in X (n-by-d) with squared-distance threshold d2_thresh.
  */
 static int sparsify_points(double ***X2, double **X, int n, int d, double d2_thresh)
 {
   int idx[n];
   idx[0] = 0;  // always include the first point
   int cnt = 1;

   //printf("sparsifying %d points...\n", n);

   for (int i = 1; i < n; i++) {
     double d2min = 2*d2_thresh + 1.0;  // needs to be bigger than d2_thresh
     for (int j = 0; j < cnt; j++) {
       double d2 = dist2(X[i], X[idx[j]], d);
       if (d2 < d2min)
	 d2min = d2;
     }
     if (d2min > d2_thresh)  // add the point if no other point is nearby
       idx[cnt++] = i;

     //printf("d2min = %f\n", d2min);
   }

   //printf("...keeping %d points\n", cnt);

   if (cnt > 0) {
     *X2 = new_matrix2(cnt, d);
     reorder_rows(*X2, X, idx, cnt, d);
   }

   return cnt;
 }


 /*
  * Given an existing sparse map, sparsify a new matrix X.
  * Returns 
  */
 static int sparsify_points_with_map(double ***X2, nn_map_t *sparse_map, double **X, int n, int d, double d2_thresh)
 {
   if (sparse_map == NULL || sparse_map->n == 0)
     return sparsify_points(X2, X, n, d, d2_thresh);

   int idx[n];
   int cnt = 0;

   //printf("sparsifying %d points...\n", n);

   for (int i = 0; i < n; i++) {

     int nn_idx = nn_map_lookup(NULL, sparse_map, X[i]);
     double d2min = dist2(X[i], &sparse_map->X[d*nn_idx], d);

     if (d2min > d2_thresh) {
       for (int j = 0; j < cnt; j++) {
	 double d2 = dist2(X[i], X[idx[j]], d);
	 if (d2 < d2min)
	   d2min = d2;
       }
       if (d2min > d2_thresh)  // add the point if no other point is nearby
	 idx[cnt++] = i;
     }

     //printf("d2min = %f\n", d2min);
   }

   //printf("...keeping %d points\n", cnt);

   *X2 = new_matrix2(cnt, d);
   reorder_rows(*X2, X, idx, cnt, d);

   return cnt;
 }


 /*
  * Locally-Weighted Regression (with FLANN).  Note that b will contain (d+1) regression coefficients.
  * Returns y = dot([1,x],b)
  */
 static double lwr_flann(double *b, double *x, nn_map_t *map, double *r)
 {
   int knn = 500;
   double lambda = 1; //.1;
   double epsilon = 1e-16;

   //double r2 = r*r;
   int d = map->dx;

   int nn_idx[knn];
   double nn_d2[knn];
   nn_map_find_knn(nn_idx, nn_d2, map, x, knn);
   //flann_find_nearest_neighbors_index_double(map->index, x, 1, nn_idx, nn_d2, knn, &map->params);

   double **X2 = new_matrix2(knn,d);
   double Y2[knn];
   double w[knn];
   for (int i = 0; i < knn; i++) {
     memcpy(X2[i], &map->X[d*nn_idx[i]], d*sizeof(double));
     Y2[i] = map->Y[nn_idx[i]];
     //w[i] = exp(-nn_d2[i] / r2);
   }
   for (int i = 0; i < knn; i++) {
     nn_d2[i] = 0;
     for (int j = 0; j < d; j++)
       nn_d2[i] += r[j] * r[j] * (x[j] - X2[i][j]) * (x[j] - X2[i][j]);
     w[i] = exp(-nn_d2[i]);
   }
   double wtot = sum(w, knn);

   // compute the offset, b[0], and center the Y's
   b[0] = (wtot > epsilon ? dot(Y2, w, knn) / wtot : 0.);
   for (int i = 0; i < knn; i++)
     Y2[i] -= b[0];

   // use ridge regression to find b[1:d]
   weighted_ridge_regression(&b[1], X2, Y2, w, lambda, knn, d);
   //iterative_weighted_ridge_regression(&b[1], X2, Y2, w, lambda, knn, d);

   free_matrix2(X2);

   double y = b[0] + dot(&b[1], x, d);

   return y;
 }




 //---------------------------------- swing data processing ------------------------------------//


 static arm_dynamics_signal_t *differentiate_swing_data(const SwingData &swing)
 {
   arm_dynamics_signal_t *signal;
   safe_calloc(signal, 1, arm_dynamics_signal_t);
   int n = swing.t.size();
   int d = swing.joint_angles.size() / n;

   double **Q = new_matrix2(d,n-2);
   double **dQ = new_matrix2(d,n-2);
   double **ddQ = new_matrix2(d,n-2);
   double **U = new_matrix2(d,n-2);

   signal->Q = Q;
   signal->dQ = dQ;
   signal->ddQ = ddQ;
   signal->U = U;
   signal->n = n-2;
   signal->d = d;

   for (int j = 0; j < d; j++) {
     for (int i = 0; i < n-2; i++) {

       double q0 = swing.joint_angles[i*d+j];
       double q1 = swing.joint_angles[(i+1)*d+j];
       double q2 = swing.joint_angles[(i+2)*d+j];

       double t0 = swing.t[i];
       double t1 = swing.t[i+1];
       double t2 = swing.t[i+2];

       double dq1 = (q1-q0)/(t1-t0);
       double dq2 = (q2-q1)/(t2-t1);

       double ddq = (dq2-dq1)/(t2-t1);
       double u = swing.joint_torques[(i+1)*d+j];

       Q[j][i] = q1;
       dQ[j][i] = dq1;
       ddQ[j][i] = ddq;
       U[j][i] = u;
     }
   }

   return signal;
 }


 //TODO: add to libbingham
 static void smooth_signal(double *y, double *x, int n, int w)
 {
   double *x2 = x;
   if (y==x) {
     safe_calloc(x2, n, double);
     memcpy(x2, x, n*sizeof(double));
   }

   int h = MAX(w/2, 1);  // moving average kernel half-width
   double d = (double)w;

   for (int i = 0; i < n; i++) {
     double s = 0.;
     int j0 = MAX(i-h, 0);
     int j1 = MIN(i+h, n-1);
     for (int j = j0; j <= j1; j++)
       s += x2[j];
     y[i] = s/d;
   }

   if (y==x)
     free(x2);
 }


 static void smooth_swing_data(arm_dynamics_signal_t *signal)
 {
   int w = 5;

   for (int i = 0; i < signal->d; i++) {
     smooth_signal(signal->dQ[i], signal->dQ[i], signal->n, w);
     //smooth_signal(signal->dQ[i], signal->dQ[i], signal->n, w);

     smooth_signal(signal->ddQ[i], signal->ddQ[i], signal->n, w);
     smooth_signal(signal->ddQ[i], signal->ddQ[i], signal->n, w);
     //smooth_signal(signal->ddQ[i], signal->ddQ[i], signal->n, w);
     //smooth_signal(signal->ddQ[i], signal->ddQ[i], signal->n, w);
   }
 }


 /*
  * Differentiate and smooth swing data signal.
  */
 arm_dynamics_signal_t *get_swing_signals(const SwingData &swing)
 {
   arm_dynamics_signal_t *signal = differentiate_swing_data(swing);
   smooth_swing_data(signal);

   return signal;
 }


 static int build_lookup_tables(double ***X, arm_dynamics_signal_t *signal)
 {
   double **Q = signal->Q;
   double **dQ = signal->dQ;
   double **U = signal->U;
   int n = signal->n;
   int d = signal->d;

   for (int j = 0; j < d; j++) {  // j for joint
     X[j] = new_matrix2(n, 2*d+1);
     for (int i = 0; i < n; i++) {
       for (int k = 0; k < d; k++)
	 X[j][i][k] = Q[k][i];
       for (int k = 0; k < d; k++)
	 X[j][i][d+k] = dQ[k][i];
       X[j][i][2*d] = U[j][i];
     }
   }

   return 2*d+1;
 }


 //------------------------------------------ Public API -------------------------------------------------//


 bool ArmDynamics::save(const char *base_name)
 {
   char fname[1024];

   for (int i = 0; i < num_joints; i++) {
     int n = full_maps[i].n;
     int dx = full_maps[i].dx;
     int dy = full_maps[i].dy;
     double **X = new_matrix2(n,dx);
     double **Y = new_matrix2(n,dy);
     memcpy(X[0], full_maps[i].X, n*dx*sizeof(double));
     memcpy(Y[0], full_maps[i].Y, n*dy*sizeof(double));

     sprintf(fname, "%s_X%d.txt", base_name, i);
     save_matrix(fname, X, n, dx);
     sprintf(fname, "%s_Y%d.txt", base_name, i);
     save_matrix(fname, Y, n, dy);

     free_matrix2(X);
     free_matrix2(Y);
   }

   return true;
 }

 bool ArmDynamics::load(const char *base_name)
 {
   char fname[1024];
   double **X[num_joints];
   double *Y[num_joints];
   int n, dx, dy;

   for (int i = 0; i < num_joints; i++) {
     sprintf(fname, "%s_X%d.txt", base_name, i);
     X[i] = load_matrix(fname, &n, &dx);
     sprintf(fname, "%s_Y%d.txt", base_name, i);
     double **Y_mat = load_matrix(fname, &n, &dy);

     if (X[i] == NULL || Y_mat == NULL)
       return false;

     Y[i] = Y_mat[0];
   }

   addData(X, Y, n, dx);

   return true;
 }

 void ArmDynamics::addData(double ***X, double **Y, int n, int dx)
 {
   int d = num_joints;

   // create maps if empty
   if (isEmpty()) {
     safe_calloc(full_maps, d, nn_map_t);
     //safe_calloc(lwr_maps, d, nn_map_t);
   }

   // build full NN-maps from (Q,dQ,U) --> ddQ.
   for (int i = 0; i < d; i++)
     nn_map_add_data(&full_maps[i], X[i][0], Y[i], n, dx, 1);

   /*
   // sparsify new (Q,dQ,U) points for LWR NN-map
   double d2_thresh = .00001;
   double **X2[d];
   int n2[d];
   for (int i = 0; i < d; i++)
     n2[i] = sparsify_points_with_map(&X2[i], &lwr_maps[i], X[i], n, dx, d2_thresh);

   // compute LWR coefficients B at each new point (Q,dQ,U).
   double **B[d];
   double r = .5;  //.1;  //.05;  // kernel width
   int db = dx+1;
   for (int i = 0; i < d; i++) {
     if (n2[i] > 0) {
       B[i] = new_matrix2(n2[i], db);
       for (int j = 0; j < n2[i]; j++)
	 lwr_flann(B[i][j], X2[i][j], &full_maps[i], r);
     }
   }

   // (optional) add back some deleted LWR points to enforce approximation error bounds.

   // build LWR NN-maps from (Q,dQ,U) --> B
   for (int i = 0; i < d; i++)
     if (n2[i] > 0)
       nn_map_add_data(&lwr_maps[i], X2[i][0], B[i][0], n2[i], dx, db);
   */
 }

 void ArmDynamics::addSwingData(const SwingData &swing)
 {
   printf("   swing length = %d (%.2f secs)...", swing.t.size(), swing.t.back() - swing.t.front());

   // pre-process swing data
   arm_dynamics_signal_t *signal = get_swing_signals(swing);
   int n = signal->n;
   signal->d = num_joints;  // only model the first 'num_joints' joints
   int d = signal->d;
   double **X[d];
   int dx = build_lookup_tables(X, signal);
   double **Y = signal->ddQ;

   addData(X, Y, n, dx);
 }


 /*
  * Computes joint accelerations and gradients w.r.t. (q,dq,u).
  * 'ddq' and 'gradients' must be pre-allocated.  'ddq' should be a length d array and
  * 'gradients' should be a (d-by-2d+2) matrix, which can be created with new_matrix2(d, 2*d+2).
  * Upon return:
  *   ddq[0,...,j-1] will contain the join accelerations,
  *   gradients[i][0] will contain the partial derivatives of ddq[i] w.r.t. 1,
  *   gradients[i][1,...,j] will contain the partial derivatives of ddq[i] w.r.t. q,
  *   gradients[i][j+1,...,2*j] will contain the partial derivatives of ddq[i] w.r.t. dq, and
  *   gradients[i][2*j+1] will contain the partial derivative of ddq[i] w.r.t. u[i].
  */
 void ArmDynamics::dynamics(double *ddq, double **gradients, double *q, double *dq, double *u)
 {
   //printf("arm dynamics()\n");
   //printf("q = (%.2f, %.2f, %.2f), dq = (%.2f, %.2f, %.2f), u = (%.2f, %.2f, %.2f)\n",
   //	 q[0], q[1], q[2], dq[0], dq[1], dq[2], u[0], u[1], u[2]);

   double joint_noise[7] = {.1,.05,.05,.05,.01,.01,.002};

   int d = num_joints;
   double x[2*d+2];
   double b[2*d+2];
   for (int i = 0; i < d; i++) {
     memcpy(&x[0], q, d*sizeof(double));
     memcpy(&x[d], dq, d*sizeof(double));
     x[2*d] = u[i];
     //nn_map_lookup(b, &lwr_maps[i], x);
     //double r = .5; //.1;
     double r[15] = {.1,.1,.1,.1,.1,.1,.1, .2,.2,.2,.2,.2,.2,.2, 10*joint_noise[i]};

     lwr_flann(b, x, &full_maps[i], r);

     //knn_map_lookup_smooth(b, &lwr_maps[i], x, k, r);

     //printf("b%d = (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)\n", i, b[0], b[1], b[2], b[3], b[4], b[5], b[6]); //dbug

     if (ddq)
       ddq[i] = b[0] + dot(&b[1], x, 2*d+1);
     if (gradients)
       memcpy(gradients[i], b, (2*d+2)*sizeof(double));
   }
 }




 //------------------------------------------ Gravity Compensation -------------------------------------------------//

 void get_gravity_torques(double *torques, double *joint_angles)
 {
   //DQ2 = (repmat(q, [ng,1]) - gravity_table.Q_table).^2;
   //[~,idx] = min(DQ2*[1 1 1 1 .1 .1 .1]');
   //U(i,:) = gravity_table.U_table(idx,:);

   int n = gravity_table_length;

   // compute weighted squared-distances between joint_angles and gravity_Q_table
   double w[7] = {1, 1, 1, 1, .3, .3, .3};
   double d2[n];
   for (int i = 0; i < n; i++) {
     double dq[7];
     sub(dq, joint_angles, (double *)&gravity_Q_table[7*i], 7);
     vmult(dq, dq, w, 7);
     d2[i] = dot(dq, dq, 7);
   }

   memcpy(torques, &gravity_U_table[7*find_min(d2,n)], 7*sizeof(double));
 }

 double **get_gravity_torques_for_trajectory(double **Q, int n)
 {
   // look up gravity torques from the gravity table
   double **Ut = new_matrix2(7,n);
   for (int i = 0; i < n; i++) {
     double u[7];
     get_gravity_torques(u, Q[i]);
     for (int j = 0; j < 7; j++)
       Ut[j][i] = u[j];
   }

   // smooth gravity torques along the trajectory
   int w = 5;
   for (int j = 0; j < 7; j++)
     smooth_signal(Ut[j], Ut[j], n, w);

   double **U = new_matrix2(n,7);
   transpose(U, Ut, 7, n);
   free_matrix2(Ut);

   return U;
 }



 //------------------------------------------ Swing Torques -------------------------------------------------//


 swing_table_t load_swing_table(char *path)
 {
   swing_table_t table;
   int n, m;

   //dbug
   //char buf[1024];
   //sprintf(buf, "%s/default_swing_table", path);
   //path = buf;

   // load hit params
   char fname[1024];
   sprintf(fname, "%s/swing_table_T.txt", path);
   double **T = load_matrix(fname, &n, &m);

   if (T == NULL) {
     table.n = 0;
     return table;
   }

   table.T = T[0];
   free(T);
   sprintf(fname, "%s/swing_table_E.txt", path);
   double **E = load_matrix(fname, &n, &m);
   table.E = E[0];
   free(E);
   sprintf(fname, "%s/swing_table_B.txt", path);
   table.B = load_matrix(fname, &n, &m);
   sprintf(fname, "%s/swing_table_PP.txt", path);
   table.PP = load_matrix(fname, &n, &m);
   sprintf(fname, "%s/swing_table_PV.txt", path);
   table.PV = load_matrix(fname, &n, &m);
   sprintf(fname, "%s/swing_table_PN.txt", path);
   table.PN = load_matrix(fname, &n, &m);
   table.n = n;

   // load swing torques
   safe_calloc(table.U, n, double**);
   safe_calloc(table.N, n, int);
   for (int i = 0; i < n; i++) {
     sprintf(fname, "%s/swing_table_U%d.txt", path, i+1);
     table.U[i] = load_matrix(fname, &table.N[i], &m);

     printf("U = %p, n = %d, m = %d\n", table.U[i], table.N[i], m); //dbug
   }

   return table;
 }


 swing_torques_t get_swing_torques(swing_table_t *table, hit_t hit, double t0)
 {
   double d2thresh = .04; //1000000000.0;

   swing_torques_t torques;
   torques.n = 0;

   //printf("hit.pos       = (%.2f, %.2f, %.2f)\n", hit.pos[0], hit.pos[1], hit.pos[2]); //dbug

   /* look up swing based on the pre-hit (y,z) position
   double dt = .01; //.02;
   int n = floor((hit.t - t0)/dt);  // hit time step
   int n1 = (n%2 ? n-8 : n-9);  // reach the y,z goal
   double y_prehit = hit.pos[1] - dt*(n-n1)*hit.vel[1];
   double z_prehit = hit.pos[2] - dt*(n-n1)*hit.vel[2];
   double hit_pos[3] = {hit.pos[0], y_prehit, z_prehit};
   */

   double t = hit.t - t0;
   double d2min = d2thresh;
   int imin = -1;
   for (int i = 0; i < table->n; i++) {
     if (table->E[i] > .06)
       continue;
     double dt = t - table->T[i];
     //if (dt > .01)  //dbug
     //  continue;
     //double dvx = hit.vel[0] - table->PV[i][0];
     //double d2 = dist2(hit_pos, table->PP[i], 3) + .01*dvx*dvx + 9*dt*dt;
     double d2 = dist2(hit.pos, table->PP[i], 3) + .04*dist2(hit.vel, table->PV[i], 3)
       + .06*dist2(hit.normal, table->PN[i], 3) + 9*dt*dt;
    if (d2 < d2min) {
      d2min = d2;
      imin = i;
    }
  }
  if (imin >= 0) {
    printf("imin = %d, pp = (%.2f, %.2f, %.2f)\n", imin, table->PP[imin][0], table->PP[imin][1], table->PP[imin][2]); //dbug

    torques.n = table->N[imin];
    torques.U = table->U[imin];
  }

  return torques;
}



/*
static vector<double **> swing_torques_table;
static vector<double *> swing_torques_p_hit;
static const int swing_torques_swing_length = 26;  //TODO: make this variable?
static const int swing_torques_num_joints = 7;

double **get_swing_torques(double *p_hit)
{
  double d2min = 10000000.;
  uint imin = -1;
  for (uint i = 0; i < swing_torques_table.size(); i++) {
    double d2 = dist2(p_hit, swing_torques_p_hit[i], 3);
    if (d2 < d2min) {
      d2min = d2;
      imin = i;
    }
  }
  if (d2min < .03*.03)
    return swing_torques_table[imin];
  return NULL;
}

void update_swing_torques(const fastwam::SwingData &swing, double *p_hit)
{
  // get new swing torques
  double **new_torques = new_matrix2(26,7);
  for (int i = 0; i < 260; i++)
    for (int j = 0; j < 7; j++)
      new_torques[i/10][j] += .1*swing.joint_torques[7*i+j];

  double **torques = get_swing_torques(p_hit);
  if (torques == NULL) {
    swing_torques_table.push_back(new_torques);
    swing_torques_p_hit.push_back(p_hit);
  }
  else {
    avg(torques[0], torques[0], new_torques[0], 26*7);
    free_matrix2(new_torques);
  }
}
*/
