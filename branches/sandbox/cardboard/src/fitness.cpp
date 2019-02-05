
#include "fitness.h"



namespace cardboard {


  float range_image_fitness(const pcl::RangeImage &range_image, const MatrixXf &cost_map, const PointCloudXYZ &cloud, float out_of_bounds_cost) {

    return range_image_fitness(range_image, cost_map, cloud, out_of_bounds_cost, false);
  }

  float range_image_fitness(const pcl::RangeImage &range_image, const MatrixXf &cost_map, const PointCloudXYZ &cloud, float out_of_bounds_cost, bool debug)
  {
    //float hidden_range_dist = .2;

    //double t, t2;
    //t = get_time_ms();
    
    // init cloud_ranges matrix
    int w = 3*range_image.width;  // extra buffer around the edges
    int h = 3*range_image.height;  // extra buffer around the edges
    int x_offset = range_image.width;
    int y_offset = range_image.height;
    float cloud_ranges[w][h];
    float costs[w][h];
    for (int i = 0; i < w; i++) {
      for (int j = 0; j < h; j++) {
	cloud_ranges[i][j] = INFINITY;
	costs[i][j] = 0.0;
      }
    }

    //t2 = get_time_ms();
    //printf("break 1: %.2f ms\n", t2-t);
    //t = t2;
    
    //printf("w = %d, h = %d\n", w, h);
    
    
    //int cloud_range_indices[cloud.points.size()];
    //int cloud_range_indices_cnt = 0;


    size_t cloud_size = cloud.points.size();

    // compute L2 diff between cloud_ranges and range_image
    float d = 0.0;
    int cnt = 0;
    for (size_t i = 0; i < cloud_size; i++) {  //i += 5
      Vector3f p(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
      int image_x, image_y;
      float r;

      range_image.getImagePoint(p, image_x, image_y, r);

      int xi = image_x + x_offset;
      int yi = image_y + y_offset;
      
      if (xi >= 0 && yi >= 0 && xi < w && yi < h && isfinite(r) && r < cloud_ranges[xi][yi]) {
	bool new_range_cell = !isfinite(cloud_ranges[xi][yi]);
	cloud_ranges[xi][yi] = r;

	d -= costs[xi][yi];
	if (range_image.isInImage(image_x, image_y)) {
	  float image_r = range_image.getPoint(image_x, image_y).range;
	  float cloud_r = r;
	  if (isfinite(image_r) || image_r > 0) {
	    if (isfinite(image_r))
	      costs[xi][yi] = (image_r - cloud_r) * (image_r - cloud_r);
	    else if (image_r > 0)
	      costs[xi][yi] = cost_map(image_x, image_y);
	    if (new_range_cell)
	      cnt++;
	  }
	}
	else {
	  costs[xi][yi] = out_of_bounds_cost;
	  if (new_range_cell)
	    cnt++;
	}
	d += costs[xi][yi];

	//cloud_range_indices[cloud_range_indices_cnt++] = xi*h+yi;
      }
    }

  
    //t2 = get_time_ms();
    //printf("break 2: %.6f ms\n", t2-t);
    //t = t2;

    //t2 = get_time_ms();
    //printf("break 3: %.2f ms\n", t2-t);

    if (debug) {
      static int debug_cnt = 0;
      static FILE *f_debug;
      if (debug_cnt == 0) {
	f_debug = fopen("debug.m", "w");
      }
      debug_cnt++;

      fprintf(f_debug, "\ncosts%d = [ ", debug_cnt);
      for (int i = 0; i < w; i++)
	for (int j = 0; j < h; j++)
	  fprintf(f_debug, "%.4f ", costs[i][j]);
      fprintf(f_debug, "];\n");
      fprintf(f_debug, "C%d = reshape(costs%d, [%d %d]);\n\n", debug_cnt, debug_cnt, h, w);
    }

    return (cnt ? d/cnt : 0);
  }
  

  float range_image_normal_fitness(const pcl::RangeImage &range_image, const PointCloudXYZN &cloud_normals)
  {
    return range_image_normal_fitness(range_image, cloud_normals, false);
  }

  float range_image_normal_fitness(const pcl::RangeImage &range_image, const PointCloudXYZN &cloud_normals, bool debug)
  {
    // init cloud_ranges matrix
    //int w = 3*range_image.width;  // extra buffer around the edges
    //int h = 3*range_image.height;  // extra buffer around the edges
    //int x_offset = range_image.width;
    //int y_offset = range_image.height;
    int w = range_image.width;
    int h = range_image.height;
    float cloud_ranges[w][h];
    float costs[w][h];
    for (int i = 0; i < w; i++) {
      for (int j = 0; j < h; j++) {
	cloud_ranges[i][j] = INFINITY;
	costs[i][j] = 0.0;
      }
    }

    //int cloud_range_indices[cloud_normals.points.size()];  // to keep track of which range image cells have point normals
    //int cloud_indices_stack[cloud_normals.points.size()];  // to keep track of which points to process
    //int cloud_indices_cnt = 0;

    size_t cloud_size = cloud_normals.points.size();

    // get cost
    float d = 0.0;
    int cnt = 0;
    for (size_t i = 0; i < cloud_size; i ++) {  //i += 5

      float x = cloud_normals.points[i].x;
      float y = cloud_normals.points[i].y;
      float z = cloud_normals.points[i].z;

      if (!isfinite(x) || !isfinite(y) || !isfinite(z))
	continue;

      Vector3f p(x,y,z);
      int xi, yi;
      float r;

      range_image.getImagePoint(p, xi, yi, r);

      //xi += x_offset;
      //yi += y_offset;
      
      if (xi >= 0 && yi >= 0 && xi < w && yi < h && isfinite(r) && r < cloud_ranges[xi][yi]) {
	bool new_range_cell = !isfinite(cloud_ranges[xi][yi]);
	cloud_ranges[xi][yi] = r;
	
	float image_r = range_image.getPoint(xi, yi).range;
	if (isfinite(image_r)) {
	  float inx = range_image.getPoint(xi, yi).x;
	  float iny = range_image.getPoint(xi, yi).y;
	  float inz = range_image.getPoint(xi, yi).z;
	  float cnx = cloud_normals.points[i].normal[0];
	  float cny = cloud_normals.points[i].normal[1];
	  float cnz = cloud_normals.points[i].normal[2];

	  if (isfinite(cnx) && isfinite(cny) && isfinite(cnz)) {
	    d -= costs[xi][yi];
	    costs[xi][yi] = acos(fabs(inx*cnx + iny*cny + inz*cnz));
	    d += costs[xi][yi];
	    if (new_range_cell)
	      cnt++;
	  }
	}

	//cloud_range_indices[cloud_indices_cnt] = xi*h+yi;   // push
	//cloud_indices_stack[cloud_indices_cnt] = i;         // push
	//cloud_indices_cnt++;
      }
    }

    if (debug) {
      printf("\ncosts = [ ");
      for (int i = 0; i < w; i++)
	for (int j = 0; j < h; j++)
	  printf("%.2f ", costs[i][j]);
      printf("];\n");
      printf("C = reshape(costs, [%d %d]);\n", h, w);
    }

    return (cnt ? d/cnt : 0);
  }


  float range_cloud_fitness(KdTreeXYZ &kdtree, const pcl::RangeImage &range_cloud)
  {
    float d = 0.0;
    int k = 1;
    std::vector<int> knn_indices(k);
    std::vector<float> knn_distances(k);
    
    int cnt = 0;
    for (size_t i = 0; i < range_cloud.points.size(); i++) {
      if (isfinite(range_cloud.points[i].range)) {
	pcl::PointXYZ p(range_cloud.points[i].x, range_cloud.points[i].y, range_cloud.points[i].z);
	kdtree.nearestKSearch(p, k, knn_indices, knn_distances);
	d += knn_distances[0];  //*knn_distances[0];
	cnt++;
      }
    }
    
    return (cnt ? d/cnt : 0);
  }
  

  float range_cloud_fitness(const DistanceTransform3D &distance_transform, const pcl::RangeImage &range_cloud)
  {
    float d_tot = 0.0;
    int cnt = 0;

    for (size_t i = 0; i < range_cloud.points.size(); i++) {
      if (isfinite(range_cloud.points[i].range)) {
	pcl::PointXYZ p(range_cloud.points[i].x, range_cloud.points[i].y, range_cloud.points[i].z);
	float d = distance_transform.getDistance(p);
	d_tot += d*d;
	cnt++;
      }
    }
    
    return (cnt ? d_tot/cnt : 0);
  }


  float point_cloud_fitness(KdTreeXYZ &kdtree, const PointCloudXYZ &point_cloud)
  {
    float d = 0.0;
    int k = 1;
    std::vector<int> knn_indices(k);
    std::vector<float> knn_distances(k);
    
    int cnt = 0;
    for (size_t i = 0; i < point_cloud.points.size(); i++) {
      if (!isnan(point_cloud.points[i].x) && !isnan(point_cloud.points[i].y) && !isnan(point_cloud.points[i].z)) {
	kdtree.nearestKSearch(point_cloud.points[i], k, knn_indices, knn_distances);
	d += knn_distances[0];  //*knn_distances[0];
	cnt++;
      }
    }
    
    return (cnt ? d/cnt : 0);
  }


  float point_cloud_fitness(const DistanceTransform3D &distance_transform, const PointCloudXYZ &point_cloud)
  {
    float d_tot = 0.0;
    int cnt = 0;

    for (size_t i = 0; i < point_cloud.points.size(); i++) {
      if (!isnan(point_cloud.points[i].x) && !isnan(point_cloud.points[i].y) && !isnan(point_cloud.points[i].z)) {
	float d = distance_transform.getDistance(point_cloud.points[i]);
	d_tot += d*d;
	cnt++;
      }
    }

    return (cnt ? d_tot/cnt : 0);
  }
  


}  // namespace cardboard
