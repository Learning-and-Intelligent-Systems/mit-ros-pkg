#include <cardboard/util.h>


using namespace std;
using namespace Eigen;


namespace cardboard {


  double get_time_ms()
  {
    struct timeval tv;
    struct timezone tz;

    gettimeofday(&tv, &tz);

    return 1000.*tv.tv_sec + tv.tv_usec/1000.;
  }

  void init_rand()
  {
    static int first = 1;
    if (first) {
      first = 0;
      srand (time(NULL));
    }
  }
  
  // returns a random double in [0,1]
  double frand()
  {
    init_rand();
    
    return fabs(rand()) / (double)RAND_MAX;
  }
  
  // approximation to the inverse error function
  double erfinv(double x)
  {
    if (x < 0)
      return -erfinv(-x);
    
    double a = .147;
    
    double y1 = (2/(M_PI*a) + log(1-x*x)/2.0);
    double y2 = sqrt(y1*y1 - (1/a)*log(1-x*x));
    double y3 = sqrt(y2 - y1);
    
    return y3;
  }
  
  
  // generate a random sample from a normal distribution
  double normrand(double mu, double sigma)
  {
    double u = frand();
    
    return mu + sigma*sqrt(2.0)*erfinv(2*u-1);
  }

  // compute the pdf of a normal random variable
  double normpdf(double x, double mu, double sigma)
  {
    double dx = x - mu;
    return exp(-dx*dx / (2*sigma*sigma)) / (sqrt(2*M_PI) * sigma);
  }
  
  Matrix4f pose_to_affine_matrix(geometry_msgs::Pose pose)
  {
    Vector3f t(pose.position.x, pose.position.y, pose.position.z);
    Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Matrix3f R(q);
    Matrix4f A;
    A << R, t, 0,0,0,1;
    
    return A;
  }
  
  Matrix4f pose_to_affine_matrix(Vector3f t, Quaternionf q)
  {
    Matrix3f R(q);
    Matrix4f A;
    A << R, t, 0,0,0,1;
    
    return A;
  }
  
  geometry_msgs::Pose affine_matrix_to_pose(Matrix4f A)
  {
    geometry_msgs::Pose p;
    
    Matrix3f R = A.topLeftCorner(3,3);
    Quaternionf q(R);
    Vector3f t = A.topRightCorner(3,1);
    
    p.position.x = t(0);
    p.position.y = t(1);
    p.position.z = t(2);
    
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    
    return p;
  }
  
  double sum(vector<double> v)
  {
    double x = 0.0;
    for (uint i = 0; i < v.size(); i++)
      x += v[i];
    return x;
  }
  
  // calculate the area of a triangle
  double triangle_area(Vector3f x, Vector3f y, Vector3f z)
  {
    double a = (x-y).norm(); //dist(x, y, n);
    double b = (x-z).norm(); //dist(x, z, n);
    double c = (y-z).norm(); //dist(y, z, n);
    double s = .5*(a + b + c);
    
    return sqrt(s*(s-a)*(s-b)*(s-c));
  }
  
  // convert a PlyVertex into an array
  Vector3f vertex_to_eigen(PlyVertex *V)
  {
    return Vector3f(V->x, V->y, V->z);
  }
  
  // calculate the orientation (normal) of a triangle in 3F
  Vector3f triangle_orientation(Vector3f x, Vector3f y, Vector3f z)
  {
    return (z-x).cross(y-x).normalized();
  }


  // dilate finite ranges
  void dilate_range_image(const pcl::RangeImage &range_image_in, pcl::RangeImage &range_image_out)
  {
    range_image_out = range_image_in;

    int w = range_image_in.width;
    int h = range_image_in.height;

    for (int i = 0; i < w*h; i++) {
      int x = i%w;
      int y = (i-x)/w;
      float r = range_image_in.points[i].range;

      if (isfinite(r))  // || r < 0)
	continue;

      float px = 0.0, py = 0.0, pz = 0.0, pr = 0.0;
      int cnt = 0;
      for (int dx = -1; dx <= 1; dx++) {
	for (int dy = -1; dy <= 1; dy++) {
	  if (dx==0 && dy==0)
	    continue;
	  int x2 = x+dx;
	  int y2 = y+dy;
	  if (x2 < 0 || x2 >= w || y2 < 0 || y2 >= h)
	    continue;
	  float r2 = range_image_in.points[y2*w+x2].range;
	  if (isfinite(r2)) {
	    px += range_image_in.points[y2*w+x2].x;
	    py += range_image_in.points[y2*w+x2].y;
	    pz += range_image_in.points[y2*w+x2].z;
	    pr += r2;
	    cnt++;
	  }
	}
      }

      if (cnt > 0) {
	range_image_out.points[i].x = px/(float)cnt;
	range_image_out.points[i].x = px/(float)cnt;
	range_image_out.points[i].x = px/(float)cnt;
	range_image_out.points[i].range = pr/(float)cnt;
      }
    }
  }


  // gaussian blur a matrix
  MatrixXf blur_matrix(const MatrixXf &A, float sigma)
  {
    // compute filter width
    int w = (int)(2*sigma);
    if (w < 1)
      w = 1;

    // compute 1D filter
    VectorXf f(2*w+1);
    for (int i = 0; i <= w; i++) {
      f(w+i) = normpdf(i,0,sigma);
      f(w-i) = f(w+i);
    }

    // apply filter to columns
    MatrixXf B(A.rows(), A.cols());
    for (int j = 0; j < A.cols(); j++) {
      for (int i = 0; i < A.rows(); i++) {
	B(i,j) = 0.0;
	float ftot = 0.0;
	for (int k = -w; k <= w; k++) {
	  if (i+k >= 0 && i+k < A.rows()) {
	    B(i,j) += f(k+w)*A(i+k,j);
	    ftot += f(k+w);
	  }
	}
	B(i,j) /= ftot;
      }
    }

    // apply filter to rows
    MatrixXf C(A.rows(), A.cols());
    for (int i = 0; i < A.rows(); i++) {
      for (int j = 0; j < A.cols(); j++) {
	C(i,j) = 0.0;
	float ftot = 0.0;
	for (int k = -w; k <= w; k++) {
	  if (j+k >= 0 && j+k < A.cols()) {
	    C(i,j) += f(k+w)*B(i,j+k);
	    ftot += f(k+w);
	  }
	}
	C(i,j) /= ftot;
      }
    }
    
    return C;
  }


  // get the matrix of far ranges in a range image
  MatrixXf get_far_range_matrix(const pcl::RangeImage &range_image)
  {
    int w = range_image.width;
    int h = range_image.height;

    MatrixXf R(w,h);
    for (int x = 0; x < w; x++) {
      for (int y = 0; y < h; y++) {
	float range = range_image.points[y*w+x].range;
	//printf("range = %f\n", range);
	if (isinf(range) && range > 0.0)  // positive infinity
	  R(x,y) = 1.0;
	else
	  R(x,y) = 0.0;
      }
    }

    return R;
  }

  /***
  void visualize_point_cloud_normals(const PointCloudXYZN &cloud_normals)
  {
    //printf("visualize_point_cloud_normals\n");

    PointCloudXYZ cloud;
    PointCloudN normals;
    point_cloud_normals_to_points_and_normals(cloud_normals, cloud, normals);

    pcl_visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addCoordinateSystem(1.0f);
    viewer.addPointCloud(cloud, "points");
    viewer.addPointCloudNormals(cloud, normals, 3, .01);

    while(!viewer.wasStopped())
      {
	viewer.spinOnce(100);
	usleep(100000);
      }
  }

  void visualize_range_image(const pcl::RangeImage &range_image)
  {
    pcl_visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addCoordinateSystem(1.0f);
    viewer.addPointCloud(range_image, "point cloud");
    //viewer.addPointCloud(range_image, "range image");
    
    // --------------------------
    // -----Show range image-----
    // --------------------------
    pcl_visualization::RangeImageVisualizer range_image_widget("range image");
    range_image_widget.setRangeImage(range_image);
    
    //--------------------
    // -----Main loop-----
    //--------------------
    while(!viewer.wasStopped() || range_image_widget.isShown())
      {
	pcl_visualization::ImageWidgetWX::spinOnce();  // process GUI events
	viewer.spinOnce(100);
	usleep(100000);
      }
  }


  void visualize_range_image_with_normals(const pcl::RangeImage &range_image, const pcl::RangeImage &range_image_normals)
  {
    //printf("visualize_range_image_with_normals\n");

    PointCloudXYZ cloud;
    PointCloudN normals;
    range_image_with_normals_to_points_and_normals(range_image, range_image_normals, cloud, normals);

    pcl_visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addCoordinateSystem(1.0f);
    viewer.addPointCloud(cloud, "points");
    viewer.addPointCloudNormals(cloud, normals, 3, .01);

    while(!viewer.wasStopped())
      {
	viewer.spinOnce(100);
	usleep(100000);
      }
  }
  ************/

  void compute_normals(const PointCloudXYZ &cloud, PointCloudXYZN &cloud_with_normals, float radius)
  {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
    //KdTreeXYZ::Ptr kdtree = boost::make_shared< pcl::KdTreeFLANN<pcl::PointXYZ> >();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ> ());

    kdtree->setEpsilon(0.);
    normal_estimation.setSearchMethod(kdtree);
    //normal_estimation.setKSearch(10);
    normal_estimation.setRadiusSearch(radius);
    normal_estimation.setInputCloud(cloud.makeShared());
    normal_estimation.setViewPoint(cloud.sensor_origin_(0), cloud.sensor_origin_(1), cloud.sensor_origin_(2));
    normal_estimation.compute(cloud_with_normals);

    for (uint i = 0; i < cloud.points.size(); i++) {
      cloud_with_normals.points[i].x = cloud.points[i].x;
      cloud_with_normals.points[i].y = cloud.points[i].y;
      cloud_with_normals.points[i].z = cloud.points[i].z;
    }
  }


  void compute_principal_curvatures(const PointCloudXYZ &cloud, const PointCloudXYZN &cloud_with_normals,
				    pcl::PointCloud<pcl::PrincipalCurvatures> &principal_curvatures_cloud,
				    float radius)
  {
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PrincipalCurvatures> pcs_estimation;
    //KdTreeXYZ::Ptr kdtree = boost::make_shared< pcl::KdTreeFLANN<pcl::PointXYZ> >();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ> ());

    kdtree->setEpsilon(0.);
    pcs_estimation.setSearchMethod(kdtree);
    //pcs_estimation.setKSearch(10);
    pcs_estimation.setRadiusSearch(radius);
    pcs_estimation.setInputCloud(cloud.makeShared());
    pcs_estimation.setInputNormals(cloud_with_normals.makeShared());
    pcs_estimation.compute(principal_curvatures_cloud);
  }


  void compute_fpfhs(const PointCloudXYZ &cloud, const PointCloudXYZN &cloud_with_normals,
		     pcl::PointCloud<pcl::FPFHSignature33> &fpfh_cloud,
		     float radius)
  {
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh_estimation;
    //KdTreeXYZ::Ptr kdtree = boost::make_shared< pcl::KdTreeFLANN<pcl::PointXYZ> >();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ> ());

    kdtree->setEpsilon(0.);
    fpfh_estimation.setSearchMethod(kdtree);
    //fpfh_estimation.setKSearch(10);
    fpfh_estimation.setRadiusSearch(radius);
    fpfh_estimation.setInputCloud(cloud.makeShared());
    fpfh_estimation.setInputNormals(cloud_with_normals.makeShared());
    fpfh_estimation.compute(fpfh_cloud);
  }


  void compute_range_image_normals(const pcl::RangeImage &range_image, const PointCloudXYZN &normals,
				   pcl::RangeImage &range_image_normals)
  {
    range_image_normals = range_image;
    uint w = range_image.width;
    uint h = range_image.height;

    // initialize finite ranges in range_image_normals to +INFINITY
    for (size_t i = 0; i < w*h; i++)
      if (isfinite(range_image_normals.points[i].range))
	range_image_normals.points[i].range = INFINITY;

    // compute (finite) range image normals
    for (size_t i = 0; i < normals.points.size(); i++) {
      Vector3f p(normals.points[i].x, normals.points[i].y, normals.points[i].z);
      int xi, yi;
      float r;

      range_image.getImagePoint(p, xi, yi, r);
      int j = yi*w + xi;  //dbug

      if (xi >= 0 && yi >= 0 && xi < w && yi < h && r < range_image_normals.points[j].range) {
	range_image_normals.points[j].range = r;
	range_image_normals.points[j].x = normals.points[i].normal[0];
	range_image_normals.points[j].y = normals.points[i].normal[1];
	range_image_normals.points[j].z = normals.points[i].normal[2];
      }
    }
  }


  void point_cloud_normals_to_points_and_normals(const PointCloudXYZN &cloud_normals, PointCloudXYZ &cloud, PointCloudN &normals)
  {
    cloud.points.resize(cloud_normals.points.size());
    normals.points.resize(cloud_normals.points.size());

    uint cnt = 0;
    for (uint i = 0; i < cloud_normals.points.size(); i++) {
      float x = cloud_normals.points[i].x;
      float y = cloud_normals.points[i].y;
      float z = cloud_normals.points[i].z;
      float nx = cloud_normals.points[i].normal[0];
      float ny = cloud_normals.points[i].normal[1];
      float nz = cloud_normals.points[i].normal[2];
      float curvature = cloud_normals.points[i].curvature;

      if (isfinite(x) && isfinite(y) && isfinite(z) && isfinite(nx) && isfinite(ny) && isfinite(nz) && isfinite(curvature)) {
	cloud.points[cnt].x = x;
	cloud.points[cnt].y = y;
	cloud.points[cnt].z = z;
	normals.points[cnt].normal[0] = nx;
	normals.points[cnt].normal[1] = ny;
	normals.points[cnt].normal[2] = nz;
	normals.points[cnt].curvature = curvature;
	cnt++;
      }
    }
    cloud.points.resize(cnt);
    normals.points.resize(cnt);
    cloud.width = cnt;
    cloud.height = 1;
    cloud.is_dense = false;
    normals.width = cnt;
    normals.height = 1;
    normals.is_dense = false;
  }


  void range_image_with_normals_to_points_and_normals(const pcl::RangeImage &range_image, const pcl::RangeImage &range_image_normals,
						      PointCloudXYZ &cloud, PointCloudN &normals)
  {
    cloud.points.resize(range_image.points.size());
    normals.points.resize(range_image.points.size());

    uint cnt = 0;
    for (uint i = 0; i < range_image.points.size(); i++) {
      float x = range_image.points[i].x;
      float y = range_image.points[i].y;
      float z = range_image.points[i].z;
      float nx = range_image_normals.points[i].x;
      float ny = range_image_normals.points[i].y;
      float nz = range_image_normals.points[i].z;
      float nr = range_image_normals.points[i].range;

      if (isfinite(x) && isfinite(y) && isfinite(z) && isfinite(nx) && isfinite(ny) && isfinite(nz) && isfinite(nr)) {
	//printf("%.4f %.4f %.4f %.4f %.4f %.4f\n", x, y, z, nx, ny, nz);  //dbug
	cloud.points[cnt].x = x;
	cloud.points[cnt].y = y;
	cloud.points[cnt].z = z;
	normals.points[cnt].normal[0] = nx;
	normals.points[cnt].normal[1] = ny;
	normals.points[cnt].normal[2] = nz;
	cnt++;
      }
    }
    cloud.points.resize(cnt);
    normals.points.resize(cnt);
    cloud.width = cnt;
    cloud.height = 1;
    cloud.is_dense = false;
    normals.width = cnt;
    normals.height = 1;
    normals.is_dense = false;
  }

}  // namespace cardboard

