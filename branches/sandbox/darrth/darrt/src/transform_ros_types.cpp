#include "darrt/transform_ros_types.hh"
#include "darrt/utils.hh"

geometry_msgs::Point darrt::transform_point
(const geometry_msgs::Point &pt, const geometry_msgs::Pose &transform) {
  return transform_point(pt, pose_to_transform(transform));
}

geometry_msgs::Point darrt::transform_point
(const geometry_msgs::Point &pt, const geometry_msgs::Transform &transform) {
  const geometry_msgs::Quaternion &quat = transform.rotation;
  const geometry_msgs::Vector3 &trans = transform.translation;
  //create the rotation matrix (or i can't keep this all straight
  double row00 = 1.0 - 2*quat.y*quat.y - 2*quat.z*quat.z;
  double row01 = 2*quat.x*quat.y - 2*quat.z*quat.w;
  double row02 = 2*quat.x*quat.z + 2*quat.y*quat.w;

  double row10 = 2*quat.x*quat.y + 2*quat.z*quat.w;
  double row11 = 1 - 2*quat.x*quat.x - 2*quat.z*quat.z;
  double row12 = 2*quat.y*quat.z - 2*quat.x*quat.w;

  double row20 = 2*quat.x*quat.z - 2*quat.y*quat.w;
  double row21 = 2*quat.y*quat.z + 2*quat.x*quat.w;
  double row22 = 1 - 2*quat.x*quat.x - 2*quat.y*quat.y;
  
  geometry_msgs::Point tp;
  tp.x = row00*pt.x + row01*pt.y + row02*pt.z + trans.x;
  tp.y = row10*pt.x + row11*pt.y + row12*pt.z + trans.y;
  tp.z = row20*pt.x + row21*pt.y + row22*pt.z + trans.z;
  
  return tp;
}

geometry_msgs::Point darrt::inverse_transform_point
(const geometry_msgs::Point &pt, const geometry_msgs::Pose &transform) {
  return inverse_transform_point(pt, pose_to_transform(transform));
}

geometry_msgs::Point darrt::inverse_transform_point
(const geometry_msgs::Point &pt, const geometry_msgs::Transform &transform) {
  const geometry_msgs::Quaternion &quat = transform.rotation;
  const geometry_msgs::Vector3 &trans = transform.translation;
  //note: this is called often enough that we don't want to
  //create vectors

  double row00 = 1.0 - 2*quat.y*quat.y - 2*quat.z*quat.z;
  double row01 = 2*quat.x*quat.y - 2*quat.z*quat.w;
  double row02 = 2*quat.x*quat.z + 2*quat.y*quat.w;

  double row10 = 2*quat.x*quat.y + 2*quat.z*quat.w;
  double row11 = 1 - 2*quat.x*quat.x - 2*quat.z*quat.z;
  double row12 = 2*quat.y*quat.z - 2*quat.x*quat.w;

  double row20 = 2*quat.x*quat.z - 2*quat.y*quat.w;
  double row21 = 2*quat.y*quat.z + 2*quat.x*quat.w;
  double row22 = 1 - 2*quat.x*quat.x - 2*quat.y*quat.y;

  geometry_msgs::Point tp;
  tp.x = row00*(pt.x - trans.x) + row10*(pt.y - trans.y)
    + row20*(pt.z - trans.z);
  tp.y = row01*(pt.x - trans.x) + row11*(pt.y - trans.y)
    + row21*(pt.z - trans.z);
  tp.z = row02*(pt.x - trans.x) + row12*(pt.y - trans.y)
    + row22*(pt.z - trans.z);
  return tp;
}

geometry_msgs::Quaternion darrt::transform_quaternion
(const geometry_msgs::Quaternion &quat, const geometry_msgs::Pose &transform) {
  return transform_quaternion(quat, pose_to_transform(transform));
}

geometry_msgs::Quaternion darrt::transform_quaternion
(const geometry_msgs::Quaternion &quat, 
 const geometry_msgs::Transform &transform) {
  geometry_msgs::Quaternion ret;
  multiply(transform.rotation, quat, ret);
  return ret;
}

geometry_msgs::Quaternion darrt::inverse_transform_quaternion
(const geometry_msgs::Quaternion &quat, const geometry_msgs::Pose &transform) {
  return inverse_transform_quaternion(quat, pose_to_transform(transform));
}

geometry_msgs::Quaternion darrt::inverse_transform_quaternion
(const geometry_msgs::Quaternion &quat, 
 const geometry_msgs::Transform &transform) {
  geometry_msgs::Quaternion conj, ret;
  conjugate(transform.rotation, conj);
  multiply(conj, quat, ret);
  return ret;
}

geometry_msgs::Vector3 darrt::transform_vector
(const geometry_msgs::Vector3 &vector, const geometry_msgs::Pose &transform) {
  return transform_vector(vector, pose_to_transform(transform));
}

geometry_msgs::Vector3 darrt::transform_vector
(const geometry_msgs::Vector3 &vector, const geometry_msgs::Transform &transform) {
  geometry_msgs::Point p;
  p.x = vector.x;
  p.y = vector.y;  
  p.z = vector.z;
  geometry_msgs::Transform t = transform;
  t.translation.x = 0.0;
  t.translation.y = 0.0;  
  t.translation.z = 0.0;
  geometry_msgs::Point tp = transform_point(p, t);
  geometry_msgs::Vector3 v;
  v.x = tp.x;
  v.y = tp.y;
  v.z = tp.z;
  return v;
}


geometry_msgs::Vector3 darrt::inverse_transform_vector
(const geometry_msgs::Vector3 &vector, const geometry_msgs::Pose &transform) {
  return inverse_transform_vector(vector, pose_to_transform(transform));
}

geometry_msgs::Vector3 darrt::inverse_transform_vector
(const geometry_msgs::Vector3 &vector, const geometry_msgs::Transform &transform) {
  geometry_msgs::Point p;
  p.x = vector.x;
  p.y = vector.y;  
  p.z = vector.z;
  geometry_msgs::Transform t = transform;
  t.translation.x = 0.0;
  t.translation.y = 0.0;  
  t.translation.z = 0.0;
  geometry_msgs::Point tp = inverse_transform_point(p, t);
  geometry_msgs::Vector3 v;
  v.x = tp.x;
  v.y = tp.y;
  v.z = tp.z;
  return v;
}


geometry_msgs::Pose darrt::transform_pose
(const geometry_msgs::Pose &pose, const geometry_msgs::Pose &transform) {
  return transform_pose(pose, pose_to_transform(transform));
}

geometry_msgs::Pose darrt::transform_pose
(const geometry_msgs::Pose &pose, const geometry_msgs::Transform &transform) {
  geometry_msgs::Pose p;
  p.position = transform_point(pose.position, transform);
  p.orientation = transform_quaternion(pose.orientation, transform);
  return p;
}

geometry_msgs::Pose darrt::inverse_transform_pose
(const geometry_msgs::Pose &pose, const geometry_msgs::Pose &transform) {
  return inverse_transform_pose(pose, pose_to_transform(transform));
}

geometry_msgs::Pose darrt::inverse_transform_pose
(const geometry_msgs::Pose &pose, const geometry_msgs::Transform &transform) {
  geometry_msgs::Pose p;
  p.position = inverse_transform_point(pose.position, transform);
  p.orientation = inverse_transform_quaternion(pose.orientation, transform);
  return p;
}

geometry_msgs::Transform darrt::pose_to_transform
(const geometry_msgs::Pose &pose) {
  geometry_msgs::Transform trans;
  trans.translation.x = pose.position.x;
  trans.translation.y = pose.position.y;
  trans.translation.z = pose.position.z;

  trans.rotation.x = pose.orientation.x;
  trans.rotation.y = pose.orientation.y;  
  trans.rotation.z = pose.orientation.z;
  trans.rotation.w = pose.orientation.w;
  return trans;
}

geometry_msgs::Pose darrt::transform_to_pose
(const geometry_msgs::Transform &trans) {
  geometry_msgs::Pose pose;
  
  pose.position.x = trans.translation.x;
  pose.position.y = trans.translation.y;
  pose.position.z = trans.translation.z;

  pose.orientation.x = trans.rotation.x;
  pose.orientation.y = trans.rotation.y;
  pose.orientation.z = trans.rotation.z;
  pose.orientation.w = trans.rotation.w;
  return pose;
}

geometry_msgs::Pose2D darrt::pose_to_pose2D
(const geometry_msgs::Pose &pose) {
  geometry_msgs::Pose2D p;
  p.x = pose.position.x;
  p.y = pose.position.y;
  p.theta = quaternion_to_yaw(pose.orientation);
  return p;
}

geometry_msgs::Pose darrt::pose2D_to_pose
(const geometry_msgs::Pose2D &pose) {
  geometry_msgs::Pose p;
  p.position.x = pose.x;
  p.position.y = pose.y;
  p.orientation.z = sin(pose.theta/2.0);
  p.orientation.w = cos(pose.theta/2.0);
  return p;
}

geometry_msgs::Pose darrt::pose2D_to_ground_pose
(const geometry_msgs::Pose2D &pose) {
  geometry_msgs::Pose p;
  p.position.x = pose.x;
  p.position.y = pose.y;
  p.position.z = 0.0;
  p.orientation.x = 0.0;
  p.orientation.y = 0.0;
  p.orientation.z = sin(pose.theta/2.0);
  p.orientation.w = cos(pose.theta/2.0);
  return p;
}


std::vector<geometry_msgs::Point> darrt::corners
(const arm_navigation_msgs::Shape &shape) {
  double minx=MATH_INF, miny=MATH_INF, minz=MATH_INF, 
    maxx=-1.0*MATH_INF, maxy=-1.0*MATH_INF, maxz=-1.0*MATH_INF;
  std::vector<geometry_msgs::Point> corners(8);
  std::vector<double> dimensions = shape.dimensions;
  switch(shape.type) {
  case arm_navigation_msgs::Shape::BOX:
    minx = -1.0*dimensions[0]/2.0;
    miny = -1.0*dimensions[1]/2.0;
    minz = -1.0*dimensions[2]/2.0;
    maxx = dimensions[0]/2.0;
    maxy = dimensions[1]/2.0;
    maxz = dimensions[2]/2.0;
    break;
  case arm_navigation_msgs::Shape::CYLINDER:
    minx = -1.0*dimensions[0];
    miny = -1.0*dimensions[0];
    minz = -1.0*dimensions[1]/2.0;
    maxx = dimensions[0];
    maxy = dimensions[0];
    maxz = dimensions[1]/2.0;
    break;
  case arm_navigation_msgs::Shape::SPHERE:
    minx = -1.0*dimensions[0];
    miny = -1.0*dimensions[0];
    minz = -1.0*dimensions[0];
    maxx = dimensions[0];
    maxy = dimensions[0];
    maxz = dimensions[0];
    break;
  case arm_navigation_msgs::Shape::MESH:
    for (size_t i = 0; i < shape.vertices.size(); i++) {
      geometry_msgs::Point p = shape.vertices[i];
      if (p.x < minx) {
	minx = p.x;
      }
      if (p.y < miny) {
	miny = p.y;
      }
      if (p.z < minz) {
	minz = p.z;
      }
      if (p.x > maxx) {
	maxx = p.x;
      }
      if (p.y > maxy) {
	maxy = p.y;
      }
      if (p.z > maxz) {
	maxz = p.z;
      }
    }
    break;
  default:
    ROS_ERROR("Unknown shape type: %d", shape.type);
    return corners;
  }
  geometry_msgs::Point c;
  c.x = minx;
  c.y = miny;
  c.z = minz;
  corners[0] = c;
  c.z = maxz;
  corners[1] = c;
  c.y = maxy;
  corners[2] = c;
  c.z = minz;
  corners[3] = c;
  c.x = maxx;
  corners[4] = c;
  c.y = miny;
  corners[5] = c;
  c.z = maxz;
  corners[6] = c;
  c.y = maxy;
  corners[7] = c;
  return corners;
}

geometry_msgs::Point darrt::object_bottom_point
(const arm_navigation_msgs::CollisionObject &obj,
 const geometry_msgs::Pose &pose) {
  double minz = MATH_INF;
  geometry_msgs::Point bpt;
  for (unsigned int i = 0; i < obj.shapes.size(); i++) {
    std::vector<geometry_msgs::Point> obj_corners = corners(obj.shapes[i]);
    for (unsigned int j = 0; j < obj_corners.size(); j++) {
      geometry_msgs::Point pt = transform_point(transform_point(obj_corners[j], obj.poses[i]),
						pose);
      if (pt.z < minz) {
	minz = pt.z;
	bpt = pt;
      }
    }
  }
  return bpt;
}

geometry_msgs::Point darrt::object_top_point
(const arm_navigation_msgs::CollisionObject &obj,
 const geometry_msgs::Pose &pose) {
  double maxz = -1;
  geometry_msgs::Point tpt;
  for (unsigned int i = 0; i < obj.shapes.size(); i++) {
    std::vector<geometry_msgs::Point> obj_corners = corners(obj.shapes[i]);
    for (unsigned int j = 0; j < obj_corners.size(); j++) {
      geometry_msgs::Point pt = transform_point(transform_point(obj_corners[j], obj.poses[i]),
						pose);
      if (pt.z > maxz) {
	maxz = pt.z;
	tpt = pt;
      }
    }
  }
  return tpt;
}
    
geometry_msgs::Vector3 darrt::point_to_vector(const geometry_msgs::Point &p) {
  geometry_msgs::Vector3 v;
  v.x = p.x;
  v.y = p.y;
  v.z = p.z;
  return v;
}

geometry_msgs::Point darrt::vector_to_point(const geometry_msgs::Vector3 &v) {
  geometry_msgs::Point p;
  p.x = v.x;
  p.y = v.y;
  p.z = v.z;
  return p;
}

geometry_msgs::Vector3 darrt::cross(const geometry_msgs::Vector3 &u, const geometry_msgs::Vector3 &v) {
  geometry_msgs::Vector3 c;
  c.x = u.y*v.z - u.z*v.y;
  c.y = u.z*v.x - u.x*v.z;
  c.z = u.x*v.y - u.y*v.x;
  return c;
}

double darrt::dot(const geometry_msgs::Vector3 &u, const geometry_msgs::Vector3 &v) {
  return u.x*v.x + u.y*v.y + u.z*v.z;
}

double darrt::norm(const geometry_msgs::Vector3 &v) {
  return sqrt(dot(v, v));
}

geometry_msgs::Vector3 darrt::subtract(const geometry_msgs::Vector3 &u, const geometry_msgs::Vector3 &v) {
  geometry_msgs::Vector3 s;
  s.x = u.x - v.x;
  s.y = u.y - v.y;
  s.z = u.z - v.z;
  return s;
}

darrt::Line::Line(const geometry_msgs::Point &start, const geometry_msgs::Point &end) {
  p1 = start;
  p2 = end;
  if (fabs(p1.x - p2.x) < DIST_EPS) {
    slope = MATH_INF;
    y_int = (p1.x+p2.x)/2.0;
    vertical = true;
  } else {
    slope = (p1.y - p2.y)/(p1.x - p2.x);
    y_int = p1.y - slope*p1.x;
    vertical = false;
  }
  y_min = p1.y;
  y_max = p2.y;
  if (p2.y < p1.y) {
    y_min = p2.y;
    y_max = p1.y;
  }
  x_min = p1.x;
  x_max = p2.x;
  if (p2.x < p1.x) {
    x_min = p2.x;
    x_max = p1.x;
  }
}

double darrt::Line::length() const {
  return sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y));
}

geometry_msgs::Point darrt::Line::nearest_point(const geometry_msgs::Point &p) const {
  double d = length();
  if (d < DIST_EPS) {
    return p1;
  }
  geometry_msgs::Point n;
  double u = ((p.x - p1.x)*(p2.x - p1.x) +
	      (p.y - p1.y)*(p2.y - p1.y))/(d*d);
  n.x = p1.x + u*(p2.x - p1.x);
  n.y = p1.y + u*(p2.y - p1.y);
  n.z = 0.0;
  if (contains(n)) {
    return n;
  }
  if (darrt::distance(p, p1) < darrt::distance(p, p2)) {
    return p1;
  }
  return p2;
}

bool darrt::Line::contains(const geometry_msgs::Point &p) const {
  if (vertical) {
    return fabs(p.x - y_int) < DIST_EPS && 
      p.y >= y_min - DIST_EPS && p.y <= y_max + DIST_EPS;
  }
  if (fabs(p.y - (slope*p.x + y_int)) > DIST_EPS) {
    return false;
  }
  return p.x >= x_min - DIST_EPS && p.x <= x_max + DIST_EPS &&
    p.y >= y_min - DIST_EPS && p.y <= y_max + DIST_EPS;
}

bool darrt::Line::intersection(const Line &l, geometry_msgs::Point &p) const {
  if ((vertical && l.vertical) || fabs(slope - l.slope) < DIST_EPS) {
    if (contains(l.p1)) {
      p = l.p1;
      return true;
    }
    double ds = MATH_INF;
    double df = MATH_INF;
    bool lcontains = false;
    if (l.contains(p1)) {
      ds = darrt::distance(l.p1, p1);
      lcontains = true;
    }
    if (l.contains(p2)) {
      df = darrt::distance(l.p1, p2);
      lcontains = true;
    }
    if (lcontains) {
      if (ds < df) {
	p = p1;
	return true;
      }
      p = p2;
      return true;
    }
    return false;
  }
  p.z = 0;
  if (vertical) {
    p.x = y_int;
    p.y = l.slope*p.x + l.y_int;
  } else if (l.vertical) {
    p.x = l.y_int;
    p.y = slope*p.x + y_int;
  } else {
    p.x = (y_int - l.y_int)/(l.slope - slope);
    p.y = slope*p.x + y_int;
  }
  if (contains(p) && l.contains(p)) {
    return true;
  }
  return false;
}
