#include "darrt/utils.hh"
#include "darrt/support_surface.hh"
#include "darrt/transform_ros_types.hh"
#include <cmath>

//find the bottom point
bool darrt::SupportSurface::supporting
(const an::CollisionObject &obj, const gm::Pose &pose, double eps) const {
  if (debug_level >= DPROPAGATE) {
    ROS_INFO_STREAM("The bottom point of object " << obj.id << " is\n" << object_bottom_point(obj, pose));
  }
  return on_surface(object_bottom_point(obj, pose), eps);
}

darrt::MeshSurface::MeshSurface(std::string name, const gm::Pose &pose, 
				const an::Shape &mesh) :
  SupportSurface(name) {
  set_pose_and_mesh(pose, mesh);
}

void darrt::MeshSurface::set_pose_and_mesh(const gm::Pose &pose, 
					   const an::Shape &mesh) {
  
  pose_ = pose;
  mesh_ = mesh;
  if (!mesh_.vertices.size()) {
    ROS_ERROR("Creation of mesh surface with no points!");
  }
  
  //right now, we require that surfaces be approximately normal to the z axis
  double z = mesh.vertices[0].z;
  xmin_ = MATH_INF;
  ymin_ = MATH_INF;
  xmax_ = -1.0*MATH_INF;
  ymax_ = -1.0*MATH_INF;
  for (unsigned int i = 1; i < mesh.vertices.size(); i++) {
    z += mesh.vertices[i].z;
    if (mesh.vertices[i].x < xmin_) {
      xmin_ = mesh.vertices[i].x;
    }
    if (mesh.vertices[i].y < ymin_) {
      ymin_ = mesh.vertices[i].y;
    }
    if (mesh.vertices[i].x > xmax_) {
      xmax_ = mesh.vertices[i].x;
    }
    if (mesh.vertices[i].y > ymax_) {
      ymax_ = mesh.vertices[i].y;
    }
  }
  pose_.position.z += z/(double)mesh.vertices.size();

}

bool darrt::MeshSurface::on_surface(const gm::Point &pt,
				    double eps) const {
  gm::Point p = inverse_transform_point(pt, pose_);
  //assumes the normal is along the z axis
  if (fabs(p.z) > eps) {
    return false;
  }

  gm::Point p1 = mesh_.vertices[mesh_.vertices.size()-1];
  gm::Point p2 = mesh_.vertices[0];
  double z = p1.x*(p2.y - p.y) + p2.x*(p.y - p1.y) + p.x*(p1.y - p2.y);
  int direction = 1;
  if (z < 0) {
    direction = -1;
  }
  if (fabs(z) < eps) {
    direction = 0;
  }

  for (unsigned int i = 0; i < mesh_.vertices.size()-1; i++) {
    gm::Point p1 = mesh_.vertices[i];
    gm::Point p2 = mesh_.vertices[i+1];
    double z = p1.x*(p2.y - p.y) + p2.x*(p.y - p1.y) + p.x*(p1.y - p2.y);
    if (fabs(z) < eps) {
      continue;
    }
    int sgn = 1;
    if (z < 0) {
      sgn = -1;
    }
    if (direction == 0) {
      direction = sgn;
    }
    if (sgn != direction) {
      return false;
    }
  }
  return true;
}

double darrt::MeshSurface::distance(const gm::Point &p) const {
  gm::Point tp = inverse_transform_point(p, pose_);
  double zdist = tp.z;
  tp.z = 0.0;
  if (on_surface(transform_point(tp, pose_))) {
    return fabs(zdist);
  }
  double mindist = MATH_INF;
  for (unsigned int i = 0; i < mesh_.vertices.size(); i++) {
    Line l(mesh_.vertices[i], mesh_.vertices[(i+1)%mesh_.vertices.size()]);
    double dist = darrt::distance(tp, l.nearest_point(tp));
    if (dist < mindist) {
      mindist = dist;
    }
  }
  return sqrt(mindist*mindist+zdist*zdist);
}

gm::Point darrt::MeshSurface::last_point_on_surface
(const gm::Point &p1, const gm::Point &p2) const {
  return last_point_on_surface(p1, p2, 0);
}

gm::Point darrt::MeshSurface::last_point_on_surface
(const gm::Point &p1, const gm::Point &p2, double offset) const {
  //project these points onto the surface
  gm::Point tp1 = inverse_transform_point(p1, pose_);
  tp1.z = 0.0;
  gm::Point ap1 = transform_point(tp1, pose_);
  gm::Point tp2 = inverse_transform_point(p2, pose_);
  tp2.z = 0.0;
  gm::Point ap2 = transform_point(tp2, pose_);
  if (!on_surface(ap1)) {
    ROS_ERROR("Request for last point on surface but original point doesn't project to surface");
    return p1;
  }
  //this is a bit sketchy since we assume the table isn't tilted but it actually is
  //mostly migated by flattening the table
  if (on_surface(ap2)) {
    //this might be closer than offset... deal with that later
    return p2;
  }
  Line l(tp1, tp2);

  for (unsigned int i = 0; i < mesh_.vertices.size(); i++) {
    Line b(mesh_.vertices[i], mesh_.vertices[(i+1)%mesh_.vertices.size()]);
    gm::Point intersect;
    //this is only the line segment... if there is an intersection
    //it should be the point on the table
    if (b.intersection(l, intersect)) {
      double dy = intersect.y - tp1.y;
      double dx = intersect.x - tp1.x;
      double dist = sqrt(dx*dx + dy*dy);
   
      double theta = atan2(l.p2.y - l.p1.y, l.p2.x - l.p1.x);
      double phi = atan2(b.p2.y - b.p1.y, b.p2.x - b.p1.x);
      double alpha = angular_distance(theta, phi);
      if (alpha < ANGLE_EPS) {
	return p1;
      }
      double d = offset/sin(alpha);
      if (fabs(d) > fabs(dist)) {
	return p1;
      }

      gm::Point lp;
      lp.x = intersect.x - fabs(d/dist)*dx;
      lp.y = intersect.y - fabs(d/dist)*dy;
      lp.z = intersect.z;
      return transform_point(lp, pose_);
    }
  }
  
  pause("Unable to find the last point on surface!!");
  return ap1;
}

gm::Point darrt::MeshSurface::randomPointOnSurface() const {
  gm::Point pt;
  while (true) {
    pt.x = xmin_ + rand()/(double)RAND_MAX*(xmax_ - xmin_);
    pt.y = ymin_ + rand()/(double)RAND_MAX*(ymax_ - ymin_);
    pt.z = 0;
    pt = transform_point(pt, pose_);
    if (on_surface(pt)) {
      return pt;
    }
  }
}
