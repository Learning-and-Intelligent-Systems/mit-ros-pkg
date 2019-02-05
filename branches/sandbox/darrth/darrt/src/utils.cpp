#include "darrt/utils.hh"
#include <cmath>
#include <visualization_msgs/Marker.h>

int darrt::debug_level = DMINIMAL;
bool darrt::do_pause = true;

bool darrt::inrange(double f1, double f2, double eps) {
  if (fabs(f1 - f2) < eps) {
    return true;
  }
  return false;
}

double darrt::wrap_angle(double a) {
  double pi = boost::math::constants::pi<double>();
  while (a < -1.0*pi) {
    a += 2.0*pi;
  }
  while (a > pi) {
    a -= 2.0*pi;
  }
  return a;
}

double darrt::wrap_angle_positive(double a) {
  double pi = boost::math::constants::pi<double>();
  while (a < 0) {
    a += 2.0*pi;
  }
  while (a > 2.0*pi) {
    a -= 2.0*pi;
  }
  return a;
}

double darrt::angular_distance(double a1, double a2) {
  a1 = wrap_angle_positive(a1);
  a2 = wrap_angle_positive(a2);
  double d = wrap_angle_positive(a1 - a2);
  if (d > boost::math::constants::pi<double>()) {
    d = wrap_angle_positive(a2 - a1);
  }
  return d;
}

visualization_msgs::Marker darrt::make_marker
(geometry_msgs::PoseStamped ps, unsigned int type, double scale, 
 double r, double g, double b, double a, std::string ns, int id) {
  visualization_msgs::Marker marker;
  marker.header = ps.header;
  marker.header.stamp = ros::Time(0);
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = ps.pose;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.lifetime = ros::Duration();
  return marker;
}

void darrt::display_marker(const ros::Publisher &viz_pub,
			   geometry_msgs::PoseStamped ps, 
			   unsigned int type, double scale, 
			   double r, double g, double b,
			   double a, std::string ns, int id) {
  visualization_msgs::Marker marker = make_marker(ps, type, scale, r, g, b,
						  a, ns, id);
  for (int i = 0; i < 10; i++) {
    //for some reason you have to publish over and over to
    //see the darn thing
    viz_pub.publish(marker);
  }
}

double darrt::distance(const geometry_msgs::Point &p1,
		       const geometry_msgs::Point &p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

double darrt::distance(const geometry_msgs::Quaternion &q1,
		       const geometry_msgs::Quaternion &q2) {
  //inner product
  double ip = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w;
  if (ip > 1.0) {
    ip = 1.0;
  }
  if (ip < -1.0) {
    ip = -1.0;
  }
  return acos(2.0*ip*ip - 1.0);

}

double darrt::quaternion_to_phi(const geometry_msgs::Quaternion &q) {
  return atan2(2.0*(q.w*q.x + q.y*q.z), 1.0 - 2.0*(q.x*q.x + q.y*q.y));
} 

double darrt::quaternion_to_theta(const geometry_msgs::Quaternion &q) {
  return asin(2.0*(q.w*q.y - q.z*q.x));
} 

//these are the same thing
double darrt::quaternion_to_psi(const geometry_msgs::Quaternion &q) {
  return atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
} 

double darrt::quaternion_to_yaw(const geometry_msgs::Quaternion &q) {
  return quaternion_to_psi(q);
} 



void darrt::yaw_to_quaternion(double a, 
			      geometry_msgs::Quaternion &q) {
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(a/2.0);
  q.w = cos(a/2.0);
}

//rotate around the actual z axis
void darrt::apply_yaw_to_quaternion
(double a, const geometry_msgs::Quaternion &q, 
 geometry_msgs::Quaternion &result) {
  geometry_msgs::Quaternion zq;
  euler_to_quaternion(0, 0, a, zq);
  //empirically... this works for 90 degrees around y
  //not sure if it's actually right don't care too much right
  //now
  multiply(zq, q, result);
}

void darrt::quaternion_to_euler
(const geometry_msgs::Quaternion &q, double &phi, double &theta, double &psi) {
  phi = quaternion_to_phi(q);
  theta = quaternion_to_theta(q);
  psi = quaternion_to_psi(q);
}
  

//phi=pitch, theta=roll, psi=yaw
void darrt::euler_to_quaternion(double phi, double theta, 
				double psi,
				geometry_msgs::Quaternion &q) {
  q.x = sin(phi/2.0)*cos(theta/2.0)*cos(psi/2.0) -
    cos(phi/2.0)*sin(theta/2.0)*sin(psi/2.0);
  q.y = cos(phi/2.0)*sin(theta/2.0)*cos(psi/2.0) +
    sin(phi/2.0)*cos(theta/2.0)*sin(psi/2.0);
  q.z = cos(phi/2.0)*cos(theta/2.0)*sin(psi/2.0) -
    sin(phi/2.0)*sin(theta/2.0)*cos(psi/2.0);
  q.w = cos(phi/2.0)*cos(theta/2.0)*cos(psi/2.0) +
    sin(phi/2.0)*sin(theta/2.0)*sin(psi/2.0);
}

void darrt::multiply(const geometry_msgs::Quaternion &q1,
		     const geometry_msgs::Quaternion &q2,
		     geometry_msgs::Quaternion &q) {
  q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
  q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
  q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
  q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
}

void darrt::conjugate(const geometry_msgs::Quaternion &q_in,
		      geometry_msgs::Quaternion &q_out) {
  q_out.x = q_in.x;
  q_out.y = q_in.y;
  q_out.z = q_in.z;
  q_out.w = -1.0*q_in.w;
}

//less than zero indicates not between
//greater than 1 indictes everything is equal

double darrt::between(const ob::SO2StateSpace::StateType *s,
		      const ob::SO2StateSpace::StateType *source,
		      const ob::SO2StateSpace::StateType *destination,
		      double eps) {

  //this are just angles... don't think about how much i hate angles...
  double a = wrap_angle_positive(s->value);
  double a1 = wrap_angle_positive(source->value);
  double a2 = wrap_angle_positive(destination->value);
  return angle_between(a, a1, a2);
}

double darrt::between(const ob::SO3StateSpace::StateType *s,
		      const ob::SO3StateSpace::StateType *source,
		      const ob::SO3StateSpace::StateType *destination,
		      double eps) {
  geometry_msgs::Quaternion q, q1, q2;
  q.x = s->x;
  q.y = s->y;
  q.z = s->z;
  q.w = s->w;

  q1.x = source->x;
  q1.y = source->y;
  q1.z = source->z;
  q1.w = source->w;

  q2.x = destination->x;
  q2.y = destination->y;
  q2.z = destination->z;
  q2.w = destination->w;
  
  double f = angle_between(quaternion_to_psi(q), quaternion_to_psi(q1),
			   quaternion_to_psi(q2), eps);
  if (f < 0) {
    return f;
  }


  double fth = angle_between(quaternion_to_theta(q), quaternion_to_theta(q1),
			     quaternion_to_theta(q2), eps);
  if (fth < 0) {
    return fth;
  }

  if (fabs(f - fth) > eps && f < 1.0 && fth < 1.0) {
    return -1.0;
  }

  if (f > 1.0) {
    f = fth;
  } else if (fth < 1.0) {
    f = (f + fth)/2.0;
  }

  double fphi = angle_between(quaternion_to_phi(q), quaternion_to_phi(q1),
			      quaternion_to_phi(q2), eps);

  if (fphi < 0) {
    return fphi;
  }

  if (fphi > 1.0) {
    return f;
  }

  if (f < 1.0 && fabs(fphi - f) > eps) {
    return -1.0;
  }

  if (f > 1.0) {
    return fphi;
  }

  return (2.0*f + fphi)/3.0;
}

double darrt::angle_between(double a, double a1, double a2, double eps) {
  if (angular_distance(a1, a2) < eps) {
    if (angular_distance(a, a1) < eps && angular_distance(a, a2) < eps) {
      return 2.0;
    }
    return -1.0;
  }
  
  double mina = a1;
  double maxa = a2;
  double pi = boost::math::constants::pi<double>();
  if (a1 > a2) {
    mina = a2;
    maxa = a1;
  }

  double f = angular_distance(a1, a)/angular_distance(a2, a1);
  if (debug_level == DBETWEEN) {
    ROS_INFO("fraction = %f, mina = %f, maxa = %f, a = %f eps = %f",
	   f, mina, maxa, a, eps);
  }
  if (f > 1.0) {
    return -1.0;
  }

  if (fabs(maxa - mina) < pi + eps &&
      fabs(maxa - mina) > pi - eps) {
    //anything is ok
    return f;
  }

  if (maxa - mina < pi) {
    if (a < mina - eps || a > maxa + eps) {
      return -1.0;
    }
    return f;
  }

  
  if (a > mina + eps && a < maxa - eps) {
    return -1.0;
  }
  return f;
}

double darrt::between(const ob::SE2StateSpace::StateType *s,
		      const ob::SE2StateSpace::StateType *source,
		      const ob::SE2StateSpace::StateType *destination,
		      double deps, double aeps) {
  gm::Point p, r, d;
  p.x = s->getX();
  p.y = s->getY();
  p.z = 0.0;

  r.x = source->getX();
  r.y = source->getY();
  r.z = 0.0;

  d.x = destination->getX();
  d.y = destination->getY();
  d.z = 0.0;

  double f = between(p, r, d, deps);
  if (f < 0) {
    return -1.0;
  }
  double a = angle_between(s->getYaw(), source->getYaw(), destination->getYaw(), aeps);
  if (a < 0) {
    return -1.0;
  }
  if (a > 1.1) {
    return f;
  }
  if (f > 1.1) {
    return a;
  }
  if (fabs(f - a) > deps) {
    return -1;
  }
  return f;
}

double darrt::between(const gm::Point &p, const gm::Point &source,
		      const gm::Point &destination, double eps) {
  std::vector<double> pv(3), sv(3), dv(3);
  pv[0] = p.x;
  pv[1] = p.y;
  pv[2] = p.z;
  sv[0] = source.x;
  sv[1] = source.y;
  sv[2] = source.z;
  dv[0] = destination.x;
  dv[1] = destination.y;
  dv[2] = destination.z;
  return between(pv, sv, dv, eps);
}

double darrt::between(const ob::RealVectorStateSpace::StateType *s,
		      const ob::RealVectorStateSpace::StateType *source,
		      const ob::RealVectorStateSpace::StateType *destination,
		      unsigned int dimension, double eps) {
  if (debug_level == DBETWEEN) {
    ROS_INFO("Checking between for real vector state space");
  }
  std::vector<double> pv(dimension), sv(dimension), dv(dimension);
  for (unsigned int i = 0; i < dimension; i++) {
    pv[i] = s->values[i];
    sv[i] = source->values[i];
    dv[i] = destination->values[i];
  }
  return between(pv, sv, dv, eps);
}

double darrt::between(const std::vector<double> &vec,
		      const std::vector<double> &source,
		      const std::vector<double> &destination, double eps) {
  if (vec.size() != source.size() || vec.size() != destination.size()) {
    pause("Call to between with vectors of different sizes ("
	  +makestring(vec.size())+", "+makestring(source.size())+
	  ", "+makestring(destination.size())+")");
    return false;
  }
  double f = 2.0;
  for (unsigned int i = 0; i < vec.size(); i++) {
    double v = vec[i];
    double v1 = source[i];
    double v2 = destination[i];
    if (debug_level == DBETWEEN) {
      ROS_INFO("v = %f, v1 = %f, v2 = %f", v, v1, v2);
    }
    if (fabs(v2 - v1) < eps) {
      if (fabs(v2 - v) > eps || fabs(v1 - v) > eps) {
	return -1.0;
      }
      continue;
    }

    double r = (v - v1)/(v2 - v1);
    if (debug_level == DBETWEEN) {
      ROS_INFO("r = %f, f = %f", r, f);
    }
    if (r > 1.0 || r < 0.0) {
      if (r > 1.0 && r < 1.0 + eps) {
	r = 1.0;
      } else if (r < 0.0 && r > -1.0*eps) {
	r = 0.0;
      } else {
	return -1.0;
      }
    }

    if (f > 1.0) {
      f = r;
    } else if (r < 1.1 && fabs(f - r) > eps) {
      return -1.0;
    }
  }
  
  return f;

}

void darrt::pause(std::string msg, int level, bool always_pause) {
  switch(level) {
  case 0:
    ROS_INFO("%s", msg.c_str());
    break;
  case 1:
    ROS_WARN("%s", msg.c_str());
    break;
  default:
    ROS_ERROR("%s", msg.c_str());
    break;
  }
  //don't allow any pauses to happen in released version
  if (ros::ok() && (do_pause || always_pause || level < 0 || level > 1)) {
    std::string buff;
    getline(std::cin, buff);
  }
}

bool darrt::is_shutdown() {
  return !ros::ok();
}

double darrt::distance2D(double x1, double y1, double x2, double y2) {
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}
