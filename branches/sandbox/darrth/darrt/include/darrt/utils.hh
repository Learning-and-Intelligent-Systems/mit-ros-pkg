#ifndef __DARRT_UTILS_HH__
#define __DARRT_UTILS_HH__

#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <string>

//ros includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>


#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>

#define makestring(x) boost::lexical_cast<std::string>(x)

namespace ob = ompl::base;
namespace gm = geometry_msgs;

namespace darrt {

  const double EPSILON = 0.00001;
  const double MATH_INF = std::numeric_limits<double>::infinity();
  const double MATH_PI = boost::math::constants::pi<double>();
  const double DIST_EPS = 0.001;
  const double ANGLE_EPS = 0.01;
  //a factor to convert between the distances
  const double OBJECT_ROBOT_CONVERSION = 1.0;
  enum DEBUGLEVEL {DSILENT, DMINIMAL, DRRT, DPROPAGATE, DDISTANCE, DBETWEEN,
		   DCOLLISIONS, DTRANSFORMS};
  extern int debug_level;
  extern bool do_pause;

  bool inrange(double f1, double f2, double eps=EPSILON);
  double wrap_angle(double a);
  double wrap_angle_positive(double a);
  double angular_distance(double a1, double a2);
  visualization_msgs::Marker make_marker
  (geometry_msgs::PoseStamped ps, unsigned int type, double scale, double r, 
   double g, double b, double a, std::string ns, int id);
  void display_marker(const ros::Publisher &viz_pub,
		      geometry_msgs::PoseStamped ps, unsigned int type,
		      double scale, double r, double g, double b,
		      double a, std::string ns, int id);
  double distance(const geometry_msgs::Point &p1,
		  const geometry_msgs::Point &p2);
  double distance(const geometry_msgs::Quaternion &q1,
		  const geometry_msgs::Quaternion &q2);
  double quaternion_to_phi(const geometry_msgs::Quaternion &q);
  double quaternion_to_theta(const geometry_msgs::Quaternion &q);
  double quaternion_to_psi(const geometry_msgs::Quaternion &q);
  double quaternion_to_yaw(const geometry_msgs::Quaternion &q);
  void yaw_to_quaternion(double a, geometry_msgs::Quaternion &q);
  void apply_yaw_to_quaternion(double a, const geometry_msgs::Quaternion &q,
			       geometry_msgs::Quaternion &result);
  void quaternion_to_euler(const geometry_msgs::Quaternion &q, double &phi, 
			   double &theta, double &psi);
  void euler_to_quaternion(double phi, double theta, double psi,
			   geometry_msgs::Quaternion &q);
  void multiply(const geometry_msgs::Quaternion &q1,
		const geometry_msgs::Quaternion &q2,
		geometry_msgs::Quaternion &q);
  void conjugate(const geometry_msgs::Quaternion &q_in,
		 geometry_msgs::Quaternion &q_out);

  double between(const ob::SO2StateSpace::StateType *s,
		 const ob::SO2StateSpace::StateType *source,
		 const ob::SO2StateSpace::StateType *destination,
		 double eps=ANGLE_EPS);
  double between(const ob::SO3StateSpace::StateType *s,
		 const ob::SO3StateSpace::StateType *source,
		 const ob::SO3StateSpace::StateType *destination,
		 double eps=ANGLE_EPS);
  double angle_between(double a, double a1, double a2, double eps=ANGLE_EPS);
  double between(const ob::SE2StateSpace::StateType *s,
		 const ob::SE2StateSpace::StateType *source,
		 const ob::SE2StateSpace::StateType *destination,
		 double deps=DIST_EPS, double aeps=ANGLE_EPS);
  double between(const ob::RealVectorStateSpace::StateType *s,
		 const ob::RealVectorStateSpace::StateType *source,
		 const ob::RealVectorStateSpace::StateType *destination,
		 unsigned int dimension, double eps=DIST_EPS);
  double between(const gm::Point &p, const gm::Point &source,
		 const gm::Point &destination, double eps=DIST_EPS);
  double between(const std::vector<double> &vec,
		 const std::vector<double> &source,
		 const std::vector<double> &destination, double eps=DIST_EPS);
  double distance2D(double x1, double y1, double x2, double y2);
  void pause(std::string, int level=-1, bool always_pause=false);
  bool is_shutdown();


  //ob::PlannerTerminationCondition terminate_after(double time);
}

#endif //utils.hh
