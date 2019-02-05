#ifndef __DARRT_TYPES_HH__
#define __DARRT_TYPES_HH__

#include "darrt/primitive.hh"
#include "darrt/utils.hh"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <ompl/base/State.h>

namespace ob = ompl::base;

namespace darrt {

  class PrimitivePathEntry {
  public:
    const Primitive *primitive;
    std::string type;
    std::vector<const ob::State *> states;

    PrimitivePathEntry(const Primitive *p, std::string t, 
		       std::vector<const ob::State *> s) 
    {primitive = p; type = t; states = s;}
      
  };

  typedef std::vector<PrimitivePathEntry> PrimitivePath;

  class Displayable {

  public:
    enum ColorPalate {GOALPALATE, STARTPALATE, ORANGEPALATE,
		      PRIMARYPALATE, PASTELPALATE, BWPALATE,
		      REDPALATE, GREENPALATE, BLUEPALATE, PURPLEPALATE,
		      CYANPALATE, YELLOWPALATE};
    
    static void interpret_palate(ColorPalate p, float &r, float &g, float &b)
    {
      switch(p) {
      case PRIMARYPALATE:
	r = 0.0; g = 0.0; b = 1.0;
	return;
      case PASTELPALATE:
	r = 0.0; g = 0.5; b = 0.5;
	return;
      case BWPALATE:
	r = 0.0; g = 0.0; b = 0.0;
	return;
      case STARTPALATE:
	r = 1.0; g = 0.0; b = 1.0;
	return;
      case GOALPALATE:
	r = 0.0; g = 1.0; b = 0.0;
	return;
      case REDPALATE:
	r = 1.0; g = 0.0; b = 0.0;
	return;
      case GREENPALATE:
	r = 0.0; g = 1.0; b = 0.0;
	return;
      case BLUEPALATE:
	r = 0.0; g = 0.0; b = 1.0;
	return;
      case PURPLEPALATE:
	r = 1.0; g = 0.0; b = 1.0;
	return;
      case CYANPALATE:
	r = 0.0; g = 1.0; b = 1.0;
	return;
      case YELLOWPALATE:
	r = 1.0; g = 1.0; b = 0.0;
	return;
      case ORANGEPALATE:
	r = 1.0; g = 0.5; b = 0.0;
	return;
      default:
	ROS_WARN("PushingStateSpace::display_state: unrecognized color palate");
	r = 0.0; g = 0.0; b = 1.0;
	return;
      }
    }
    


    Displayable(std::string topic_name) {
      ros::NodeHandle n("~");
      viz_pub_ = n.advertise<visualization_msgs::MarkerArray>
	(topic_name, 100);
    }
    void display(visualization_msgs::MarkerArray marray) {
      viz_pub_.publish(marray);
    }

    virtual std::string state_string(const ob::State *state) const=0;

    virtual visualization_msgs::MarkerArray displayable_state
    (const ob::State *state, std::string ns, int id, double scale=0.05,
     ColorPalate p=PRIMARYPALATE, double alpha=0.5, bool minimal=false) const = 0;
    
    virtual void display_state(const ob::State *state,
			       std::string ns, int id, double scale=0.05,
			       ColorPalate p=PRIMARYPALATE,
			       double alpha=0.5, bool minimal=false) const {
      viz_pub_.publish(displayable_state(state, ns, id, scale, p, alpha, minimal));
    }
    virtual void display_states(const std::vector<ob::State *> &states,
				std::string ns, int id, double scale=0.05,
				ColorPalate p=PRIMARYPALATE,
				double alpha=0.5, bool minimal=false) const {
      visualization_msgs::MarkerArray marray;
      for (size_t i = 0; i < states.size(); i++) {
	visualization_msgs::MarkerArray od = displayable_state
	  (states[i], ns, id, scale, p, alpha, minimal);
	for (size_t j = 0; j < od.markers.size(); j++) {
	  marray.markers.push_back(od.markers[j]);
	}
      }
      viz_pub_.publish(marray);
    }
    // virtual void display_states(const std::vector<const ob::State *> &states,
    // 				std::string ns, int id, double scale=0.05,
    // 				ColorPalate p=PRIMARYPALATE,
    // 				double alpha=0.5, bool) const {
    //   visualization_msgs::MarkerArray marray;
    //   for (size_t i = 0; i < states.size(); i++) {
    // 	visualization_msgs::MarkerArray od = displayable_state
    // 	  (states[i], ns, id, scale, p, alpha);
    // 	for (size_t j = 0; j < od.markers.size(); j++) {
    // 	  marray.markers.push_back(od.markers[j]);
    // 	}
    //   }
    //   viz_pub_.publish(marray);
    // }
  protected:
    ros::Publisher viz_pub_;
  };

  class Approximable {
  public:
    virtual bool near_states
    (const ob::State *s1, const ob::State *s2,
     double deps=DIST_EPS, double aeps=ANGLE_EPS) const=0;
    virtual double between(const ob::State *state,
			   const ob::State *source,
			   const ob::State *destination,
			   double deps=DIST_EPS, double aeps=ANGLE_EPS) const
    {return -1.0;}
  };

}

#endif //types.hh
