#ifndef __DARRT_GOAL_HH__
#define __DARRT_GOAL_HH__

#include <map>
#include <limits>

#include "darrt/control.hh"
#include "darrt/object_types.hh"
#include "darrt/primitive.hh"
#include "darrt/robot_base_space.hh"
#include "darrt/support_surface.hh"
#include "darrt/types.hh"
#include "darrt/utils.hh"

#include <arm_navigation_msgs/OrderedCollisionOperations.h>
#include <arm_navigation_msgs/Shape.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/RealVectorBounds.h>

namespace gm = geometry_msgs;
namespace an = arm_navigation_msgs;
namespace ob = ompl::base;
namespace oc = ompl::control;

namespace darrt {

  //identity projection function
  void goalProjectionFunction(const ob::State *from, ob::State *sample);

  class Body {
  public:
    an::Shape shape;
    gm::Pose pose;

    Body() {}
    Body(an::Shape s, gm::Pose p) {shape = s; pose = p;}
  };
  typedef std::vector<Body> Border;

  //only point, points, and rectangles have been tested!!!
  
  class Goal;

  class SingleGoal {
  public:
    enum Type {POSE, POSES, ROTATIONALLY_SYMMETRIC_POSE, ROTATIONALLY_SYMMETRIC_POSES, SHAPE, BORDER, PRIMITIVE};
    //goal type
    Type type;

    //one of these should be filled in
    gm::Pose pose;
    std::vector<gm::Pose> poses;
    Body shape;
    Border border;
    const Primitive *primitive;
    
    //the primitive goal also wants a later
    //goal if that exists
    const darrt::Goal *primitive_actual_goal;
    
    //an::CollisionObject object;

    //bounds on the whole state space
    ob::RealVectorBounds bounds;
    //allowed time
    double allowed_time;
    double threshold;
    SingleGoal() : bounds(0) {allowed_time = 1.0; threshold=DIST_EPS; 
      primitive=NULL; primitive_actual_goal=NULL;}

    ob::Goal *satisfiable_goal(const ob::SpaceInformationPtr &si) const;

    visualization_msgs::MarkerArray displayable(std::string frame_id) const;

  };

  //The sort of numbers we want to be able to set with
  //a parameter service
  class Params {
  public:
    double object_allowed_time;
    double darrt_allowed_time;
    double object_threshold;
    double darrt_threshold;
    unsigned int darrt_tries;
    bool use_darrt;
    double goal_bias;
    int debug_level;
    bool interactive;
    bool forward;
    ob::RealVectorBounds bounds;
    SingleGoal::Type object_goal_type;
    Params() : bounds(3) {object_allowed_time = 10.0; darrt_allowed_time = 15.0;
      object_threshold = 0.02; darrt_threshold = 0.02; darrt_tries = 1; use_darrt=true;
      goal_bias=0.1; debug_level=DMINIMAL; bounds.setLow(-5); bounds.setHigh(5);
      bounds.setLow(2, 0); bounds.setHigh(2, 1.7); 
      object_goal_type = SingleGoal::PRIMITIVE; interactive=false; forward=false;}
  };


  class Goal : public std::map<std::string, SingleGoal> {
  public:
    std::vector<const DARRTObject *> objects;
    std::vector< std::pair<const Primitive *, double> > primitives;
    Params params;
    std::vector<std::string> robot_groups;
    SupportSurfaceList support_surfaces;
    
    void copyInfo(const darrt::Goal &goal);
    ob::Goal *satisfiable_goal(const oc::SpaceInformationPtr &si,
			       const ob::State *starting_state) const;
  };

  class RandomGoalStates : public ob::GoalStates {
  public:
    RandomGoalStates(const ob::SpaceInformationPtr &si);
    void sampleGoal(ob::State *state) const;
    void display(std::string ns, int id, double r, double g, double b, 
		 double a) const;
    
  protected:
    ros::Publisher viz_;
  };

  class RotationallySymmetricGoalState : public ob::GoalSampleableRegion {
  public:
    RotationallySymmetricGoalState(const ob::SpaceInformationPtr &si,
				   const gm::Pose &pose) :
      ob::GoalSampleableRegion(si) {pose_ = pose;}
    double distanceGoal(const ob::State *st) const;
    void sampleGoal(ob::State *st) const;
    unsigned int maxSampleCount() const
    {return std::numeric_limits<unsigned int>::infinity()-1;}
    bool canSample() const {return true;}
    const gm::Pose &getPose() const {return pose_;}
  protected:
    gm::Pose pose_;
  };

  class RotationallySymmetricGoalStates : public ob::GoalSampleableRegion {
  public:
    RotationallySymmetricGoalStates(const ob::SpaceInformationPtr &si,
				    const std::vector<gm::Pose> &poses);
    double distanceGoal(const ob::State *st) const;
    void sampleGoal(ob::State *st) const;
    unsigned int maxSampleCount() const
    {return std::numeric_limits<unsigned int>::infinity()-1;}
    bool canSample() const {return states_.size();}
    ~RotationallySymmetricGoalStates();
  protected:
    std::vector<RotationallySymmetricGoalState *> states_;
  };

  class PrimitiveGoal : public ob::GoalSampleableRegion {
  public:
    PrimitiveGoal(const oc::SpaceInformationPtr &si,
		  const Primitive *prim,
		  const ob::State *starting_state,
		  const darrt::Goal *actual_goal);
    void setStartingState(const ob::State *starting_state)
    {starting_state_ = starting_state;}
    void setActualGoal(const darrt::Goal *actual_goal);
    bool isSatisfied(const ob::State *st, double *distance) const;
    void print(std::ostream &out=std::cout) const;
    void sampleGoal(ob::State *state) const;
    bool couldSample() const {ROS_INFO("Starting state is %p", starting_state_); return starting_state_ != NULL;}
    unsigned int maxSampleCount() const
    {ROS_INFO("Returning a big number!"); return std::numeric_limits<unsigned int>::infinity()-1;}
    double distanceGoal(const ob::State *st) const
    {pause("Call to distanceGoal in primitive goal, which is unused!");
      return MATH_INF;}
    ~PrimitiveGoal();

  protected:
    const oc::SpaceInformationPtr siC_;
    const Primitive *prim_;
    const ob::State *starting_state_;
    ob::StateSamplerPtr state_sampler_;
    ob::State *sampled_state_;
    oc::Control *sampled_control_;
    oc::DirectedControlSamplerPtr control_sampler_;
    ob::GoalSampleableRegion *actual_goal_;
    oc::StatePropagatorPtr propagator_;

    int robot_index_;
    int base_index_;

    const RobotBaseStateSpace *base_space_;
  };


  //also define a goal sampleable region
  class GoalShape : public ob::GoalSampleableRegion {
  public:
    GoalShape(const ob::SpaceInformationPtr &si,
	      const Body &body);
    
    bool isSatisfied(const ob::State *st) const
    {return isSatisfied(st, NULL);}
    bool isSatisfied(const ob::State *st, double *distance) const;

    double distanceGoal(const ob::State *st) const
    {pause("Call to distanceGoal in shape goal, which is unused!");
      return MATH_INF;}
		     
    void sampleGoal(ob::State *state) const;
    unsigned int maxSampleCount() const 
    {return std::numeric_limits<unsigned int>::infinity();}
    bool canSample() const {return true;}

    void display(std::string ns, int id, double r, double g, double b, 
		 double a) const;
  protected:
    Body body_;
    ros::Publisher viz_;

    //returns a sample in the shape's own frame
    gm::Point sample() const;
  };

  
  class GoalBorder : public ob::GoalSampleableRegion {
  public:
    GoalBorder(const ob::SpaceInformationPtr &si, 
	       const Border &border); 
    void sampleGoal(ob::State *state) const;
    unsigned int maxSampleCount() const 
    {return std::numeric_limits<unsigned int>::infinity();}

    bool isSatisfied(const ob::State *st) const
    {return isSatisfied(st, NULL);}
    bool isSatisfied(const ob::State *st, double *distance) const;

    double distanceGoal(const ob::State *st) const
    {pause("Call to distanceGoal in border goal, which is unused!");
      return MATH_INF;}
    
    void display(std::string ns, int id, double r, double g, double b, double a)
      const;

    unsigned int num_shapes() const {return samplers_.size();}

    ~GoalBorder();
  
  protected:
    std::vector<GoalShape *> samplers_;
    ros::Publisher viz_;
  };

  class CompoundGoal : public ob::GoalSampleableRegion {
  public:
    CompoundGoal(const oc::SpaceInformationPtr &si,
		 const darrt::Goal &goal,
		 const ob::State *starting_state);
    
    
    void sampleGoal(ob::State *state) const;
    unsigned int maxSampleCount() const {return max_samples_;}

    bool isSatisfied(const ob::State *st, double *distance) const;
    bool isSatisfied(const ob::State *st) const
    {return isSatisfied(st, NULL);}

    double distanceGoal(const ob::State *st) const
    {pause("Call to distanceGoal in compound goal, which is unused!");
      return MATH_INF;}

    void display(std::string ns, Displayable::ColorPalate palate, 
		 double a=0.5) const;
    ~CompoundGoal();

  protected:
    const oc::SpaceInformationPtr siC_;
    std::vector<unsigned int> active_spaces_, sampleable_spaces_;
    std::map<unsigned int, ob::Goal *> goals_;
    ros::Publisher viz_;
    unsigned int max_samples_;
    ob::StateSamplerPtr state_sampler_;
    const ob::State *starting_state_;
    oc::DirectedControlSamplerPtr control_sampler_;
    oc::Control *sampled_control_;
  };


}
    

#endif //goal.hh
