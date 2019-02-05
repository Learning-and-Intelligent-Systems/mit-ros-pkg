#ifndef __DARRT_BLACK_BOX_PRIMITIVES__HH__
#define __DARRT_BLACK_BOX_PRIMITIVES__HH__

#include <map>

#include "darrt/collision_aware_types.hh"
#include "darrt/primitive.hh"
#include "darrt/space.hh"
#include "darrt/utils.hh"

#include "darrt_msgs/DARRTPrimitivePlanningAction.h"
#include "darrt_msgs/DARRTPrimitiveAction.h"

#include <actionlib/client/simple_action_client.h>

namespace darrt {

  class BlackBoxPrimitive : public TransferPrimitive {
  public:
    BlackBoxPrimitive(std::string planning_action_name);
    BlackBoxPrimitive(std::string planning_action_name,
		      std::string execution_action_name);
    virtual bool setup(const oc::SpaceInformation *si);
    virtual bool useful(const ob::State *source, 
			const ob::State *destination) const;
    virtual unsigned int sampleTo(const ob::State *source,
				  const ob::State *target,
				  PIList &clist) const;
    //i really don't like this... need a better way of doing distance!!!!
    virtual double distance_to_nearest_grasp
    (unsigned int objind,
     const ob::State *source, const ob::State *destination) const;
    
    virtual bool execute(const std::vector<const ob::State *> &path) const;
    virtual ~BlackBoxPrimitive() {delete planning_action_; delete execute_action_;}

  protected:
    std::string planning_action_name_, execution_action_name_;
    actionlib::SimpleActionClient<darrt_msgs::DARRTPrimitivePlanningAction> 
    *planning_action_;
    actionlib::SimpleActionClient<darrt_msgs::DARRTPrimitiveAction> 
    *execute_action_;
    const PR2ArmTransit *transit_; //this is to have properly named transits
    const DARRTStateSpace *dspace_;

    void get_goal(const ob::State *source, const ob::State *target, 
		  darrt_msgs::DARRTPrimitivePlanningGoal &goal) const;
    ob::State *get_first_state(unsigned int objind, const ob::State *source,
			       const ob::State *destination) const;
    double distance(const ob::State *source, const ob::State *destination) const;
  };

  class BlackBoxWithPregrasp : public BlackBoxPrimitive {
  public:
    typedef std::map<std::string, ob::State *> PreGraspMap;
    BlackBoxWithPregrasp(std::string planning_action_name,
			 std::string execution_action_name) :
      BlackBoxPrimitive(planning_action_name, execution_action_name) 
    {pregrasp_map_ = new PreGraspMap();}
    //i really don't like this... need a better way of doing distance!!!!
    double distance_to_nearest_grasp
    (unsigned int objind,
     const ob::State *source, const ob::State *destination) const;
    ~BlackBoxWithPregrasp();
  protected:
    PreGraspMap *pregrasp_map_;
  };
    

}

#endif //pr2_primitives.hh
