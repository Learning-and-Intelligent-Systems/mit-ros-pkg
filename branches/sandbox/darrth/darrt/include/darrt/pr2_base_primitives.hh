#ifndef __DARRT_BASE_PRIMITIVES_HH__
#define __DARRT_BASE_PRIMITIVES_HH__

#include "darrt/collision_aware_types.hh"
#include "darrt/primitive.hh"
#include "darrt/robot_base_space.hh"
#include "darrt/space.hh"
#include "darrt/state_transformer.hh"
#include "darrt/utils.hh"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>

#define BASE_TRIES 8

namespace darrt {
  const double BASE_PENALTY = 2.0;
  const double MIN_BASE_DIST = 0.3;
  const unsigned int NBASE_POSES_TO_TEST = 15;

  class ArmIK;

  class BaseIK {
  public:
    BaseIK(std::string group_name) {group_name_ = group_name; dspace_ = NULL;}
    bool setup(const oc::SpaceInformation *si);
    ob::State *base_state(ob::State *state) const;
    gm::Pose2D base_pose(const ob::State *state) const;
    const ob::State *base_state(const ob::State *state) const;
    void get_shoulder_positions(const ob::State *state,
				std::vector<gm::Point> &shoulders) const;
    gm::Pose2D get_base_pose(const gm::TransformStamped &transform, 
			     const gm::Pose &shoulder_pose) const;
    void get_base_pose_for_wrist(const gm::Pose &wrist_pose, ob::State *state) const;
    void get_base_pose_for_object(const gm::Pose &object_pose, ob::State *state) const;
    gm::Pose2D get_base_pose2d_for_wrist(const gm::Pose &wrist_pose) const;
    gm::Pose2D get_base_pose2d_for_object(const gm::Pose &object_pose) const;

    DARRTStateSpace *dspace() {return dspace_;}
    const DARRTStateSpace *dspace() const {return dspace_;}
    CompoundRobotStateSpace *robot_space() {return robot_space_;}
    const CompoundRobotStateSpace *robot_space() const {return robot_space_;}
    RobotBaseStateSpace *base_space() {return base_space_;}
    const RobotBaseStateSpace *base_space() const {return base_space_;}    
    std::string group_name() const {return group_name_;}
    unsigned int base_index() const {return base_index_;}
    StateTransformer &transformer() {return transformer_;}
    const StateTransformer &transformer() const {return transformer_;}

    unsigned int sampleBaseMove(const Primitive *prim, std::string type,
				const ob::State *source, 
				const ob::State *target, PIList &clist) const;

  protected:
    StateTransformer transformer_;
    std::vector<gm::TransformStamped> shoulder_transforms_;

    gm::Pose2D get_base_pose2d(const gm::Pose &object_pose, double gripper_length) const;
    void get_base_ik(const gm::Pose &object_pose, ob::State *state, double gripper_length) const;
    DARRTStateSpace *dspace_;
    CompoundRobotStateSpace *robot_space_;
    unsigned int base_index_;
    RobotBaseStateSpace *base_space_;
    std::string group_name_;
    const oc::SpaceInformation *si_;

  };
    
  class PR2BaseTransit : public TransitPrimitive {
  public:
    PR2BaseTransit(std::string group_name, std::string name="BaseTransit") : 
      TransitPrimitive(name+"-"+group_name), base_ik_(group_name) {arm_ik_ = NULL;}
    virtual Primitive *copy() const;
    virtual bool setup(const oc::SpaceInformation *si);
    virtual bool useful(const ob::State *source, const ob::State *destination) const;
    virtual unsigned int sampleTo(const ob::State *source, const ob::State *target,
				  PIList &clist) const;
    bool execute(const std::vector<const ob::State *> &path) const;
    const BaseIK &base_ik() const {return base_ik_;}
    BaseIK &base_ik() {return base_ik_;}
  protected:
    BaseIK base_ik_;
    const ArmIK *arm_ik_;
    DARRTStateSpace *dspace_;
  };


  class PR2BaseManipulation : public PR2BaseTransit {
  public:
    PR2BaseManipulation(std::string group_name, std::string name="BaseManipulation") : 
      PR2BaseTransit(group_name, name) {validity_checker_ = NULL;}
    bool setup(const oc::SpaceInformation *si);
    void set_validity_checker(const ob::StateValidityChecker *vc) {validity_checker_ = vc;}
    bool useful(const ob::State *source, const ob::State *destination) const;
    virtual unsigned int sampleTo(const ob::State *source, const ob::State *target,
				  PIList &clist) const;

  protected:
    const ob::StateValidityChecker *validity_checker_;
  };

  //Really should also play with the arms...  oh well
  class PR2Warp : public PR2BaseManipulation {
  public:
    PR2Warp(std::string group_name) : PR2BaseManipulation(group_name, "Warp") {}
    unsigned int sampleTo(const ob::State *source, const ob::State *target, PIList &clist) const;
    //deletes the state when the primitive is deleted
    void add_set_position(std::string name, const ob::State *state) 
    {set_positions_.push_back(std::make_pair(name, state));}
  protected:
    std::vector<std::pair<std::string, const ob::State *> >set_positions_;
    StateTransformer transformer_;
  };

  class PR2BaseTransitInstance : public CollisionAwarePrimitiveInstance {
  public:
    PR2BaseTransitInstance(const Primitive *prim, std::string type, const ob::State *source,
			   const ob::State *destination, std::string group_name);
    virtual PrimitiveInstance *copy() const;
    virtual bool contains(const ob::State *state) const;
    bool propagate(const ob::State *state, const double duration, ob::State *result)
      const;
    ob::State *base_state(ob::State *state) const;
    const ob::State *base_state(const ob::State *state) const;
    void setWarp() {warp_ = true;}
  protected:
    DARRTStateSpace *dspace_;
    CompoundRobotStateSpace *robot_space_;
    unsigned int base_index_;
    RobotBaseStateSpace *base_space_;
    bool warp_;
    std::string group_name_;
  };

  // class PR2BaseManipulationInstance : public PR2BaseTransitInstance {
  // public:
  //   PR2BaseManipulationInstance(const Primitive *prim, const ob::State *source,
  // 				const ob::State *destination) :
  //     PR2BaseTransitInstance(prim, source, destination) {}

  //   PrimitiveInstance *copy() const;
  //   bool contains(const ob::State *state) const;
  // };
}

#endif //pr2_base_primitives.hh
