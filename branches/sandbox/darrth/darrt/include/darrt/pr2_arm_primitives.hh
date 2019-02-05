#ifndef __DARRT_PR2_ARM_PRIMITIVES__HH__
#define __DARRT_PR2_ARM_PRIMITIVES__HH__

#include <map>

#include <geometry_msgs/Vector3.h>

#include "darrt/chain_space.hh"
#include "darrt/collision_aware_types.hh"
#include "darrt/pr2_base_primitives.hh"
#include "darrt/primitive.hh"
#include "darrt/space.hh"
#include "darrt/state_transformer.hh"
#include "darrt/utils.hh"

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

#define GRASP_MEMO_SIZE 0
#define NEAR_DIST_EPS 0.01
#define NEAR_ANGLE_EPS 0.1
#define BLOCK_OFFSET 0.02

namespace an=arm_navigation_msgs;
namespace gm=geometry_msgs;

namespace darrt {

  const std::string TABLE_PLACEHOLDER_NAME = "table_place_holder_name";
  const double MAX_APPROACH_DISTANCE = 0.15;
  const double MIN_APPROACH_DISTANCE = 0.05;
  const double GRASP_OFFSET = 0.03;
  const double GRIPPER_LENGTH = 0.20; //changed for the right arm
  const double PUSH_INTO_TABLE = 0.03;
  const double PUSH_OFFSET = 0.008;
  const double PUSH_TABLE_OFFSET = 0.03;
  const double TABLE_CONTAINS = 0.05;
  const double ARM_LENGTH = 0.75;

  class ArmIK {
  public:
    ArmIK(std::string arm_name) {arm_name_ = arm_name; dspace_ = NULL;} 
    bool setup(DARRTStateSpace *dspace);
    
    ob::State *arm_state(ob::State *full_state) const;
    
    const ob::State *arm_state(const ob::State *full_state) const;

    RobotChainStateSpace *arm_space() {return arm_space_;}
    
    const RobotChainStateSpace *arm_space() const {return arm_space_;}

    gm::PoseStamped propagate_grasp
    (const ob::State *destination, const ob::State *source, 
     unsigned int objind) const;

    ob::State *propagate_grasp
    (const ob::State *destination, const ob::State *source, 
     unsigned int objind, const ob::State *seed) const;
    
    bool propagate_grasp
    (const ob::State *destination, const ob::State *source, 
     unsigned int objind, ob::State *result, const ob::State *seed) const;

    bool feasible
    (const ob::State *source, double distance, const gm::Vector3 &direction, 
     const ob::State *seed=NULL) const;

    bool propagate_interpolated_ik
    (const ob::State *source, const ob::State *destination, double fraction,
     ob::State *result, const ob::State *seed=NULL) const;

    bool propagate_interpolated_ik
    (const ob::State *source, const ob::State *destination, const gm::Vector3 &trans, 
     double fraction, ob::State *result, const ob::State *seed=NULL) const;

    bool propagate_interpolated_ik
    (const ob::State *source, const gm::Transform &trans, double fraction, 
     ob::State *result, const ob::State *seed) const;


    gm::Vector3Stamped translation(const ob::State *source,
				   const ob::State *destination) const;

    gm::Transform wrist_transform(const ob::State *source, 
				  const ob::State *destination) const;
    gm::Pose update_attached_object_position(unsigned int ind, const ob::State *source,
					     const ob::State *destination) const;
    void update_attached_object_positions(const ob::State *source,
					  ob::State *result) const;

    std::string arm_name() {return arm_name_;}

    gm::PoseStamped transform_pose_stamped
    (std::string new_frame, const gm::PoseStamped &ps, const ob::State *state) const;

    gm::Vector3Stamped transform_vector_stamped
    (std::string new_frame, const gm::Vector3Stamped &v, const ob::State *state) const;

    const StateTransformer &transformer() const {return transformer_;}

  protected:
    DARRTStateSpace *dspace_;
    CompoundRobotStateSpace *robot_space_;
    unsigned int arm_index_;
    RobotChainStateSpace *arm_space_;
    std::string arm_name_;
    StateTransformer transformer_;
    gm::TransformStamped world_to_base_, world_to_ik_, base_to_ik_;
  };

  class PR2Arm {
  public:
    PR2Arm(std::string arm_name) : ik_solver_(arm_name) {arm_name_ = arm_name;}
    virtual bool setup_arm(const oc::SpaceInformation *si);
    virtual bool execute_path(const std::vector<const ob::State *> &path) const;
    bool open_gripper() const;
    bool close_gripper() const;
    bool move_gripper(double pos) const;

    bool convert_path_to_joint_trajectory
    (const std::vector<const ob::State *> &path,
     trajectory_msgs::JointTrajectory &traj) const;
    bool unnormalize_trajectory(trajectory_msgs::JointTrajectory &traj) const;
    ob::State *arm_state(ob::State *full_state) const;
    const ob::State *arm_state(const ob::State *full_state) const;
    std::string arm_name() const {return arm_name_;}
    const ArmIK *ik_solver() const {return &ik_solver_;}
    unsigned int sampleArmMove(const Primitive *prim, std::string type,
			       const ob::State *source, 
			       const ob::State *target, PIList &clist) const;
    unsigned int sampleLineArmMove
    (const Primitive *prim, std::string type,
     const ob::State *source, const gm::Transform &trans,
     double step_size, const an::OrderedCollisionOperations &ordered_collisions,
     PIList &clist) const;
    unsigned int sampleLineArmMove
    (const Primitive *prim, std::string type,
     const ob::State *source, 
     const ob::State *destination, double desired_distance, 
     double min_distance, const gm::Vector3 &direction, 
     double step_size,
     const an::OrderedCollisionOperations &ordered_collisions, 
     PIList &clist, bool reverse=false) const;
    //doesn't exactly belong in this class, but it's easy
    void collisionOperations(const ob::State *state, an::OrderedCollisionOperations &ops) const;

  protected:
    DARRTStateSpace *dspace_;
    CompoundRobotStateSpace *robot_space_;
    RobotChainStateSpace *arm_space_;
    std::string arm_name_;
    unsigned int arm_index_;
    double step_size_;
    ArmIK ik_solver_;
  };

  class PR2ArmPrimitiveInstance : public CollisionAwarePrimitiveInstance {
  public:
    PR2ArmPrimitiveInstance(const Primitive *prim, std::string type, const ob::State *source,
			    const ob::State *destination);
  protected:
    DARRTStateSpace *dspace_;
    CompoundRobotStateSpace *robot_space_;
    RobotChainStateSpace *arm_space_;
    std::string arm_name_;
    unsigned int arm_index_;
    const PR2Arm *arm_;
  };

  class PR2ArmTransitInstance : public PR2ArmPrimitiveInstance {
  public:
    PR2ArmTransitInstance(const Primitive *prim,
			  std::string type,
			  const ob::State *source,
			  const ob::State *destination) :
      PR2ArmPrimitiveInstance(prim, type, source, destination) {}
    virtual PrimitiveInstance *copy() const;
    virtual bool contains(const ob::State *state) const;
    virtual bool propagate(const ob::State *state,
  			   const double duration,
  			   ob::State *result) const;
  protected:
    virtual double fraction(const ob::State *state) const;
  };

  class PR2ArmTransit : public TransitPrimitive, public PR2Arm {
  public:
    PR2ArmTransit(std::string arm_name, std::string type="ArmTransit") : 
      TransitPrimitive(type+"-"+arm_name), PR2Arm(arm_name) 
    {type_ = type;}
    virtual Primitive *copy() const;
    virtual bool setup(const oc::SpaceInformation *si);
    virtual bool useful(const ob::State *source, 
  			const ob::State *destination) const;
    virtual bool active(const ob::State *source, 
  			const ob::State *destination) const;

    virtual unsigned int sampleTo(const ob::State *source,
  				  const ob::State *target,
				  PIList &clist) const;
    virtual bool execute(const std::vector<const ob::State *> &path) const
    {return execute_path(path);}
    std::string type() {return type_;}
    void projectionFunction(const ob::State *from, ob::State *sample) const;
    virtual DARRTProjectionFunction getProjectionFunction() const;

  protected:
    std::string type_;
  };

  class PR2LineArmTransit : public PR2ArmTransit {
  public:
    PR2LineArmTransit(std::string name, std::string arm_name) : 
      PR2ArmTransit(arm_name, name) {}
    
    virtual bool setup(const oc::SpaceInformation *si); 
    virtual unsigned int sampleTo
    (const ob::State *source, const ob::State *destination, 
     double desired_distance, double min_distance, const gm::Vector3 &direction, 
     const an::OrderedCollisionOperations &ops, PIList &clist, bool reverse=false) const;

    virtual unsigned int sampleTo
    (const ob::State *source, const gm::Transform &trans,
     const an::OrderedCollisionOperations &ordered_collisions,
     PIList &clist) const;

    virtual bool execute(const std::vector<const ob::State *> &path) const;
  };

  class Approach : public PR2LineArmTransit {
  public:
    Approach(std::string arm_name,
	     double desired_distance = MAX_APPROACH_DISTANCE,
	     double min_distance=0);
    virtual Primitive *copy() const;
    bool setup(const oc::SpaceInformation *si);
    bool useful(const ob::State *source, const ob::State *destination) const;
    unsigned int sampleTo(const ob::State *source, const ob::State *target,
			  PIList &clist) const;
    double desired_distance() const {return desired_distance_;}
    double min_distance() const {return min_distance_;}
  protected:
    double desired_distance_, min_distance_;
    const PR2ArmTransit *arm_transit_;
    const PR2BaseTransit *base_transit_;
  };



  class Retreat : public PR2LineArmTransit {
  public:
    Retreat(std::string arm_name,
	    double desired_distance=MAX_APPROACH_DISTANCE,
	    double min_distance=0) :
      PR2LineArmTransit("Retreat", arm_name), 
      desired_distance_(desired_distance), min_distance_(min_distance) {}
    virtual Primitive *copy() const;
    bool useful(const ob::State *source, const ob::State *destination) const;
    unsigned int sampleTo(const ob::State *source, const ob::State *target,
			  PIList &clist) const;
    double desired_distance() const {return desired_distance_;}
    double min_distance() const {return min_distance_;}
  protected:
    double desired_distance_, min_distance_;
  };
    
  class UseSpatula;

  class RigidTransfer : public TransferPrimitive, public PR2Arm {
  public:
    RigidTransfer(std::string arm_name, std::string objid) : 
      TransferPrimitive("RigidTransfer-"+arm_name+"-"+objid), PR2Arm(arm_name), objid_(objid),
      uspat_(NULL) {}

    virtual Primitive *copy() const;
    virtual bool setup(const oc::SpaceInformation *si);
    virtual bool useful(const ob::State *source, 
			const ob::State *destination) const;
    virtual bool active(const ob::State *source, 
			const ob::State *destination) const;
    //returns false if there is a collision or constraint violation
    virtual unsigned int sampleTo(const ob::State *source,
				  const ob::State *target,
				  PIList &clist) const;
    virtual bool execute(const std::vector<const ob::State *> &path) const
    {return execute_path(path);}
    const BaseIK *base_ik() const {return base_ik_;}

    unsigned int sampleMove(const ob::State *source, const ob::State *result, PIList &clist) const;
    std::string objid() const {return objid_;}
    void projectionFunction(const ob::State *from, ob::State *sample) const;
    virtual DARRTProjectionFunction getProjectionFunction() const;

  protected:
    const BaseIK *base_ik_;
    std::string objid_, base_transit_name_;
    unsigned int objind_;
    const UseSpatula *uspat_;
  };

  class DiscreteGraspsPrimitive : public TransferPrimitive, public PR2Arm {
  public:
    class Grasp {
    public:
      //gives the location of the object in the end effector's frame 
      gm::Transform grasp;
      std::vector<std::string> touch_links;
      double min_distance, desired_distance;
      Grasp() 
      {grasp.rotation.w = 1.0; min_distance=0.0; desired_distance=0.0;}
      Grasp(const gm::Transform g, const std::vector<std::string> &tl, 
	    double md, double dd)
      {grasp = g; touch_links = tl; min_distance=md; desired_distance=dd;}
      virtual Grasp *copy() const 
      {return new Grasp(grasp, touch_links, min_distance, desired_distance);}
      virtual ~Grasp() {}
    };

    class GraspWithIK {
    public:
      ob::State *grasp_ik;
      Grasp *grasp;
      //note: always in the robot's frame
      gm::Vector3 direction_to_pregrasp;
      GraspWithIK() 
      {grasp_ik = NULL; grasp = NULL;}
      GraspWithIK(const GraspWithIK &gs) 
      {grasp_ik = gs.grasp_ik; 
	direction_to_pregrasp = gs.direction_to_pregrasp; 
	grasp = gs.grasp->copy();}
      ~GraspWithIK() {if (grasp) {delete grasp;}}
    };

    typedef std::vector<Grasp *> GraspList;

    DiscreteGraspsPrimitive(std::string name, std::string arm_name, std::string objid);
    virtual bool setup(const oc::SpaceInformation *si);
    virtual bool useful(const ob::State *source, const ob::State *target) 
      const;
    virtual bool active(const ob::State *source, const ob::State *target) const;
    virtual unsigned int sampleTo(const ob::State *source,
				  const ob::State *target,
				  PIList &clist) const;
    bool grasp_ik(const ob::State *source, 
		  const ob::State *destination,
		  std::vector<GraspWithIK> &grasp_sols) const;
    std::string objid() const {return objid_;}
  protected:
    std::string objid_;
    unsigned int objind_;
    const PR2ArmTransit *transit_;
    const PR2BaseTransit *base_transit_;
    const Retreat *retreat_;
    //this should be a pointer now that approach is its own primitive
    const Approach *approach_; 
    double max_retreat_dist_;

    virtual bool manipulableObject(const ob::State *source,
				   const ob::State *destination) const;

    virtual void extraGraspPoses(const GraspWithIK &grasp,
				 const gm::PoseStamped &grasp_pose,
				 const ob::State *source, const ob::State *target,
				 std::vector<gm::PoseStamped> &extra_poses) const;

    bool has_grasp(const ob::State *source, 
		   const ob::State *destination, bool requires_pregrasp) 
      const;
    

    bool feasible_grasp(const ob::State *source,
			const ob::State *destination, const Grasp &grasp,
			bool requires_pregrasp) const;

    bool grasp_ik(const ob::State *source, 
		  const ob::State *destination, const Grasp &grasp,
		  GraspWithIK &grasp_sol) const;

    gm::Pose get_grasp_pose(const gm::Transform &grasp, const gm::Pose &opose) 
      const;

    virtual unsigned int sample_from_grasp
    (const ob::State *source, const ob::State *target,
     const GraspWithIK &grasp, PIList &clist) const=0;
    virtual bool grasps
    (const ob::State *source, const ob::State *destination, 
     GraspList &grasps) const=0;

    virtual Grasp *best_grasp(const ob::State *source, 
			      const ob::State *destination) 
      const=0;
  };
    
  class Push : public DiscreteGraspsPrimitive {
  public:
    Push(std::string arm_name, std::string objid, std::vector<std::string> pushing_links,
	 double table_offset=0.0);
    
    virtual Primitive *copy() const;
    bool setup(const oc::SpaceInformation *si);
    bool execute(const std::vector<const ob::State *> &path) const
    {return execute_path(path);}
    void projectionFunction(const ob::State *from, ob::State *sample) const;
    virtual DARRTProjectionFunction getProjectionFunction() const;

  protected:
    std::vector<std::string> pushing_links_;
    double table_offset_;

    bool manipulableObject(const ob::State *source,
			    const ob::State *destination) const;
    void extraGraspPoses(const GraspWithIK &grasp,
			 const gm::PoseStamped &grasp_pose,
			 const ob::State *source, const ob::State *target,
			 std::vector<gm::PoseStamped> &extra_poses) const;

    bool grasps(const ob::State *source,
		const ob::State *destination, 
		GraspList &grasps) const;
    unsigned int sample_from_grasp
    (const ob::State *source, const ob::State *target,
     const GraspWithIK &grasp, PIList &clist) const;
    Grasp *best_grasp(const ob::State *source, 
		      const ob::State *destination) const;
  };  


  class PickupPrimitive : public DiscreteGraspsPrimitive {
  public:
    class RigidGrasp : public Grasp {
    public:
      double min_distance_from_surface;
      std::string attach_link;
      RigidGrasp() : Grasp() 
      {attach_link = ""; min_distance_from_surface = -1;}
      RigidGrasp(const RigidGrasp &rg) : Grasp(rg) 
      {attach_link = rg.attach_link; 
	min_distance_from_surface = rg.min_distance_from_surface;}
      RigidGrasp(const gm::Transform g, const std::vector<std::string> &tl, 
		 std::string al, double md, double dd) :
	Grasp(g, tl, md, dd) {attach_link = al; min_distance_from_surface = -1;}
      RigidGrasp(const gm::Transform g, const std::vector<std::string> &tl, 
		 std::string al, double md, double dd, double mds) :
	Grasp(g, tl, md, dd) 
      {attach_link = al; min_distance_from_surface = mds;}
      Grasp *copy() const {return new RigidGrasp(*this);}
    };

    typedef std::vector<RigidGrasp> RigidGraspList;
    
    //note: if you are doing anything that plans a pickup
    //and then something else, you don't need a minimum distance
    PickupPrimitive(std::string arm_name, std::string objid, const RigidGraspList &grasps, const gm::Vector3 &lift,
		    double min_lift_distance=0.0, double desired_lift_distance=MAX_APPROACH_DISTANCE);
    Primitive *copy() const;
    bool setup(const oc::SpaceInformation *si);
    bool execute(const std::vector<const ob::State *> &path) const;
    bool becomesGoal() const {return false;}

    const RigidGraspList &getGrasps() const {return grasps_;}
    const gm::Vector3 &getLift() const {return lift_;}
    double getMinLiftDistance() const {return min_lift_distance_;}
    double getDesiredLiftDistance() const {return desired_lift_distance_;}
  protected:

    void extraGraspPoses(const GraspWithIK &grasp,
			 const gm::PoseStamped &grasp_pose,
			 const ob::State *source, const ob::State *target,
			 std::vector<gm::PoseStamped> &extra_poses) const;

    bool grasps(const ob::State *source, const ob::State *destination,
		GraspList &grasps) const;
    bool grasps
    (const ob::State *source, const ob::State *destination,
     GraspList &grasps, bool return_first_found) const;
    unsigned int sample_from_grasp
    (const ob::State *source, const ob::State *target,
     const GraspWithIK &grasp, PIList &clist) const;
    Grasp *best_grasp(const ob::State *source, 
		      const ob::State *destination) const;

    RigidGraspList grasps_;
    gm::Vector3 lift_;
    double min_lift_distance_, desired_lift_distance_;
  };



  class PlacePrimitive : public TransferPrimitive, public PR2Arm {
  public:
    class Place {
    public:
      ob::State *place;
      unsigned int objind, tableind;
      gm::Vector3 direction_to_preplace;
      Place(ob::State *p, const gm::Vector3 &dp, unsigned int oi,
	    unsigned int ti) 
      {place=p; direction_to_preplace = dp; objind=oi; tableind=ti;}
    };
    PlacePrimitive(std::string arm_name, const gm::Vector3 &approach_direction,
		   double min_distance=MIN_APPROACH_DISTANCE, 
		   double desired_distance=MAX_APPROACH_DISTANCE);

    bool setup(const oc::SpaceInformation *si);
    bool useful(const ob::State *source, const ob::State *destination) const;
    unsigned int sampleTo(const ob::State *source, const ob::State *target,
			  PIList &clist) const;
    bool execute(const std::vector<const ob::State *> &path) const;\
    bool becomesGoal() const {return false;}
    ~PlacePrimitive() {}
  protected:
    gm::Vector3 approach_direction_;
    double min_distance_, desired_distance_;
    const RigidTransfer *rigid_transfer_;
    
    bool placedObjects(const ob::State *source, const ob::State *destination,
		       std::vector< pair<unsigned int, unsigned int> > 
		       &placed_objects) const;
    Place *get_place(const ob::State *source, 
		     const ob::State *destination) const;
    gm::Pose get_object_pose_on_table
    (const an::CollisionObject &obj, const gm::Point &point_on_table,
     const gm::Quaternion &object_orientation) const;
  };

  class StatePathInstance : public CollisionAwarePrimitiveInstance {
  public:
    StatePathInstance(const Primitive *prim, std::string type, 
		      const std::vector<ob::State *> &state_path);
    virtual PrimitiveInstance *copy() const;
    virtual bool contains(const ob::State *state) const;
    virtual bool propagate(const ob::State *state,
  			   const double duration,
  			   ob::State *result) const;
    const std::vector<ob::State *> &path() const {return path_;}
    virtual ~StatePathInstance();
  protected:
    std::vector<ob::State *> path_;
    int *curr_state_; //pointer to get around constness issues... wince
    const DARRTStateSpace *dspace_;
    bool allow_path_sampling_;
  };

  class UseSpatula : public TransferPrimitive, public PR2Arm {
  public:
    UseSpatula(std::string arm_name, std::string spatid,
	       std::string objid, std::string blockid,
	       const PickupPrimitive::RigidGraspList &spatula_grasps,
	       const gm::Vector3 &spatula_lift,
	       double horizontal_approach_distance=MAX_APPROACH_DISTANCE,
	       double vertical_approach_distance=MAX_APPROACH_DISTANCE,
	       double lift_distance=MAX_APPROACH_DISTANCE,
	       double spatula_min_lift_distance=0,
	       double spatula_desired_lift_distance=MAX_APPROACH_DISTANCE);
    Primitive *copy() const;
    bool setup(const oc::SpaceInformation *si);
    bool useful(const ob::State *source,
		const ob::State *destination) const;
    unsigned int sampleTo(const ob::State *source,
			  const ob::State *target,
			  PIList &clist) const;
    bool becomesGoal() const {return false;}
    void chooseToolUse(const ob::State *source,
		       const ob::State *target,
		       gm::Pose &object_pose,
		       gm::Pose &spatula_pose,
		       gm::Pose &flat_spatula_pose,
		       gm::Vector3Stamped &horizontal_approach_direction) const;
    unsigned int objind() const {return objind_;}
    const PickupPrimitive &pickup() const {return pickup_;}
  protected:
    int objectBlockSide(const ob::State *state) const;
    std::pair<unsigned int, unsigned int> blockOrientation(const ob::State *state) const;

    std::string spatid_, blockid_, objid_, arm_transit_name_, approach_name_;
    unsigned int spatind_, blockind_, objind_;
    double horizontal_approach_distance_, vertical_approach_distance_, 
      lift_distance_;
    const Push *push_;
    const RigidTransfer *rigid_transfer_;
    const PR2BaseTransit *base_transit_;
    const Retreat *retreat_;
    PickupPrimitive pickup_;
    const an::CollisionObject *obj_, *block_, *spatula_;
  };

  class SpatulaTransfer : public TransferPrimitive, public PR2Arm {
  public:
    SpatulaTransfer(std::string arm_name, std::string spatid,
		    std::string objid);
    Primitive *copy() const;
    bool setup(const oc::SpaceInformation *si);
    bool useful(const ob::State *source,
		const ob::State *destination) const;
    unsigned int sampleTo(const ob::State *source,
			  const ob::State *target,
			  PIList &clist) const;
    void projectionFunction(const ob::State *from, ob::State *sample) const;
    virtual DARRTProjectionFunction getProjectionFunction() const;


  protected:
    std::string spatid_, objid_, approach_name_, arm_transit_name_;
    unsigned int spatind_, blockind_, objind_;
    const PR2BaseTransit *base_transit_;
    const Approach *approach_;
    const UseSpatula *uspat_;
    const an::CollisionObject *obj_, *spatula_;
  };


}

#endif //pr2_primitives.hh
