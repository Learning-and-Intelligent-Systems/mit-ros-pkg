#include "darrt/pr2_base_primitives.hh"
#include "darrt/pr2_arm_primitives.hh"
#include "darrt/object_space.hh"
#include "darrt/state_transformer.hh"
#include "darrt/transform_ros_types.hh"

#include <base_trajectory_action/BaseTrajectoryAction.h>

#include <actionlib/client/simple_action_client.h>

bool darrt::BaseIK::setup(const oc::SpaceInformation *si) {
  si_ = si;
  dspace_ = si->getStateSpace()->as<DARRTStateSpace>();
  robot_space_ = dspace_->robot_state_space()->as<CompoundRobotStateSpace>();
  base_index_ = robot_space_->getSubspaceIndex(group_name_);
  base_space_ = robot_space_->as<RobotBaseStateSpace>(base_index_);

  for (unsigned int i = 0; i < dspace_->primitives().size(); i++) {
    const PR2ArmTransit *arm_transit = dynamic_cast<const PR2ArmTransit *>
      (dspace_->primitives().at(i));
    if (arm_transit) {
      gm::TransformStamped strans = transformer_.get_transform
	(arm_transit->arm_name().substr(0,1)+"_shoulder_lift_link", 
	 transformer_.robot_frame_id(), 
	 *(robot_space_->environment_interface()->kinematicState()));
      strans.transform.rotation.x = 0.0;
      strans.transform.rotation.y = 0.0;
      strans.transform.rotation.z = 0.0;
      strans.transform.rotation.w = 1.0;
      shoulder_transforms_.push_back(strans);
    }
  }
  if (!shoulder_transforms_.size()) {
    ROS_ERROR("Base transfer: no way of moving the arms");
    return false;
  }
  return true;
}

ob::State *darrt::BaseIK::base_state(ob::State *state) const {
  return robot_space_->get_state(base_index_, dspace_->robot_state(state));
}


const ob::State *darrt::BaseIK::base_state(const ob::State *state) const {
  return robot_space_->get_state(base_index_, dspace_->robot_state(state));
}

gm::Pose2D darrt::BaseIK::base_pose(const ob::State *state) const {
  return base_space_->get_pose(base_state(state));
}

void darrt::BaseIK::get_shoulder_positions(const ob::State *state,
					   std::vector<gm::Point> &shoulders) 
  const {
  gm::Pose2D bpose = base_space_->get_pose(base_state(state));
  gm::Point bpt;
  bpt.x = bpose.x;
  bpt.y = bpose.y;
  bpt.z = 0.0;
  for (unsigned int i = 0; i < shoulder_transforms_.size(); i++) {
    shoulders.push_back(transform_point(bpt, shoulder_transforms_[i].transform));
  }
}

gm::Pose2D darrt::BaseIK::get_base_pose(const gm::TransformStamped &transform, 
					const gm::Pose &shoulder_pose) const {
  //the transform is in the frame of the robot
  gm::Pose origin;
  origin.orientation.w = 1.0;
  return pose_to_pose2D(transform_pose(inverse_transform_pose(origin, transform.transform), shoulder_pose));
}

void darrt::BaseIK::get_base_pose_for_wrist(const gm::Pose &wrist_pose, ob::State *state) const {
  get_base_ik(wrist_pose, state, 0);
}

void darrt::BaseIK::get_base_pose_for_object(const gm::Pose &object_pose, ob::State *state) const {
  get_base_ik(object_pose, state, GRIPPER_LENGTH);
}

void darrt::BaseIK::get_base_ik(const gm::Pose &pose, ob::State *state, double gripper_length) const {
  gm::Pose2D bpose = get_base_pose2d(pose, gripper_length);
  base_space_->set_pose(base_state(state), bpose);
}

gm::Pose2D darrt::BaseIK::get_base_pose2d_for_wrist(const gm::Pose &wrist_pose) const {
  return get_base_pose2d(wrist_pose, 0);
}

gm::Pose2D darrt::BaseIK::get_base_pose2d_for_object(const gm::Pose &object_pose) const {
  return get_base_pose2d(object_pose, GRIPPER_LENGTH);
}


gm::Pose2D darrt::BaseIK::get_base_pose2d(const gm::Pose &pose, double gripper_length) const {
  
  //choose a shoulder positions
  double alpha = rand()/((double)RAND_MAX)*2.0*MATH_PI;
  unsigned int sind = rand() % shoulder_transforms_.size();
  double zdiff = fabs(shoulder_transforms_[sind].transform.translation.z -
		      pose.position.z);
  double dist2Dsq = -1;
  double maxlen = ARM_LENGTH+gripper_length;
  double minlen = MIN_BASE_DIST;
  if (zdiff > maxlen) {
    dist2Dsq = MIN_BASE_DIST*MIN_BASE_DIST;
  } else {
    dist2Dsq = rand()/(double)RAND_MAX*(maxlen*maxlen - zdiff*zdiff - minlen*minlen) 
      + minlen*minlen;
    if (dist2Dsq < 0) {
      pause("dist2dsq < 0 (" + makestring(dist2Dsq)+").  maxlen = "+makestring(maxlen)+
	    ", minlen = "+makestring(minlen)+", zdiff = " +makestring(zdiff)+
	    ", shuolder height = " +makestring(shoulder_transforms_[sind].transform.translation.z)+
	    ", object height = "+makestring(pose.position.z));
      dist2Dsq = MIN_BASE_DIST*MIN_BASE_DIST;
    }
  }
  double dist3D = sqrt(dist2Dsq + zdiff*zdiff);
  double dist2D = sqrt(dist2Dsq);
  //ROS_INFO("Alpha = %f, dist3D = %f, dist2D = %f, zdiff = %f, maxlen = %f", 
  //	   alpha, dist3D, dist2D, zdiff, maxlen);
  
  gm::Pose spose;
  spose.position.x = pose.position.x - dist2D*cos(alpha);
  spose.position.y = pose.position.y - dist2D*sin(alpha);
  spose.position.z = shoulder_transforms_[sind].transform.translation.z;
  //right angle to face the object... except this looks more right?
  spose.orientation.z = sin(alpha/2.0);
  spose.orientation.w = cos(alpha/2.0);
    
  gm::Pose2D bpose = get_base_pose(shoulder_transforms_[sind], spose);
  
  if (debug_level >= DDISTANCE) {
    ROS_INFO("Object pose is (%f %f).  Shoulder pose is (%f, %f).  Base pose is (%f, %f, %f).  alpha = %f, trying for dist = %f (3d) %f (2d), got dist %f", pose.position.x, pose.position.y,
	     spose.position.x, spose.position.y, bpose.x, bpose.y, bpose.theta, 
	     alpha, dist3D, dist2D, darrt::distance(spose.position, pose.position));
  }
  return bpose;
}

unsigned int darrt::BaseIK::sampleBaseMove(const Primitive *prim, std::string type,
					   const ob::State *source,
					   const ob::State *target, PIList &clist) const {
  
  PR2BaseTransitInstance *pi = new PR2BaseTransitInstance(prim, type, source, source, group_name());
  dspace_->copyState(pi->destination(), source);

  base_space()->copyState(base_state(pi->destination()), base_state(target));

  //bring along any attached objects
  const RobotBaseStateSpace::BaseState *s = base_space()->base_state
    (base_state(pi->source()));
  RobotBaseStateSpace::BaseState *r = base_space()->base_state
    (base_state(pi->destination()));
  
  gm::Pose start_pose;
  start_pose.position.x = s->getX();
  start_pose.position.y = s->getY();
  gm::Quaternion qo;
  qo.w = 1.0;
  apply_yaw_to_quaternion(s->getYaw(), qo, start_pose.orientation);
  gm::Pose end_pose;
  end_pose.position.x = r->getX();
  end_pose.position.y = r->getY();
  apply_yaw_to_quaternion(r->getYaw(), qo, end_pose.orientation);
  gm::Pose origin;
  origin.orientation.w = 1.0;

  gm::Transform start_to_end_transform = pose_to_transform
    (transform_pose(inverse_transform_pose(origin, start_pose), end_pose));
  gm::Transform end_to_start_transform = pose_to_transform
      (transform_pose(inverse_transform_pose(origin, end_pose), start_pose));
  for (unsigned int i = 0; i < dspace()->object_indexes().size(); i++) {
    unsigned int ind = dspace()->object_indexes().at(i);
    const ObjectStateSpace *ospace = dspace()->getSubspace(ind)->as<ObjectStateSpace>();
    const ObjectStateSpace::StateType *ss = 
      dspace()->get_state(ind, source)->as<ObjectStateSpace::StateType>();      
    gm::Pose opose;
    ospace->object_position(ss, opose);
    if (ss->attach_link.size()) {
      opose = transform_pose(opose, start_to_end_transform);
    }
    ospace->set_object_position(dspace()->get_state(ind, pi->destination()), opose);
  }

  //allow collisions among unattached objects during movement
  an::CollisionOperation op;
  op.operation = op.DISABLE;
  for (unsigned int i = 0; i < dspace()->object_indexes().size(); i++) {
    unsigned int ind = dspace()->object_indexes().at(i);
    const ObjectStateSpace::StateType *os;
    os = dspace()->get_state(ind, source)->as<ObjectStateSpace::StateType>();
    op.object1 = dspace()->getSubspace(ind)->getName();
    if (os->support_surface) {
      op.object2 = os->support_surface->name();
      pi->add_allowed_collision(op);
    } else {
      for (unsigned int j = 0; j < os->touch_links.size(); j++) {
	op.object2 = os->touch_links[j];
	pi->add_allowed_collision(op);
      }
    }
  }
  unsigned int turns = static_cast<unsigned int>
    (base_space()->distance(base_state(pi->destination()), 
				     base_state(pi->source()))/si_->getPropagationStepSize())+1;
  pi->set_turns(turns);
  clist.push_back(pi);
  return turns;
}

darrt::Primitive *darrt::PR2BaseTransit::copy() const {
  PR2BaseTransit *p = new PR2BaseTransit(base_ik_.group_name());
  if (p->is_setup()) {
    p->setup(si_);
  }
  return p;
}

bool darrt::PR2BaseTransit::setup(const oc::SpaceInformation *si) {
  if (!TransitPrimitive::setup(si)) {
    return false;
  }
  if (!base_ik_.setup(si)) {
    return false;
  }
  dspace_ = base_ik_.dspace();
  for (unsigned int i = 0; i < dspace_->primitives().size(); i++) {
    const PR2Arm *prarm = dynamic_cast<const PR2Arm *>(dspace_->primitives().at(i));
    if (prarm) {
      arm_ik_ = prarm->ik_solver();
    }
  }
  return arm_ik_;
}

bool darrt::PR2BaseTransit::useful(const ob::State *source, 
				   const ob::State *destination) const {
  //this is NOT useful when we are holding something
  //in that case we move using RigidTransfer
  //may want to change this later

  const DARRTStateSpace::StateType *s = source->as<DARRTStateSpace::StateType>();
  const DARRTStateSpace::StateType *d = destination->as<DARRTStateSpace::StateType>();
  if (!s->valid() ||
      base_ik_.base_space()->near_states(base_ik_.base_state(source), base_ik_.base_state(destination)) ||
      (s->control() && !s->control()->name().substr(0,4).compare("Push")) ||
      (d->control() && (!d->control()->name().substr(0,4).compare("Push") ||
			!d->control()->name().substr(0,8).compare("Approach")))) { 
    return false;
  }

  if (!base_ik_.dspace()->near_object_states(source, destination)) {
    return false;
  }

  for (size_t i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    const ObjectStateSpace::StateType *os =
      dspace_->get_state(ind, source)->as<ObjectStateSpace::StateType>();
    if (os->attach_link.size()) {
      return false;
    }
  }
  return true;
}

unsigned int darrt::PR2BaseTransit::sampleTo
(const ob::State *source, const ob::State *target, PIList &clist) const {
  return base_ik_.sampleBaseMove(this, name(), source, target, clist);
}

bool darrt::PR2BaseTransit::execute(const std::vector<const ob::State *> &path) const {
  if (!path.size()) {
    ROS_INFO("Empty path, nothing to execute.");
    return true;
  }
  base_trajectory_action::BaseTrajectoryGoal goal;
  for (unsigned int i = 0; i < path.size(); i++) {
    goal.trajectory.push_back(base_ik_.base_space()->get_pose(base_ik_.base_state(path[i])));
  }
  goal.world_frame = base_ik_.transformer().world_frame_id();
  goal.robot_frame = base_ik_.transformer().robot_frame_id();
  goal.linear_velocity = 0.15;
  goal.angular_velocity = MATH_PI/6.0;
  goal.angular_error = 0.01;
  goal.linear_error = 0.03;
  actionlib::SimpleActionClient<base_trajectory_action::BaseTrajectoryAction>
    executor("/base_trajectory_action");
  ROS_INFO("Waiting for base trajectory action");
  executor.waitForServer();
  ROS_INFO("Sending trajectory of %zu points.", goal.trajectory.size());
  executor.sendGoal(goal);
  executor.waitForResult();
  return true;
}

bool darrt::PR2BaseManipulation::setup(const oc::SpaceInformation *si) {
  return PR2BaseTransit::setup(si);
}

bool darrt::PR2BaseManipulation::useful(const ob::State *source, 
				   const ob::State *destination) const {
  //urgh... push is a hack.. but a useful one!
  const DARRTStateSpace::StateType *s = source->as<DARRTStateSpace::StateType>();
  return s->valid() && 
    !base_ik_.dspace()->near_object_states(source, destination) &&
    (!s->control() || s->control()->name().compare("Push"));
}


unsigned int darrt::PR2BaseManipulation::sampleTo
(const ob::State *source, const ob::State *target, PIList &clist) const {
  
  std::vector<unsigned int> objinds;
  for (unsigned int i = 0; i < base_ik_.dspace()->object_indexes().size(); i++) {
    unsigned int ind = base_ik_.dspace()->object_indexes().at(i);
    if (!base_ik_.dspace()->near_object_states(ind, source, target)) {
      objinds.push_back(ind);
    }
  }
  if (!objinds.size()) {
    ROS_ERROR("Base transfer sample to called when no objects move");
    return 0;
  }

  //choose an object at random
  unsigned int r = rand() % objinds.size();
  unsigned int objind = objinds[r];
  const ObjectStateSpace *ospace = base_ik_.dspace()->getSubspace(objind)->as<ObjectStateSpace>();
  //the pose at which we will want to manipulate
  gm::Pose pose;
  //are we holding it already?
  const ObjectStateSpace::StateType *os = base_ik_.dspace()->get_state(objind, source)->
    as<ObjectStateSpace::StateType>();
  if (os->attach_link.size()) {
    ospace->object_position(base_ik_.dspace()->get_state(objind, target), pose);
  } else {
    ospace->object_position(base_ik_.dspace()->get_state(objind, source), pose);
  }

  //ROS_INFO_STREAM("Positioning to manipulate object around\n" << pose);

  //can we reach this pose without moving? if so, sometimes don't move
  //so that we stop sampling
  bool stationary = false;
  std::vector<gm::Point> shoulder_positions;
  base_ik_.get_shoulder_positions(source, shoulder_positions);
  for (unsigned int i = 0; i < shoulder_positions.size(); i++) {
    if (darrt::distance(shoulder_positions[i], pose.position) < ARM_LENGTH+GRIPPER_LENGTH) {
      stationary = true;
      break;
    }
    if (fabs(shoulder_positions[i].z - pose.position.z) >= ARM_LENGTH+GRIPPER_LENGTH &&
	darrt::distance2D(shoulder_positions[i].x, shoulder_positions[i].y, pose.position.x, pose.position.y) <=
	ARM_LENGTH+GRIPPER_LENGTH) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("Actual pose out of reach");
      }
      stationary = true;
      break;
    }
  }
  int move = rand() % 2;
  if (debug_level >= DPROPAGATE) {
    ROS_INFO("Possible to reach object from current position: %d, distance = %f, arm + gripper = %f",
	     stationary, darrt::distance(shoulder_positions[0], pose.position), ARM_LENGTH+GRIPPER_LENGTH);
  }
  if (stationary && !move) {
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Not moving base.");
    }
    return 0;
  }

  bool valid_pose = false;
  unsigned int turns = 0;
  PIList blist;
  unsigned int ntries = 0;
  while (ros::ok() && !valid_pose && ntries < NBASE_POSES_TO_TEST) {
    ob::State *dest = base_ik_.dspace()->allocState();
    base_ik_.get_base_pose_for_object(pose, dest);
    for (unsigned int j = 0; j < blist.size(); j++) {
      delete blist[j];
    }
    blist.clear();
    turns = PR2BaseTransit::sampleTo(source, dest, blist);
    base_ik_.dspace()->freeState(dest);
    if (!blist.size()) {
      continue;
    }
    valid_pose = (!validity_checker_ || validity_checker_->isValid(blist[blist.size()-1]->destination()));
    ntries += 1;
  }
  if (debug_level >= DPROPAGATE) {
    if (ntries >= NBASE_POSES_TO_TEST) {
      ROS_INFO("Impossible to find valid base pose");
    } else if (validity_checker_) {
      ROS_INFO("Returning valid base pose!");
    }
  }
  
  for (unsigned int i = 0; i < blist.size(); i++) {
    clist.push_back(blist[i]);
  }
  return turns;
}


unsigned int darrt::PR2Warp::sampleTo(const ob::State *source, const ob::State *target, PIList &clist) const {
  unsigned int turns = PR2BaseManipulation::sampleTo(source, target, clist);
  if (turns <= 0) {
    return turns;
  }
  //a warp takes 1 turn
  turns = 1;
  clist.back()->set_turns(turns);

  dynamic_cast<PR2BaseTransitInstance *>(clist.back())->setWarp();
  ob::CompoundState *robot_state = base_ik_.dspace()->robot_state(clist.back()->destination())->as<ob::CompoundState>();

  //we allow set positions
  //figure out where attached objects are relative to their fixed links
  std::map<unsigned int, gm::Pose> link_to_object;
  pm::KinematicState kstate = *(base_ik_.robot_space()->environment_interface()->kinematicState());
  base_ik_.robot_space()->convert_ompl_to_kinematic_state(robot_state, kstate);
  for (unsigned int i = 0; i < base_ik_.dspace()->object_indexes().size(); i++) {
    unsigned int ind = base_ik_.dspace()->object_indexes().at(i);
    ObjectStateSpace::StateType *ds = 
      base_ik_.dspace()->get_state(ind, clist.back()->destination())->as<ObjectStateSpace::StateType>();
    if (!ds->attach_link.size()) {
      continue;
    }
    const ObjectStateSpace *ospace = base_ik_.dspace()->getSubspace(ind)->as<ObjectStateSpace>();
    //position of object
    gm::Pose opose;
    ospace->object_position(ds, opose);
    //position of attached link
    gm::TransformStamped attach_link_pose = 
      transformer_.get_transform(ds->attach_link, ospace->object().header.frame_id, kstate);
    //position of the object in the attach link frame
    link_to_object[ind] = inverse_transform_pose(opose, attach_link_pose.transform);
  }

  //set positions
  for (unsigned int i = 0; i < set_positions_.size(); i++) {
    unsigned int ind = base_ik_.robot_space()->getSubspaceIndex(set_positions_[i].first);
    base_ik_.robot_space()->getSubspace(ind)->copyState(robot_state->components[ind], set_positions_[i].second);
  }

  //update attached object positions
  base_ik_.robot_space()->convert_ompl_to_kinematic_state(robot_state, kstate);
  for (std::map<unsigned int, gm::Pose>::iterator it = link_to_object.begin(); 
       it != link_to_object.end(); it++) {
    unsigned int ind = it->first;
    const ObjectStateSpace *ospace = base_ik_.dspace()->getSubspace(ind)->as<ObjectStateSpace>();
    ObjectStateSpace::StateType *ds = base_ik_.dspace()->get_state(ind, clist.back()->destination())->
      as<ObjectStateSpace::StateType>();
    gm::TransformStamped attach_link_pose = transformer_.get_transform(ds->attach_link, 
								       transformer_.world_frame_id(), kstate);
    ospace->set_object_position(ds, transform_pose(it->second, attach_link_pose.transform));
  }
  return turns;
}


// double darrt::PR2BaseManipulation::distance_to_nearest_grasp
// (unsigned int objind, const ob::State *source, const ob::State *target) const {
//   //use one of the other primitives with real pre-grasp poses to determine distance
//   return MATH_INF;

//   //if the object is in reach, return infinity
//   //if not, return the distance the base must travel multiplied by a penalty
//   //since we would rather not move the base any more than we have to
  
//   const ObjectStateSpace *ospace = dspace_->getSubspace(objind)->as<ObjectStateSpace>();
//   //the pose at which we will want to manipulate
//   gm::Pose pose;
//   //are we holding it already?
//   const ObjectStateSpace::StateType *os = dspace_->get_state(objind, source)->
//     as<ObjectStateSpace::StateType>();
//   if (os->attach_link.size()) {
//     ospace->object_position(target, pose);
//   } else {
//     ospace->object_position(source, pose);
//   }

//   std::vector<gm::Point> shoulder_positions;
//   get_shoulder_positions(source, shoulder_positions);
//   double mindist = MATH_INF;
//   gm::Pose bpose = base_space_->get_pose(base_state(source));
//   for (unsigned int i = 0; i < shoulder_positions.size(); i++) {
//     if (darrt::distance(shoulder_positions[i], pose.position) < ARM_LENGTH) {
//       //can get here without moving the base
//       //use the distance from another primitive
//       return MATH_INF;
//     }
//     double dist = darrt::distance(pose.position, bpose.position);
//     if (dist < mindist) {
//       mindist = dist;
//     }
//   }
//   return mindist*BASE_PENALTY;
  
// }


darrt::PR2BaseTransitInstance::PR2BaseTransitInstance
(const Primitive *prim, std::string type, const ob::State *source, 
 const ob::State *destination, std::string group_name) :
  CollisionAwarePrimitiveInstance(prim, type, source, destination) {
  
  dspace_ = prim_->space_information()->getStateSpace()->as<DARRTStateSpace>();
  robot_space_ = dspace_->robot_state_space()->as<CompoundRobotStateSpace>();
  group_name_ = group_name;
  //std::string group_name;
  //const PR2BaseTransit *bprim = dynamic_cast<const PR2BaseTransit *>(prim_);
  //group_name = bprim->group_name();
  base_index_ = robot_space_->getSubspaceIndex(group_name);
  base_space_ = robot_space_->as<RobotBaseStateSpace>(base_index_);
  warp_ = false;
}

darrt::PrimitiveInstance *darrt::PR2BaseTransitInstance::copy() const {
  PR2BaseTransitInstance *pi = new PR2BaseTransitInstance(prim_, type_, source_, destination_, 
							  group_name_);
  pi->set_allowed_collisions(allowed_collisions);
  pi->set_turns(turns_);
  if (warp_) {
    pi->setWarp();
  }
  return pi;
}

ob::State *darrt::PR2BaseTransitInstance::base_state(ob::State *state) const {
  return robot_space_->get_state(base_index_, dspace_->robot_state(state));
}


const ob::State *darrt::PR2BaseTransitInstance::base_state(const ob::State *state) const {
  return robot_space_->get_state(base_index_, dspace_->robot_state(state));
}


bool darrt::PR2BaseTransitInstance::contains(const ob::State *state) const {
  if (debug_level == DBETWEEN) {
    ROS_INFO("PR2BaseTransit: Checking if %s is between %s and %s",
	     dspace_->state_string(state).c_str(), dspace_->state_string(source_).c_str(),
	     dspace_->state_string(destination_).c_str());
  }
  // if (!dspace_->near_object_states(source_, state)) {
  //   if (debug_level == DBETWEEN) {
  //     ROS_INFO("Base transit does not contain because object states are not near.");
  //   }
  //   return false;
  // }

  for (unsigned int i = 0; i < robot_space_->getSubspaceCount(); i++) {
    if (i == base_index_) {
      continue;
    }
    if (!robot_space_->getSubspace(i)->as<RobotStateSpace>()->near_states
	(robot_space_->get_state(i, dspace_->robot_state(state)),
	 robot_space_->get_state(i, dspace_->robot_state(source_)))) {
      if (debug_level == DBETWEEN) {
	ROS_INFO("Base transit does not contain because %s states are not near.",
		 robot_space_->getSubspace(i)->getName().c_str());
      }
      return false;
    }
  }
  double f = base_space_->between(base_state(state),
				  base_state(source_),
				  base_state(destination_));
  if (debug_level == DBETWEEN) {
    ROS_INFO("f = %f", f);
  }
  return ((f >= 0 && f < 0.999999) || (f > 1.1));
}

bool darrt::PR2BaseTransitInstance::propagate(const ob::State *state,
					      double duration,
					      ob::State *result) const {
  if (warp_) {
    if (duration < 0) {
      dspace_->copyState(result, source_);
    } else {
      dspace_->copyState(result, destination_);
    }
    result->as<DARRTStateSpace::StateType>()->set_allowed_collisions(allowed_collisions);
    result->as<DARRTStateSpace::StateType>()->set_control(prim_, type_);
    if (debug_level >= DPROPAGATE) {
      ROS_INFO("Moving to warp along %s gives %s", this->str().c_str(), dspace_->state_string(result).c_str());
    }
    return true;
  }

  double d = base_space_->distance(base_state(destination_),
				   base_state(source_));
  double ds = base_space_->distance(base_state(state),
				    base_state(source_));
  double newf;
  if (d < EPSILON) {
    newf = 1.0;
  } else {
    newf = (ds + duration)/d;
  }
  if (newf >= 1.0) {
    newf = 1.0;
  }
  if (newf < 0) {
    newf = 0.0;
  }

  dspace_->copyState(result, state);
  base_space_->interpolate(base_state(source_),
			   base_state(destination_), newf,
			   base_state(result));
  //bring along any objects
  const RobotBaseStateSpace::BaseState *s = base_space_->base_state
    (base_state(state));
  RobotBaseStateSpace::BaseState *r = base_space_->base_state
    (base_state(result));
  
  gm::Pose start_pose;
  start_pose.position.x = s->getX();
  start_pose.position.y = s->getY();
  gm::Quaternion qo;
  qo.w = 1.0;
  apply_yaw_to_quaternion(s->getYaw(), qo, start_pose.orientation);
  gm::Pose end_pose;
  end_pose.position.x = r->getX();
  end_pose.position.y = r->getY();
  apply_yaw_to_quaternion(r->getYaw(), qo, end_pose.orientation);
  gm::Pose origin;
  origin.orientation.w = 1.0;

  gm::Transform transform = pose_to_transform
    (transform_pose(inverse_transform_pose(origin, start_pose), end_pose));
  for (unsigned int i = 0; i < dspace_->object_indexes().size(); i++) {
    unsigned int ind = dspace_->object_indexes().at(i);
    ObjectStateSpace::StateType *rs = 
      dspace_->get_state(ind, result)->as<ObjectStateSpace::StateType>();
    if (!rs->attach_link.size()) {
      continue;
    }
    const ObjectStateSpace *ospace = dspace_->getSubspace(ind)->as<ObjectStateSpace>();
    gm::Pose opose;
    ospace->object_position(rs, opose);
    ospace->set_object_position(rs, transform_pose(opose, transform));
  }
  result->as<DARRTStateSpace::StateType>()->set_allowed_collisions(allowed_collisions);
  result->as<DARRTStateSpace::StateType>()->set_control(prim_, type_);
  if (debug_level >= DPROPAGATE) {
    ROS_INFO("Moving to fraction %f along %s gives %s", newf,
	     this->str().c_str(), dspace_->state_string(result).c_str());
  }
  return true;
}
