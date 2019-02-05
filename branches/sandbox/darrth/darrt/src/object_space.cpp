#include "darrt/object_space.hh"
#include "darrt/transform_ros_types.hh"
#include <planning_environment/models/model_utils.h>

darrt::ObjectStateSpace::ObjectStateSpace(const DARRTObject &obj) : 
  ob::SE3StateSpace(),
  Displayable("object_state_markers"), object_(obj) {
  if (!object_.shapes.size()) {
    throw "Cannot use collision objects with no shapes";
  }
 
  setName(object_.id);

}

ob::State *darrt::ObjectStateSpace::allocState() const {
  StateType *state = new StateType();
  allocStateComponents(state);
  return state;
}

void darrt::ObjectStateSpace::copyState
(ob::State *destination, const ob::State *source) const {
  SE3StateSpace::copyState(destination, source);
  StateType *d = destination->as<StateType>();
  const StateType *s = source->as<StateType>();
  d->set_allowed_collisions(s->allowed_collisions);
  d->set_control(s->control(), s->type());  
  d->support_surface = s->support_surface;
  d->attach_link = s->attach_link;
  d->touch_links = s->touch_links;
}


visualization_msgs::MarkerArray darrt::ObjectStateSpace::displayable_state
(const ob::State *state, std::string ns, int id, double scale,
 ColorPalate p, double alpha, bool minimal) const {

  visualization_msgs::MarkerArray marray;
  visualization_msgs::Marker marker;
  marker.header = object_.header;
  marker.id = id;
  Displayable::interpret_palate(p, marker.color.r, marker.color.g, marker.color.b);
  marker.color.a = alpha;

  //make a marker corresponding to this object
  marker.ns = ns;
  marker.action = marker.ADD;
  gm::Pose trans;
  object_position(state, trans);
  for (unsigned int i = 0; i < object_.shapes.size(); i++) {
    marker.pose = transform_pose(object_.poses[i], trans);
    planning_environment::setMarkerShapeFromShape(object_.shapes[i], marker);
    marray.markers.push_back(marker);
    marker.id++;
  }
  return marray;
}

double darrt::ObjectStateSpace::distance(const ob::State *source, 
					 const ob::State *destination) const {
  gm::Pose p1, p2;
  object_position(source, p1);
  object_position(destination, p2);
  return object_.distance(p1, p2);
}


bool darrt::ObjectStateSpace::near_states
(const ob::State *s1, const ob::State *s2, double deps, double aeps) const {
  gm::Pose p1, p2;
  object_position(s1, p1);
  object_position(s2, p2);
  return object_.nearPoses(p1, p2);

  // const ob::CompoundState *c1 = s1->as<ob::CompoundState>();  
  // const ob::CompoundState *c2 = s2->as<ob::CompoundState>();
  
  // return (getSubspace(0)->distance(c1->components[0], c2->components[0]) 
  // 	  < deps &&
  // 	  getSubspace(1)->distance(c1->components[1], c2->components[1])
  // 	  < aeps);
}

double darrt::ObjectStateSpace::between
(const ob::State *state, const ob::State *source, 
 const ob::State *destination, double deps, double aeps) const {
  //between for the euclidean component
  const ob::CompoundState *c = state->as<ob::CompoundState>();
  const ob::CompoundState *c1 = state->as<ob::CompoundState>();
  const ob::CompoundState *c2 = state->as<ob::CompoundState>();
  const ob::RealVectorStateSpace::StateType *e = 
    c->components[0]->as<ob::RealVectorStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType *e1 = 
    c1->components[0]->as<ob::RealVectorStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType *e2 = 
    c2->components[0]->as<ob::RealVectorStateSpace::StateType>();
  double ef = darrt::between(e, e1, e2, 3, deps);
  if (ef < 0) {
    return ef;
  }

  const ob::SO3StateSpace::StateType *o =
    c->components[1]->as<ob::SO3StateSpace::StateType>();
  const ob::SO3StateSpace::StateType *o1 =
    c1->components[1]->as<ob::SO3StateSpace::StateType>();
  const ob::SO3StateSpace::StateType *o2 =
    c2->components[1]->as<ob::SO3StateSpace::StateType>();
  
  double of = darrt::between(o, o1, o2, aeps);
  if (ef > 1.0 || of < 0) {
    return of;
  }
  if (of > 1.0) {
    return ef;
  }

  if (fabs(of - ef) > deps) {
    return -1.0;
  }
  return (of + ef)/2.0;
}

bool darrt::ObjectStateSpace::object_position(const ob::State *state, 
					      gm::Pose &pose) const {
  orc::SE3StateSpace_to_pose_msg(*(state->as<StateType>()), pose);
  return true;
}

bool darrt::ObjectStateSpace::set_object_position
(ob::State *state, const gm::Pose &pose) const {
  orc::pose_msg_to_SE3StateSpace(pose, *(state->as<StateType>()));
  return true;
}

void darrt::ObjectStateSpace::printState(const ob::State *state, 
					 std::ostream &out) const {
  const StateType *s = state->as<StateType>();
  out << "(" << object_.id << ": (" << s->getX() << ", " << s->getY() 
      << ", " << s->getZ() << "), (" << s->rotation().x << ", " 
      << s->rotation().y << ", " << s->rotation().z << ", " 
      << s->rotation().w << "), control: ";
  if (s->control()) {
    out << s->control()->str();
  } else {
    out << "None";
  }
  bool doubled = false;
  if (s->support_surface) {
    out << ", Support Surface: " << s->support_surface->name();
    if (s->attach_link.size()) {
      out << ", Attach Link: " << s->attach_link;
      doubled  = true;
    }
  } else {
    out << ", Attach Link: " << s->attach_link;
  }
  out << ")";
  if (doubled) {
    pause("Above state has support surface AND attach link!");
  }
}

bool darrt::ObjectStateSpace::update_model
(const ob::State *new_state, EnvironmentInterface *env) const {

  //check the bounds
  const ob::RealVectorStateSpace::StateType *s = 
    new_state->as<ob::CompoundState>()->
    as<ob::RealVectorStateSpace::StateType>(0);
  const ob::RealVectorBounds &bounds = getBounds();
  if (bounds.low.size() == 3 && bounds.high.size() == 3) {
    if (s->values[0] < bounds.low[0] || s->values[0] > bounds.high[0] ||
	s->values[1] < bounds.low[1] || s->values[1] > bounds.high[1] ||
	s->values[2] < bounds.low[2] || s->values[2] > bounds.high[2]) {
      return false;
    }
  }
  
  //check the gravity constraint
  const StateType *os = new_state->as<StateType>();
  if (!os->attach_link.size() && !os->support_surface) {
    if (debug_level >= DCOLLISIONS) {
      ROS_INFO("Object state invalid because %s is floating", getName().c_str());
    }
    return false;
  }
  
  
  const ob::SO3StateSpace::StateType *o =
    new_state->as<ob::CompoundState>()->as<ob::SO3StateSpace::StateType>(1);
  double norm = sqrt(o->x*o->x + o->y*o->y + o->z*o->z + o->w*o->w);
  if (fabs(norm - 1.0) > DIST_EPS) {
    ROS_ERROR("Invalid state because the quaternion does not have a norm of 1.  Norm is %f", norm);
    return false;
  }
  
  
  //figure out its new location
  geometry_msgs::Pose obj_pose;
  if (!object_position(new_state, obj_pose)) {
    return false;
  }
  
  //check the orientation constraint for the object... this is sort of dumb
  //should have a better way of specifying it...
  const RoundObject *roundobj = dynamic_cast<const RoundObject *>(&object_);
  if (roundobj && (!inrange(obj_pose.orientation.x, 0, ANGLE_EPS) || 
		   !inrange(obj_pose.orientation.y, 0, ANGLE_EPS))) {
    if (debug_level >= DCOLLISIONS) {
	ROS_INFO("Round object orientation constraint violated");
    }
    return false;
  }

  //put a copy into the map (don't actually change the original object
  //so we know where it started)
  an::CollisionObject copy = object();
  for (unsigned int i = 0; i < copy.poses.size(); i++) {
    copy.poses[i] = transform_pose(copy.poses[i], obj_pose);
  }
 
  //and update it in the interface
  env->addStaticObject(copy);

  return true;
}


darrt::ObjectStateValidityChecker::ObjectStateValidityChecker
(ob::SpaceInformation *si, 
 collision_space::EnvironmentModel::AllowedCollisionMatrix 
 &default_allowed_collisions, bool update_model, bool check_robot_collisions) : 
  CollisionAwareStateValidityChecker(si, default_allowed_collisions) {
  space_ = si->getStateSpace()->as<ObjectStateSpace>();
  update_model_ = update_model;
  check_robot_collisions_ = check_robot_collisions;
  ros::NodeHandle n("~");
  collision_pub_ = 
    n.advertise<visualization_msgs::Marker>("collisions", 1);
}

bool darrt::ObjectStateValidityChecker::isValid(const ob::State *state) const {

  if (update_model_) {
    if (!reconcile_collision_model(state)) {
      if (debug_level >= DPROPAGATE) {
	ROS_INFO("Object is out of bounds");
      }
      //weren't in bounds
      return false;
    }
  }

  if (debug_level >= DCOLLISIONS) {
    ROS_INFO("Doing object collision check");
  }

  bool collision = environment_interface_->isObjectInCollision
    (space_->object().id);
  if (collision) {
    if (debug_level >= DRRT) {
      ROS_INFO("Object %s is in collision", space_->object().id.c_str());
    }
    if (debug_level >= DPROPAGATE) {
      std::vector<arm_navigation_msgs::ContactInformation> contacts;
      environment_interface_->getAllEnvironmentCollisionsForObject
	(space_->object().id, contacts);
      ROS_INFO("There are %u contacts", (unsigned int)contacts.size());
      for (size_t i = 0; i < contacts.size(); i++) {
	ROS_INFO("Contact between %s and %s", 
		 contacts[i].contact_body_1.c_str(),
		 contacts[i].contact_body_2.c_str());
	//publish it
	geometry_msgs::PoseStamped cp;
	cp.header = contacts[i].header;
	cp.header.stamp = ros::Time(0);
	cp.pose.position = contacts[i].position;
	cp.pose.orientation.w = 1.0;
	for (int dm = 0; dm < 100; dm++) {
	  display_marker(collision_pub_, cp, 
			 visualization_msgs::Marker::SPHERE,
			 0.05, 1.0, 0.0, 0.0, 0.8, contacts[i].contact_body_1+
			 "_"+contacts[i].contact_body_2, i);
	}
      }
    }
  }
  if (!collision && check_robot_collisions_) {
    collision = environment_interface_->isKinematicStateInObjectCollision
      (*(environment_interface_->kinematicState()), 
       space_->object().id);
    if (collision && debug_level >= DRRT) {
      //i should write something to look at these
      ROS_INFO("Object %s is in collision with robot", 
	       space_->object().id.c_str());
    }
  }

  if (update_model_) {
    revert_collision_model();
  }
  return !collision;
}
