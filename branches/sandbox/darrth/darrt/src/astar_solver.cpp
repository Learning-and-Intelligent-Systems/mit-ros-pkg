#include "darrt/astar_solver.hh"
#include "darrt/utils.hh"
#include <visualization_msgs/Marker.h>


unsigned int darrt::MinHeap::parent(unsigned int index) {
  return index/2;
}

bool darrt::MinHeap::clear() {
  heap_.clear();
  points_.clear();
  return true;
}

std::string darrt::MinHeap::str() {
  std::string ret = "[";
  for (size_t i = 0; i < heap_.size(); i++) {
    ret += " ("+makestring(heap_[i].point.x)+", "+
      makestring(heap_[i].point.y)+", "+makestring(heap_[i].cost) + ")";
  }
  ret += "]";
  return ret;
}

bool darrt::MinHeap::push(HeapItem h) {
  HeapItem inf = h;
  inf.cost = MATH_INF;
  unsigned int index = heap_.size();
  points_[h.point] = index;
  heap_.push_back(inf);
  ROS_INFO("Before decrease key in push heap is %s", str().c_str());
  bool val = decrease_key(index, h);
  ROS_INFO("After decrease key in push heap is %s", str().c_str());
  return val;
}

darrt::HeapItem darrt::MinHeap::pop() {
  if (heap_.size() == 0) {
    ROS_WARN("Pop called on empty heap!");
    return HeapItem();
  }
  HeapItem ret = heap_[0];
  heap_[0] = heap_.back();
  points_[heap_[0].point] = 0;
  heap_.pop_back();
  points_.erase(ret.point);
  
  heapify(0);
  return ret;
}

bool darrt::MinHeap::decrease_key(unsigned int index, HeapItem new_item) {
  HeapItem &item = heap_[index];
  if (item < new_item) {
    ROS_ERROR("Attempt to decrease heap key to a larger value");
    return false;
  }
  
  heap_[index] = new_item;
  points_[new_item.point] = index;
  
  while (index > 0 && heap_[index] < heap_[parent(index)]) {
//     ROS_INFO("Exchanging %d %d cost %f and %d %d cost %f",
// 	     heap_[index].point.x, heap_[index].point.y, heap_[index].cost,
// 	     heap_[parent(index)].point.x, heap_[parent(index)].point.y,
// 	     heap_[parent(index)].cost);
    if (!exchange(index, parent(index))) {
      ROS_ERROR("Error exchanging indexes");
      return false;
    }
    index = parent(index);
  }
  return true;
}

unsigned int darrt::MinHeap::find(HeapItem h) const {
  PointKeyMap::const_iterator pit = points_.find(h.point);
  if (pit == points_.end()) {
    return heap_.size();
  }
  return pit->second;
}

darrt::HeapItem darrt::MinHeap::at(unsigned int index) const {
  if (index >= heap_.size()) {
    pause("Attempt to call at for index larger than heap size");
    return HeapItem();
  }
  return heap_[index];
}

bool darrt::MinHeap::heapify(unsigned int index) {
  unsigned int left = 2*index;
  unsigned int right = 2*index+1;
  unsigned int smallest = index;
  if (left < heap_.size() && heap_[left] < heap_[index]) {
    smallest = left;
  }
  if (right < heap_.size() && heap_[right] < heap_[smallest]) {
    smallest = right;
  }
  if (smallest != index) {
    if (!exchange(smallest, index)) {
      ROS_ERROR("Unable to exchange indexes");
      return false;
    }
    heapify(smallest);
  }  
  return true;
}

bool darrt::MinHeap::exchange(unsigned int i1, unsigned int i2) {
  points_[heap_[i2].point] = i1;
  points_[heap_[i1].point] = i2;
  HeapItem tmp = heap_[i1];
  heap_[i1] = heap_[i2];
  heap_[i2] = tmp;
  return true;
}

double darrt::distance2D(const gm::Point &p1, const gm::Point &p2) {
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  return sqrt(dx*dx + dy*dy);
}

double darrt::distanceTaxi(const GridPoint &p1, const GridPoint &p2) {
  return fabs(p2.x - p1.x) + fabs(p2.y - p1.y);
}

darrt::AStarSolver::AStarSolver(const an::CollisionObject &obj,
				planning_environment::CollisionModelsInterface
				*collision_models_interface) {
  object_ = obj;
  collision_models_interface_ = collision_models_interface;
  last_goal_ = NULL;
  last_start_ = NULL;
  configured_ = false;
  ros::NodeHandle n("~");
  rpub_ = n.advertise<visualization_msgs::Marker>("object_solver", 1);
  ROS_INFO("Object is %s", object_.id.c_str());
}

bool darrt::AStarSolver::configure(const Info &info) {
  info_ = info;
  configured_ = true;
  return true;
}

bool darrt::AStarSolver::reset() {
  last_goal_ = NULL;
  last_start_ = NULL;
  return true;
}

gm::Point darrt::AStarSolver::convert
(const GridPoint &g, const gm::Point offset) const {
  gm::Point p;
  p.x = g.x*info_.resolution + offset.x;
  p.y = g.y*info_.resolution + offset.y;
  return p;
}

darrt::GridPoint darrt::AStarSolver::convert
(const gm::Point &p, const gm::Point offset) const {
  GridPoint g;
  g.x = int((p.x - offset.x)/info_.resolution + 0.5);
  g.y = int((p.y - offset.y)/info_.resolution + 0.5);
  return g;
}

bool darrt::AStarSolver::update_object_location(const gm::Point &point) const {
  collision_models_interface_->deleteStaticObject(object_.id);

  an::CollisionObject copy;
  copy.id = object_.id;
  copy.shapes = object_.shapes;
  copy.poses.clear();
  gm::Pose obj_pose;
  obj_pose.position = point;
  obj_pose.position.z = object_.poses[0].position.z;
  obj_pose.orientation = object_.poses[0].orientation;
  copy.poses.push_back(obj_pose);
  collision_models_interface_->addStaticObject(copy);
  return true;
}

bool darrt::AStarSolver::revert_object_location() const {
  //remove the object id (with the old location) from the collision models
  collision_models_interface_->deleteStaticObject(object_.id);
  //re-add it in its original place
  collision_models_interface_->addStaticObject(object_);
  return true;
}


bool darrt::AStarSolver::is_valid(const gm::Point &point) const {

  //check that the point is inside the bounds
  if (point.x < info_.bounds.low[0] || point.x > info_.bounds.high[0] ||
      point.y < info_.bounds.low[1] || point.y > info_.bounds.high[1]) {
    return false;
  }


  if (!update_object_location(point)) {
    pause("Unable to update object location to check state validity");
    return false;
  }

  bool collision = collision_models_interface_->isObjectInCollision
    (object_.id);
  if (collision && debug_level >= DRRT) {
    ROS_INFO("Object %s is in collision", object_.id.c_str());
    std::vector<arm_navigation_msgs::ContactInformation> contacts;
    collision_models_interface_->getAllEnvironmentCollisionsForObject
      (object_.id, contacts);
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
      for (int c = 0; c < 100; c++) {
	display_marker(rpub_, cp, 
		       visualization_msgs::Marker::SPHERE,
		       0.05, 1.0, 0.0, 0.0, 0.8, contacts[i].contact_body_1+
		       "_"+contacts[i].contact_body_2, i);
      }
    }
  }
  if (!collision) {
    //should ignore the movable links
    collision = collision_models_interface_->
      isKinematicStateInObjectCollision
      (*(collision_models_interface_->getPlanningSceneState()), object_.id);
    if (collision && debug_level >= DRRT) {
      //i should write something to look at these
      ROS_INFO("Object %s is in collision with robot", object_.id.c_str());
    }
  }
  revert_object_location();
  return !collision;
}


bool darrt::AStarSolver::plan(const gm::PoseStamped &goal,
			      const gm::Point *start_ptr) {
  if (!configured_) {
    ROS_ERROR("Must configure before planning!");
    return false;
  }
  path_.clear();
  if (!is_valid(goal.pose.position)) {
    ROS_ERROR("Inavlid goal!");
    return false;
  }

  gm::Point offset;
  if (start_ptr) {
    offset = *start_ptr;
  } else {
    offset = object_.poses[0].position;
  }

  GridPoint pgoal = convert(goal.pose.position, offset);
  HeapItem start;
  start.point.x = 0;
  start.point.y = 0;
 

  if (last_goal_ != &goal || last_start_ != start_ptr) {
    //start a new search... don't continue from the last one
    last_goal_ = &goal;
    last_start_ = start_ptr;

    start.g = 0;
    start.h = distanceTaxi(start.point, pgoal);
    start.cost = start.g + start.h;
    
    //this is a class variable so we can keep generating plans
    heap_.clear();
    heap_.push(start);
    parents_[start.point] = start.point;
  }

  HeapItem curr;
  bool foundpath=false;
  while (!heap_.empty()) {
    curr = heap_.pop();
    gm::Point curr_loc = convert(curr.point, offset);
    ROS_INFO("Curr = %f %f cost = %f", curr_loc.x, curr_loc.y, curr.cost);
    if (!is_valid(curr_loc)) {
      continue;
    }
    if (curr.point == pgoal) {
      foundpath = true;
      break;
    }
    
    std::vector<GridPoint> children(4);
    children[0] = GridPoint(curr.point.x - 1, curr.point.y);
    children[1] = GridPoint(curr.point.x + 1, curr.point.y);
    children[2] = GridPoint(curr.point.x, curr.point.y - 1);
    children[3] = GridPoint(curr.point.x, curr.point.y + 1);

    for (size_t i = 0; i < children.size(); i++) {
      HeapItem c;
      c.point = children[i];
      if (parents_.find(c.point) != parents_.end()) {
	continue;
      }
      c.g = curr.g + 1;
      c.h = distanceTaxi(children[i], pgoal);
      c.cost = c.g + c.h;
      gm::Point chp = convert(c.point, offset);
      ROS_INFO("Child %d %d with cost %f", c.point.x, c.point.y, c.cost);
      unsigned int index = heap_.find(c);
      if (index < heap_.size()) {
	ROS_INFO("Heap already contains (%d %d) at index %u is (%d %d)",
		 c.point.x, c.point.y, index, heap_.at(index).point.x,
		 heap_.at(index).point.y);
	if (c.cost < heap_.at(index).cost) {
	  heap_.decrease_key(index, c);
	  parents_[c.point] = curr.point;
	}
      } else {
	heap_.push(c);
	parents_[c.point] = curr.point;
      }
    }
  }

  if (!foundpath) {
    ROS_WARN("A*: Unable to find path");
    return false;
  }
  
  //actual goal position
  Path rpath;
  rpath.push_back(goal.pose.position);
  GridPoint cp = curr.point;
  PointMap::iterator pit = parents_.find(cp);
  while (pit != parents_.end() && !(cp == start.point)) {
    cp = pit->second;
    rpath.push_back(convert(cp, offset));
    pit = parents_.find(cp);
    ROS_INFO("cp = %d %d", cp.x, cp.y);
  }

  //reverse the path
  path_.resize(rpath.size());
  for (size_t i = 0; i < rpath.size(); i++) {
    path_[rpath.size()-i-1] = rpath[i];
  }
  
  return true;
}

void darrt::AStarSolver::display_solution() const {

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = object_.header.frame_id;
  pose.header.stamp = ros::Time(0);
  pose.pose.position.z = object_.poses[0].position.z;
  pose.pose.orientation.w = 1.0;
  ROS_INFO("Path is:");
  for (size_t i = 0; i < path_.size(); i++) {
    pose.pose.position.x = path_[i].x;
    pose.pose.position.y = path_[i].y;
    for (size_t j = 0; j < 100; j++) {
      display_marker(rpub_, pose, 
		     visualization_msgs::Marker::SPHERE,
		     0.05, 1.0, 0.0, 1.0, 0.5, "path", i);
    }
    ROS_INFO("(%f, %f)", path_[i].x, path_[i].y);
  }
}
