#include "darrt/object_primitives.hh"
#include "darrt/primitive.hh"

darrt::ObjectPrimitive::ObjectPrimitive
(const Primitive *lower_level_primitive, unsigned int index) : 
  Primitive("object_"+lower_level_primitive->name()) {
  lower_level_primitive_ = lower_level_primitive;
  index_ = index;
}

bool darrt::ObjectPrimitive::setup(const oc::SpaceInformation *si) {
  if (!lower_level_primitive_->is_setup()) {
    ROS_ERROR("Must call setup for lower level primitive before calling setup for object level primitive");
    return false;
  }
  Primitive::setup(si);
  dspace_ = si->getStateSpace()->as<DARRTStateSpace>();
  ospace_ = dspace_->as<ObjectStateSpace>(index_);
  return true;
}

bool darrt::ObjectPrimitive::useful(const ob::State *source,
				    const ob::State *destination) const {
  return lower_level_primitive_->useful(source, destination);
}

unsigned int darrt::ObjectPrimitive::sampleTo
(const ob::State *source, const ob::State *target, PIList &clist) const {
  return lower_level_primitive_->sampleTo(source, target, clist);

  PIList ll_clist;

  lower_level_primitive_->sampleTo(source, target, ll_clist);

  unsigned int turns = 0;
  for (unsigned int i = 0; i < ll_clist.size(); i++) {
    //only take those samples for which the object moves
    if (dspace_->near_object_states(index_, ll_clist[i]->source(),
				    ll_clist[i]->destination())) {
      delete ll_clist[i];
      continue;
    }
    clist.push_back(ll_clist[i]);
    turns += ll_clist[i]->turns();
  }
  return turns;
}
