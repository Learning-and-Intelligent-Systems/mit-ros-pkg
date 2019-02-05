#include "darrt/primitive.hh"
#include "darrt/utils.hh"
#include "darrt/collision_aware_types.hh"

darrt::PrimitiveInstance::PrimitiveInstance(const Primitive *p, std::string type) {
  prim_ = p;
  type_ = type;
  source_ = prim_->space_information()->allocState();
  destination_ = prim_->space_information()->allocState();
  CollisionAwareState *cas = dynamic_cast<CollisionAwareState *>(source_);
  if (cas) {
    cas->set_control(prim_, type_);
  }
  cas = dynamic_cast<CollisionAwareState *>(destination_);
  if (cas) {
    cas->set_control(prim_, type_);
  }
  turns_ = 0;
}

bool darrt::PrimitiveInstance::initialize
(const Primitive *p, std::string type, const ob::State *s, const ob::State *d) {
  prim_ = p;
  type_ = type;
  source_ = prim_->space_information()->allocState();
  destination_ = prim_->space_information()->allocState();
  prim_->space_information()->copyState(source_, s);
  prim_->space_information()->copyState(destination_, d);
  CollisionAwareState *cas = dynamic_cast<CollisionAwareState *>(source_);
  if (cas) {
    cas->set_control(prim_, type_);
  }
  cas = dynamic_cast<CollisionAwareState *>(destination_);
  if (cas) {
    cas->set_control(prim_, type_);
  }
  return true;
}

bool darrt::PrimitiveInstance::equals(const PrimitiveInstance *other) const {
  return (other && prim_ == other->primitive() && type_ == other->type() &&
	  prim_->space_information()->equalStates(source_, other->source()) &&
	  prim_->space_information()->
	  equalStates(destination_, other->destination()));
}

std::string darrt::PrimitiveInstance::str() const {
  std::ostringstream ostr;
  ostr << type_ << "[" << (turns_) << "]" << "(";
  prim_->space_information()->printState(source_, ostr);
  ostr << ", ";
  prim_->space_information()->printState(destination_, ostr);
  ostr << ")";
  return ostr.str();
}

darrt::PrimitiveInstance::~PrimitiveInstance() {
  if (source_) {
    prim_->space_information()->freeState(source_);
  }
  if (destination_) {
    prim_->space_information()->freeState(destination_);
  }
}

std::string darrt::InvalidPrimitiveInstance::str() const {
  return "Invalid["+makestring(turns_)+"]";
}

bool darrt::InvalidPrimitiveInstance::propagate(const ob::State *state, const double duration, ob::State *result) const {
  prim_->space_information()->copyState(result, state);
  return true;
}
