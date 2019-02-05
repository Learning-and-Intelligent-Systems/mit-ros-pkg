#ifndef __DARRT_OBJECT_PRIMITIVES_HH__
#define __DARRT_OBJECT_PRIMITIVES_HH__

#include "darrt/primitive.hh"
#include "darrt/space.hh"
#include "darrt/collision_aware_types.hh"
#include "darrt/object_space.hh"

namespace darrt {
  
  class ObjectPrimitive : public Primitive {
  public:
    ObjectPrimitive(const Primitive *lower_level_primitive, unsigned int index);
    virtual bool setup(const oc::SpaceInformation *si);
    virtual bool useful(const ob::State *source, 
			const ob::State *destination) const;
    virtual unsigned int sampleTo(const ob::State *source, 
				  const ob::State *target,
				  PIList &clist) const;
    virtual bool transit() const {return false;}
    virtual bool transfer() const {return false;}
    const Primitive *lower_level_primitive() const 
    {return lower_level_primitive_;}
    
  protected:
    const Primitive *lower_level_primitive_;
    unsigned int index_;
    const ob::State *starting_state_;
    const DARRTStateSpace *dspace_;
    const ObjectStateSpace *ospace_;
  }; 
};

#endif //object_primitives.hh
