#ifndef __DARRT_CONVERTER_HH__
#define __DARRT_CONVERTER_HH__

#include "darrt/types.hh"
#include "darrt/space.hh"
#include "darrt/state_transformer.hh"

#include "darrt_msgs/DARRTState.h"

#include <ompl/control/PathControl.h>

namespace darrt {

  void convert_to_primitive_path
  (const oc::PathControl &path, PrimitivePath &prim_path);
  
  class DARRTConverter {
  public:
    DARRTConverter() {}

    void convert_ompl_state_to_darrt_state
    (const ob::State *ompl_state, darrt_msgs::DARRTState &darrt_state,
     const DARRTStateSpace *space) const;
    
    void convert_darrt_state_to_ompl_state
    (const darrt_msgs::DARRTState &darrt_state, ob::State *ompl_state,
     const DARRTStateSpace *space) const;

    const StateTransformer &transformer() const {return transformer_;}

  protected:
    StateTransformer transformer_;
  };
    
};

#endif //conversions.hh
