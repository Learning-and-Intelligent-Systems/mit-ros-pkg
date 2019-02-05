#ifndef OMPL_CONTROL_BIDIRECTED_CONTROL_SAMPLER_
#define OMPL_CONTROL_BIDIRECTED_CONTROL_SAMPLER_

#include "ompl/control/DirectedControlSampler.h"

namespace ompl
{
  namespace control
  {
    ClassForward(BiDirectedControlSampler);
    
    class BiDirectedControlSampler : public DirectedControlSampler 
    {
      
    public:
      
    BiDirectedControlSampler(const SpaceInformation *si) : 
      DirectedControlSampler(si) {}
     
      virtual ~BiDirectedControlSampler(void) {}

      virtual int sampleFrom
	(Control *control, const base::State *target, 
	 const base::State *source) = 0;

      virtual int sampleFrom
	(Control *control, const Control *previous,
	 const base::State *target, const base::State *source) = 0;
      
    };
  }
}

#endif //BiDirectedControlSampler.h
