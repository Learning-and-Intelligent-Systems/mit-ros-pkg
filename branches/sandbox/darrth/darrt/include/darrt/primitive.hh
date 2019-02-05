#ifndef __DARRT_PRIMITIVE__HH__
#define __DARRT_PRIMITIVE__HH__


#include <string>
#include <vector>

#include <arm_navigation_msgs/CollisionObject.h>

#include <ompl/base/State.h>
#include <ompl/control/SpaceInformation.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace darrt {

  class Primitive;
  
  //these must be defined by you
  class TransitPrimitive;
  class TransferPrimitive;

  typedef boost::function<void (const ob::State *from, ob::State *sample)> DARRTProjectionFunction;

  class PrimitiveInstance {
  public:

    PrimitiveInstance() 
    {type_ = ""; prim_ = NULL; source_ = NULL; destination_ = NULL; turns_ = 0;}
    PrimitiveInstance(const Primitive *p, std::string type);
    PrimitiveInstance(const Primitive *p, std::string type, const ob::State *s,
		      const ob::State *d) {initialize(p, type, s, d); turns_ = 0;}

    void set_turns(unsigned int t) {turns_ = t;}
    unsigned int turns() const {return turns_;}
    const Primitive *primitive() const {return prim_;}
    ob::State *source() {return source_;}
    ob::State *destination() {return destination_;}
    const ob::State *source() const {return source_;}
    const ob::State *destination() const {return destination_;}

    
    virtual PrimitiveInstance *copy() const = 0;
    virtual bool equals(const PrimitiveInstance *other) const;
    virtual std::string str() const;
    std::string type() const {return type_;}
    virtual ~PrimitiveInstance();
    
    virtual bool propagate(const ob::State *state,
			   const double duration,
			   ob::State *result) const = 0;


  protected:
    const Primitive *prim_;
    ob::State *source_, *destination_;
    unsigned int turns_;
    std::string type_;
    
    bool initialize(const Primitive *p, std::string t, const ob::State *s,
		    const ob::State *d);
  };

  class InvalidPrimitiveInstance : public PrimitiveInstance {
  public:
    InvalidPrimitiveInstance(unsigned int turns) : 
      PrimitiveInstance() {turns_ = turns;}

    PrimitiveInstance *copy() const {return new InvalidPrimitiveInstance(turns_);}
    bool equals(const PrimitiveInstance *other) const
    {return dynamic_cast<const InvalidPrimitiveInstance *>(other) && 
	other->turns() == turns_;}
    std::string str() const;
    bool propagate(const ob::State *state, const double duration, ob::State *result) const;

  };

  typedef std::vector<PrimitiveInstance *> PIList;

  class Primitive {
  public:
    Primitive(std::string name) {name_ = name; is_setup_ = false;}
    virtual bool setup(const oc::SpaceInformation *si) 
    {si_ = si; is_setup_ = true; return true;}
    virtual bool is_setup() const {return is_setup_;}
    const oc::SpaceInformation *space_information() const {return si_;}

    virtual DARRTProjectionFunction getProjectionFunction() const
    {return DARRTProjectionFunction();}
    
    virtual bool transit() const = 0;
    virtual bool transfer() const = 0;
    virtual Primitive *copy() const = 0;
    virtual bool useful(const ob::State *source, 
			const ob::State *destination) const = 0;
    //returns false if there is a collision or constraint violation
    virtual unsigned int sampleTo(const ob::State *source,
				  const ob::State *target,
				  PIList &clist) const = 0;

    virtual bool execute(const std::vector<const ob::State *> &path) const 
    {ROS_INFO("No execution for this primitive"); return true;}
    virtual std::string str() const {return name();}
    virtual std::string name() const {return name_;}
    virtual bool matches(const Primitive *other, const ob::State *st) const 
    {return (other && !name_.compare(other->name()));}
    virtual bool becomesGoal() const {return transfer();}
    virtual ~Primitive() {}

  protected:
    std::string name_;
    const oc::SpaceInformation *si_;
    bool is_setup_;
  };

  class TransferPrimitive : public Primitive {
  public:
    TransferPrimitive(std::string name) : Primitive(name) {} 
    virtual bool transit() const {return false;}
    virtual bool transfer() const {return true;}
  };

  class TransitPrimitive : public Primitive {
  public:
    TransitPrimitive(std::string name) : Primitive(name) {}
    virtual bool transit() const {return true;}
    virtual bool transfer() const {return false;}
  };



  typedef std::vector<const Primitive *> PrimitiveList;


}

#endif //primitive.hh
