#ifndef CHAOS_DETECTION_H_
#define CHAOS_DETECTION_H_

#include <chaos/common.h>
#include <chaos/scene.h>
#include <chaos/testing.h>
#include <chaos/ObjectAnalysis.h>

#include <boost/any.hpp>

usnig namespace std;

using namespace std;


/*
 * These classes repesent a hypothesize-test framework for scene analysis.
 * The job of a SceneAnalyzer class is to explain *every* point in the scene,
 * either as part of a model or as noise.  Its output should be a scene
 * reconstruction, or more generally a probability distribution over scene
 * reconstructions.  Representing this probability distribution compactly
 * may require tree-based or graph-based conditional data structures.
 *
 * Searching over all possible scene reconstructions is impossible,
 * so there are two classes which will perform the necessary simplifications,
 * the Hypothesizer class and the Tester class.  The Hypothesizer
 * class makes guesses about where a model might be, using prior knowledge,
 * scene contexts, object trackers, and local evidence.  The Tester class
 * evaluates the probability of a hypothesis, given the scene, a partial
 * reconstruction, and any prior knowledge.
 *
 * When a list of Testers is given, they should be called in the
 * order of decreasing expected information rate--i.e. information gain per unit time.
 * That way, Testers that run fast and are very informative will be chosen first.
 *
 * The Aligner class is a type of Hypothesizer which takes a hypothesis
 * and improves it locally by fitting it to the scene.  The Tracker class
 * keeps track of all object state, and is used by both the Hypothesizer and
 * the Tester class as a source of prior knowledge.
 */



//--------------  ATTRIBUTES  -------------//


#define has_attribute(label, atype) ((label).attributes.find(atype) != (label).attributes.end())
#define get_attribute(label, atype, ctype) boost::any_cast< ctype >((label).attributes[atype])
#define set_attribute(label, atype, value) (label).attributes[atype] = value

/*
 * for example, assuming H is a Hypothesis:
 *
 *   set_attribute(H.label, MODEL_NAME, "LBlock");
 *   if (has_attribute(H.label, MODEL_NAME)
 *     string model_name = get_attribute(H.label, MODEL_NAME, string);
 */



namespace chaos {


  //------------------  Hypothesis classes  ------------------//


  enum LabelType {
    OBJECT,
    JUNK
  };

  enum LabelAttributeType {
    OBJECT_ID,
    OBJECT_POSE,
    
  };


  OBJECT_ID -> int
  OBJECT_POSE -> Pose




  // base class for multiple Hypotheses
  class MultiHypothesis : public Hypothesis {
  public:
    void addHypothesis(Hypothesis *h) {
      get_attribute(label, HYPOTHESIS_LIST, vector<Hypothesis*>).push_back(h);
    }

    const vector<Hypothesis*> & getHypothesisList() {
      return get_attribute(label, HYPOTHESIS_LIST, const vector<Hypothesis*> &);
    }
  };


  // represents a composite Hypothesis (H1 and H2 and H3 ...)
  class CompositeHypothesis : public MultiHypothesis {
  public:
    CompositeHypothesis() { label.type = COMPOSITE; }
  };


  // respresents a variable Hypothesis (H1 or H2 or H3 ...)
  class VariableHypothesis : public MultiHypothesis {
  public:
    VariableHypothesis() { label.type = VARIABLE; }
  };


  /* Example of Hypothesis operator overloading:

  class LabelAttribute {
    LabelAttributeType type;
    void *data;
  };

  class Label {
  public:
    LabelType type;
    string name;
    vector<LabelAttribute> attributes;
  };

  // e.g. "LBlock"
   



  //------------------  Hypothesis classes  ------------------//

  class Hypothesis {
  public:
    double fitness;   // fitness score
    Label label;
  };




  //------------------  Hypothesizer classes  -----------------//


  // good:   h1 && h2 && (h3a || h3b || h3c)
  // bad:    (h1 && h2 && h3a) || (h1 && h2 && h3b) || (h1 && h2 && h3c)
  //
  // Now suppose a new Hypothesizer, H, needs to resolve h3 to return h4
  //         h1 && h2 && ((h3a && (h4a || h4b)) || (h3b && (h4c || h4d)) || (h3b && (h4e || h4f)))
  // or,
  //         (h1=v1) && (h2=v2) && (h3={v3a,v3b,v3c}) && (h4|h3={v3a->{v4a,v4b}, v3b->{v4c,v4d}, v3c->{v4e,v4f}})



  // base Hypothesizer class
  class Hypothesizer {
  protected:
    Scene *scene_;                                // observed scene and computed scene features

  public:
    vector<LabelAttributeType> preconditions;    // list of labels required to run this Hypothesizer
    vector<LabelAttributeType> postconditions;   // list of labels created/updated by this Hypothesizer

    virtual Hypothesis *getHypothesis(Hypothesis *assumptions) const;
    void setScene(Scene *S) { scene_ = S; }
    Scene *getScene() { return scene_; }
  };


  // Applies a given hypothesizer to each hypothesis in a VariableHypothesis
  class BranchingHypothesizer : public Hypothesizer {
  protected:
    Hypothesizer *H_;

  public:
    BranchingHypothesizer(Hypothesizer *H) : H_(H) {}

    Hypothesis *getHypothesis(Hypothesis *assumptions) {
      if (assumptions->isVariable()) {
	VariableHypothesis *vh = new VariableHypothesis();
	const vector<Hypothesis*> &hlist = ((VariableHypothesis *)assumptions)->getHypothesisList();
	for (uint i = 0; i < hlist.size(); i++) {
	  Hypothesis *h = H_->getHypothesis(hlist[i]);
	  if (h->isVariable()) {
	    const vector<Hypothesis*> &hlist2 = ((VariableHypothesis *)h)->getHypothesisList();
	    for (uint j = 0; j < hlist2.size(); j++)
	      vh->addHypothesis(hlist2[j]);
	  }
	  else
	    vh->addHypothesis(h);
	}
	return vh;
      }
      return H_->getHypothesis(assumptions);
    }
  };


  // Chains two hypothesizers together
  class HypothesizerChain : public Hypothesizer {
  protected:
    Hypothesizer *H1_;
    Hypothesizer *H2_;

  public:
    HypothesizerChain(Hypothesizer *H1, Hypothesizer *H2) : H1_(H1), H2_(H2) {}
    Hypothesis *getHypothesis(Hypothesis *assumptions) { return H2_->getHypothesis(H1_->getHypothesis(assumptions)); }
  };


  // selects the best Hypothesis in a VariableHypothesis
  class BestHypothesisHypothesizer : public Hypothesizer {
  public:
    Hypothesis *getHypothesis(Hypothesis *assumptions) {
      Hypothesis *hmin;
      if (assumptions->isVariable()) {
	const vector<Hypothesis*> &hlist = ((VariableHypothesis *)assumptions)->getHypothesisList();
	double fmin = std::numeric_limits<double>::max();
	for (uint i = 0; i < hlist.size(); i++) {
	  if (hlist[i]->fitness < fmin) {
	    hmin = hlist[i];
	    fmin = hlist[i]->fitness;
	  }
	}
      }
      else
	hmin = assumptions;
      return hmin;
    }
  };


  //------------------  Tester classes  ------------------//

  //class Tester {};



  class Aligner {};
  class Tracker {};
  class SceneAnalyzer {};



  /* Find a model to fit the given range image */
  ObjectAnalysis fit_object(const ModelManager &model_manager, ModelTester *model_tester,
			    geometry_msgs::Pose init_pose, float table_height);



  /*** deprecated ***/
  //TrackedObject fit_object_to_range_image(const pcl::RangeImage &range_image, geometry_msgs::Pose init_pose, float table_height);

}

#endif
