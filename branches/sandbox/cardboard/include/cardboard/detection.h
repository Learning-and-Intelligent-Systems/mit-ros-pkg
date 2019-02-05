#ifndef CARDBOARD_DETECTION_H_
#define CARDBOARD_DETECTION_H_
#ifndef EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#endif

#include <cardboard/common.h>
#include <cardboard/testing.h>
#include <cardboard/ObjectHypothesis.h>
#include <cardboard/SceneHypothesis.h>




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
 * the Hypothesizer class and the Test class.  The Hypothesizer
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



namespace cardboard {



  class SceneAnalyzer {};
  class Hypothesizer {};
  //class Tester {};
  class Aligner {};
  class Tracker {};



  /* Find a model to fit the given range image */
  ObjectHypothesis fit_object(const ModelManager &model_manager, ModelTester *model_tester,
			      geometry_msgs::Pose init_pose, float table_height, tf::TransformListener *tf_listener, std::vector<string> models);
  ObjectHypothesis align_object(const ModelManager &model_manager, ModelTester *model_tester,
				geometry_msgs::Pose init_pose, float table_height, string modelName);
  SceneHypothesis* detect_models(sensor_msgs::PointCloud2 &msg, string target_frame, std::vector<geometry_msgs::Polygon> surface_polygons,
				 tf::TransformListener *tf_listener, double table_filter_thresh, ModelManager *model_manager,
				 std::vector<string> models, ros::Publisher &debug_pub);  //dbug
  SceneHypothesis* align_models(sensor_msgs::PointCloud2 &msg, string target_frame, std::vector<geometry_msgs::Polygon> surface_polygons,
				std::vector<geometry_msgs::Pose> initial_poses, std::vector<string> models,
				tf::TransformListener *tf_listener, double table_filter_thresh, ModelManager *model_manager);



  /*** deprecated ***/
  //TrackedObject fit_object_to_range_image(const pcl::RangeImage &range_image, geometry_msgs::Pose init_pose, float table_height);

}

#endif
