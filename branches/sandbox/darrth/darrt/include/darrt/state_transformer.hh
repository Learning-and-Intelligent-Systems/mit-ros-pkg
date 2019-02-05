#ifndef __DARRT_STATE_TRANSFORMER_HH__
#define __DARRT_STATE_TRANSFORMER_HH__

#include <stdexcept>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <planning_environment/models/collision_models_interface.h>
#include <planning_environment/models/model_utils.h>
#include <tf/LinearMath/Transform.h>

namespace an = arm_navigation_msgs;
namespace gm = geometry_msgs;

namespace darrt {

  class StateTransformer {
  public:
    StateTransformer();
    gm::TransformStamped get_transform
    (std::string to_frame_id, std::string from_frame_id, 
     const an::RobotState &robot_state) const;

    gm::TransformStamped get_transform
    (std::string to_frame_id, std::string from_frame_id, 
     const planning_models::KinematicState &kstate) const;

    gm::Point transform
    (std::string new_frame_id, std::string old_frame_id, 
     const gm::Point &point, const an::RobotState &robot_state) const;
    gm::Quaternion transform
    (std::string new_frame_id, std::string old_frame_id, 
     const gm::Quaternion &quat, const an::RobotState &robot_state) const;
    gm::Vector3 transform
    (std::string new_frame_id, std::string old_frame_id, 
     const gm::Vector3 &vector, const an::RobotState &robot_state) const;
    gm::Pose transform
    (std::string new_frame_id, std::string old_frame_id, 
     const gm::Pose &pose, const an::RobotState &robot_state) const;
    gm::PointStamped transform
    (std::string new_frame_id, const gm::PointStamped &point, 
     const an::RobotState &robot_state) const;
    gm::QuaternionStamped transform
    (std::string new_frame_id, const gm::QuaternionStamped &quat, 
     const an::RobotState &robot_state) const;
    gm::Vector3Stamped transform
    (std::string new_frame_id, const gm::Vector3Stamped &vector,
     const an::RobotState &robot_state) const;
    gm::PoseStamped transform
    (std::string new_frame_id, const gm::PoseStamped &pose, 
     const an::RobotState &robot_state) const;

    gm::Point transform
    (std::string new_frame_id, std::string old_frame_id, 
     const gm::Point &point, const planning_models::KinematicState &robot_state) const;
    gm::Quaternion transform
    (std::string new_frame_id, std::string old_frame_id, 
     const gm::Quaternion &quat, const planning_models::KinematicState &robot_state)const;
    gm::Vector3 transform
    (std::string new_frame_id, std::string old_frame_id, 
     const gm::Vector3 &vector, const planning_models::KinematicState &robot_state) const;
    gm::Pose transform
    (std::string new_frame_id, std::string old_frame_id, 
     const gm::Pose &pose, const planning_models::KinematicState &robot_state) const;
    gm::PointStamped transform
    (std::string new_frame_id, const gm::PointStamped &point, 
     const planning_models::KinematicState &robot_state) const;
    gm::QuaternionStamped transform
    (std::string new_frame_id, const gm::QuaternionStamped &quat, 
     const planning_models::KinematicState &robot_state) const;
    gm::Vector3Stamped transform
    (std::string new_frame_id, const gm::Vector3Stamped &vector,
     const planning_models::KinematicState &robot_state) const;
    gm::PoseStamped transform
    (std::string new_frame_id, const gm::PoseStamped &pose, 
     const planning_models::KinematicState &robot_state) const;

    
    std::string world_frame_id() const
    {return world_frame_id_;}

    std::string robot_frame_id() const
    {return robot_frame_id_;}

    //removes slashes from frame ids if they have them
    std::string relative_frame(std::string frame_id) const;

  protected:
    std::string world_frame_id_, robot_frame_id_;


    bool world_transform(std::string frame_id, 
			 const planning_models::KinematicState &state,
			 tf::Transform &transform) const;
    planning_environment::RobotModels robot_model_;
  };

  class StateTransformException : public std::runtime_error {
  public:
    StateTransformException(const std::string &what) : std::runtime_error(what) {}
    virtual ~StateTransformException() throw() {}
  };
}

#endif //state_transformer.hh
