#include <ros/ros.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <geometric_shapes_msgs/Shape.h>
#include "bakebot/AttachDetach.h"

bool attach_detach(bakebot::AttachDetach::Request &req, bakebot::AttachDetach::Response &res) {

    ros::NodeHandle nh;
    ros::Publisher att_object_in_map_pub_;
    att_object_in_map_pub_  = nh.advertise<mapping_msgs::AttachedCollisionObject>("attached_collision_object", 10);
    sleep(2);

    //add a cylinder into the collision space attached to the r_gripper_r_finger_tip_link
    mapping_msgs::AttachedCollisionObject att_object;
    bool isRightArm = (req.right_arm == 1);
    if (isRightArm) {
        att_object.link_name = "r_gripper_r_finger_tip_link";
        att_object.touch_links.push_back("r_gripper_palm_link");
        att_object.touch_links.push_back("r_gripper_r_finger_link");
        att_object.touch_links.push_back("r_gripper_l_finger_link");
        att_object.touch_links.push_back("r_gripper_l_finger_tip_link");
    } else { 
        att_object.link_name = "l_gripper_l_finger_tip_link";
        att_object.touch_links.push_back("l_gripper_palm_link");
        att_object.touch_links.push_back("l_gripper_r_finger_link");
        att_object.touch_links.push_back("l_gripper_l_finger_link");
        att_object.touch_links.push_back("l_gripper_r_finger_tip_link");
    }

    att_object.object.id = "attached_spatula";
    if (req.attach == 1) {
        //att_object.object.operation.operation = mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;
        att_object.object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
        ROS_INFO("adding the object to the hand");
    } else {
        ROS_INFO("removing the object from the hand");
        att_object.object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
    }
    att_object.object.header.frame_id = isRightArm ? "r_gripper_r_finger_tip_link" : "l_gripper_l_finger_tip_link";
    att_object.object.header.stamp = ros::Time::now();
    geometric_shapes_msgs::Shape object;
    object.type = geometric_shapes_msgs::Shape::CYLINDER;
    object.dimensions.resize(2);
    object.dimensions[0] = req.radius;  //TODO may have to reverse these two
    object.dimensions[1] = req.length;
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.707;
    pose.orientation.y = 0;
    pose.orientation.z = 0.707;
    pose.orientation.w = 0;
    att_object.object.shapes.push_back(object);
    att_object.object.poses.push_back(pose);

    att_object_in_map_pub_.publish(att_object);

    ROS_INFO("published collision map request");
    ROS_INFO("DONE");

    res.status = 0;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bakebot_attach_detach_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("bakebot_attach_detach_service", attach_detach);
    ROS_INFO("The bakebot attach/detach service is ready.");
    ros::spin();
    return 0;
}
