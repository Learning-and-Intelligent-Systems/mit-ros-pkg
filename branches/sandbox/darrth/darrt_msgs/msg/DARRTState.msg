#Robot state
arm_navigation_msgs/RobotState robot_state

#positions of objects in the world
arm_navigation_msgs/CollisionObject[] collision_objects

arm_navigation_msgs/AttachedCollisionObject[] attached_collision_objects

#the collision objects that correspond to tables
string[] support_surface_ids

arm_navigation_msgs/OrderedCollisionOperations ordered_collisions