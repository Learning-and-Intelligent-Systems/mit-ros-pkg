#!/usr/bin/env python
import roslib; roslib.load_manifest('pressure_practice')
import rospy
from drive_base import drive_base_client
from pressure_listener import *
from arm_control import *
from table_detector import *
from head_control import *
from right_arm_control import *
import tf
sector_min = -.1
sector_max = .4
downward_force = -9
height_above_table = .12
wipe_quat = (-0.50128038855639834, 0.51379924694410661, 0.51694690583044567, 0.46636273697706976)
class table_wiper:
    """
    Table Wiping Object for the PR2
	
    Class containing manipulation methods which encapsulate table wiping actions for the PR2. This class assumes that the left gripper of the robot has pressure sensors on it and that the EE_Cart_Imped Controller is running on both arms. 
    """
    def __init__(self):
        """
	Creates a new table wiping object.
	"""
        print "are you crazy"
	self.arm_control = arm_control()
        print "created arm control"
	self.drive_base = drive_base_client.DriveBaseClient()
        print "created drive_base_client"
	self.pressure_listener = pressure_listener()
        print "created pressure_listener"
	self.head_control = head_control()
        print "YOU SHOULD HAVE WRITTEN DOCUMENTATION YOU IDIOT!"
        self.head_control.look_at(.5,0,0)
        self.table_detector = table_detector(self.arm_control.tf_listener)
	self.sector_width = 0.5			##Width of the sector we can effectively wipe
        table = self.table_detector.get_table()
        print table
        self.table_height = table[0][2]
        print table[2][0]
    def wipe_table(self):
        """
	Commands the PR2 to wipe a table using feedback from the pressure sensors. Assumes that the tabletop manipulation pipeline is running. Additionally assumes that the PR2 is already in front of a table.
		
	Assumes that the manipulation pipeline, ee_cart_imped controllers, and drive_base service are all running.
		
	NB: At the moment, it only wipes two sectors. Eventually it should be augmented to continue to wipe until it can see the left edge of the table.
	"""
        self.find_right_edge()
        self.wipe_side_sector('r')
        self.move_to_next_sector()
        self.wipe_sector()

    def wipe_side_sector(self, edge):
	"""
	Wipes a sector of a table. Depending on whether it is the right or left side ensures that the arm does not slide off the edge.
	
	@type edge: char
	@param edge: char that determines if the robot is wiping the right or left edge. 'r' will wipe the right side and 'l' will fill the left edge.
	"""
        if edge == 'r':##Right edge is present
            print "right"

	    (trans, rot) = self.arm_control.get_hand_pose()
            print "edge = " + str(trans[1]+.05)
            self.arm_control.add_trajectory_point(trans[0], trans[1], trans[2] + height_above_table, 
                                                  wipe_quat[0], wipe_quat[1], wipe_quat[2], wipe_quat[3],
						  1000, 1000, 1000, 30, 30, 30,
						  False, False, False, False, False, False, 2)
	    self.arm_control.execute()
	   
            self.wipe_sector(trans[1]+.05)
        else:
            print "left"
            self.arm_control.add_trajectory_point(0.75, sector_min, self.table_height + height_above_table,
                                                  wipe_quat[0], wipe_quat[1], wipe_quat[2], wipe_quat[3],
		    			          1000, 1000, 1000, 30, 30, 30,
					          False, False, False, False, False, False, 3)
	    self.arm_control.add_trajectory_point(0.75, sector_min, self.table_height,
                                                 wipe_quat[0], wipe_quat[1], wipe_quat[2], wipe_quat[3],
					          1000, 1000, downward_force,   30, 30, 30,
					          False, False, True,  False, False, False, 3)
            self.arm_control.execute()
	    self.arm_control.add_trajectory_point(.75,    sector_max, self.table_height,
                                                   wipe_quat[0], wipe_quat[1], wipe_quat[2], wipe_quat[3],
					          1000, 1000, downward_force,   30, 30, 30,
					          False, False, True,  False, False, False, 5)
	    self.arm_control.execute(True, False)
	    while True:
                if rospy.is_shutdown():
                    exit(0)
		if not self.pressure_listener.is_touching():
		    self.arm_control.stop_in_place()
		    break

	    (trans, rot) = self.arm_control.get_hand_pose()
            self.wipe_sector(sector_min, trans[1]- .06)

    def wipe_sector(self, y_min = sector_min, y_max = sector_max):
	"""
	Internal method to wipe from two different y coordinates.
	
	@type y_min: double
	@param y_min: the minimum y value that the robot should wipe on this call.
	@type y_max: double
	@type y_max: the maximum y value that the robot should wipe during this call.
	"""
	y = y_min
	while y < y_max:
#            self.r_arm_control.move_to(y)
            self.arm_control.add_trajectory_point(0.75, y, self.table_height + height_above_table,
                                                  wipe_quat[0], wipe_quat[1], wipe_quat[2], wipe_quat[3],
		    			          1000, 1000, 1000, 30, 30, 30,
					          False, False, False, False, False, False, 3)
	    self.arm_control.add_trajectory_point(0.75, y, self.table_height, 
						  wipe_quat[0], wipe_quat[1], wipe_quat[2], wipe_quat[3],
					          1000, 1000, downward_force,   30, 30, 30,
					          False, False, True,  False, False, False, 2)
            self.arm_control.execute()
	    self.arm_control.add_trajectory_point(0,    y, self.table_height,
                                                  wipe_quat[0], wipe_quat[1], wipe_quat[2], wipe_quat[3],
					          1000, 1000, downward_force,   30, 30, 30,
					          False, False, True,  False, False, False, 5)
	    self.arm_control.execute(True, False)
	    while True:
                if rospy.is_shutdown():
                    exit(0)
		if not self.pressure_listener.is_touching():
		    self.arm_control.stop_in_place()
		    break

	    (trans, rot) = self.arm_control.get_hand_pose()

	    self.arm_control.add_trajectory_point(trans[0], trans[1], trans[2] + height_above_table, 
                                                  wipe_quat[0], wipe_quat[1], wipe_quat[2], wipe_quat[3],
						  1000, 1000, 1000, 30, 30, 30,
						  False, False, False, False, False, False, 2)
	    self.arm_control.execute()
	    y += 0.1

	

    def find_right_edge(self):
	"""
	Places the arm on the table and drives the base until the arm is aligned with the right edge of the table. 
	"""
        self.arm_control.add_trajectory_point(0.75, 0, self.table_height + height_above_table, 
                                                  wipe_quat[0], wipe_quat[1], wipe_quat[2], wipe_quat[3],
		    			          1000, 1000, 1000, 30, 30, 30,
					          False, False, False, False, False, False, 3)
        self.arm_control.add_trajectory_point(0.75, 0, self.table_height, 
                                                  wipe_quat[0], wipe_quat[1], wipe_quat[2], wipe_quat[3],
					          1000, 1000, downward_force-2,   30, 30, 30,
					          False, False, True,  False, False, False, 6)
        self.arm_control.execute()
	while self.pressure_listener.is_touching():
             self.drive_base.move(0, -0.01, 0.1)
        self.arm_control.stop_in_place()

    def move_to_next_sector(self):
	"""
	Moves the robot to the next sector that is should wipe.
	
	NB: This is feedforward, so if the robot needs to check to ensure that there is another sector to wipe before it moves further.
	"""
        self.arm_control.stop_in_place()
        self.arm_control.add_trajectory_point(.4, .5, 0, 
				      wipe_quat[0], wipe_quat[1], wipe_quat[2], wipe_quat[3],
                                      1000, 1000, 1000, 30, 30, 30,
                                      False, False, False, False, False,
                                      False, 5.0)
        self.arm_control.execute(True, False)
        self.drive_base.move(0, self.sector_width, 0.10)

    def left_edge_present(self):
	"""
	Method to use the tabletop manipulation pipeline to determine if the left edge of the table is near the robot. 
		
	@returns: True if the robot's wiping sector intersects with the left edge of the table.
		
	NB: this is currently not used by the table wiper.
	"""
        self.head_control.look_at(.6, sector_max, self.table_height)
        containsLeftEdge = False
        ((x,y,z), (y_min, y_max)) = self.table_detector.get_table()
        print "y_max:" + str(y_max)
        print "y_min:" + str(y_min)
        if y_max < sector_max:
             containsLeftEdge = True
        return containsLeftEdge
    def right_edge_present(self):
        ((x,y,z), (y_min, y_max)) = self.table_detector.get_table()
        if y_min > sector_min:
            return True
        else: return False
if __name__ == "__main__":
    rospy.init_node('table_wiper')
    success = True
    print "we are starting now"
    wiper = table_wiper()
    print wiper.arm_control.get_hand_pose()
    success = wiper.wipe_table()
    if success:
       print "Tests Passed :)"
