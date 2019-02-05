#!/usr/bin/env python
import roslib
roslib.load_manifest('bakebot')
import rospy
import sys
from object_manipulation_msgs.msg import ManipulationResult, GraspableObject
from sequential_ingredient_dealer import SequentialIngredientDealer
from arm_client import *
import pickle
import bowl_dealer

class SIDTest():
    
    def __init__(self):
        self.sid = SequentialIngredientDealer()

    def ui(self):
        while True:
            print '\nMain Menu:'
            print '\n\tRobot Positioning:'
            print '\t\t1) move both arms to side'
            print '\t\t2) raise torso (toso_lifter.up())'

            print '\n\tRaw Tabletop Detection:'
            print '\t\t5) find the table'
            print '\t\t6) call tabletop detection'
            print '\t\t7) filter ingredients (must call 6 first)'
            print '\t\t8) reset collision map'
            print '\t\t9) reset STATIC collision map'

            print '\n\tSID Handles:'
            print '\t\t10) scan tabletop'
            print '\t\t11) deal 1 ingredient'
            print '\t\t12) deal all ingredients'

            print '\n\tIntermediate steps:'
            print '\t\t20) grab ingredient'
            print '\t\t21) transit bowl'
            print '\t\t22) pour ingredient'
            print '\t\t23) return the empty bowl'
            print '\t\t24) rotate the hand to the post grasp'

            print '\n\tCombo steps:'
            print '\t\t30) clear collision map, move arms to side, load detectedobj.log, deal all ingred'
            print '\t\t31) clear collision map, move arms to side, load detectedobj.log, SMACH'

            print '\n\tSMACH steps:'
            print '\t\t40) grab ingrient with SMACH'

            print '\n\tSystem:'
            print '\t\t100) quit'
            print '\t\t95) save all detected objects to a file'
            print '\t\t96) load all detected objects from a file'
            print '\t\t97) attempt to reload sid'
            print '\t\t98) arm client ui'
            print '\t\t99) python interpreter'

            choice = 0
            try:
                choice = int(raw_input('\n\n\t\t\tchoice: '))
            except Exception as e:
                print 'invalid input'
                continue

            if choice == 100:
                return
            elif choice == 1:
                self.sid.move_both_arms_to_side()
            elif choice == 2:
                self.sid.torso_lifter.up()
            elif choice == 5:
                self.sid.pick_and_place_manager.find_table()
            elif choice == 6:
                self.sid.pick_and_place_manager.find_table
                (self.sid.detected_objects, self.sid.table) = self.sid.pick_and_place_manager.call_tabletop_detection(take_static_collision_map = 1, update_table = 1, clear_attached_objects = 1)
                print self.sid.detected_objects
                print self.sid.table
            elif choice == 7:
                (self.sid.detected_ingredients, self.mixing_bowl) = self.sid.filter_ingredients(self.sid.detected_objects)
            elif choice == 8:
                self.sid.pick_and_place_manager.reset_collision_map();
            elif choice == 9:
                self.sid.pick_and_place_manager.take_static_map();
            elif choice == 10:
                self.sid.scan_tabletop()
            elif choice == 11:
                for i, ingred in enumerate(self.sid.detected_ingredients):
                    print i, ingred.collision_name, ingred.pose.pose.position.x, ingred.pose.pose.position.y, 'cluster =', (ingred.object.type == GraspableObject.POINT_CLUSTER)
                try: 
                    choice = int(raw_input('\n\n\t\t\tingred choice: '))
                    print choice
                    print self.sid.detected_ingredients
                    print self.sid.detected_ingredients[choice]
                    self.sid.deal_ingredient(self.sid.detected_ingredients[choice])    
                except Exception as e:
                    print 'caught exception!'
                    print str(e)
            elif choice == 12:
                self.sid.deal_all_ingredients()
            elif choice == 20:
                for i, ingred in enumerate(self.sid.detected_ingredients):
                    print i, ingred.collision_name, ingred.pose.pose.position.x, ingred.pose.pose.position.y, (ingred.object.type == GraspableObject.POINT_CLUSTER)
                try: 
                    choice = int(raw_input('\n\n\t\t\tingred choice: '))
                    print choice
                    print self.sid.detected_ingredients
                    print self.sid.detected_ingredients[choice]
                    self.sid.grab_ingredient(self.sid.detected_ingredients[choice])    
                except Exception as e:
                    print 'caught exception!'
                    print str(e)

            elif choice == 21:
                self.sid.transit_bowl(True, self.sid.pre_pour_position)
            elif choice == 22:
                self.sid.pour_ingredient(True)
            elif choice == 23:
                self.sid.return_empty_bowl(True)
            elif choice == 24:
                print 'trying to rotate'
                print self.sid.arm_client.rotate_end_effector(True, .7, 0, -.7, 0)

            elif choice == 30:
                self.sid.pick_and_place_manager.reset_collision_map()
                self.sid.move_both_arms_to_side()
                myfile = open('detectedobj.log', 'rb')
                (self.sid.broad_object_manager.detected_objects, self.sid.pick_and_place_manager.table_height)  = pickle.load(myfile)
                print 'saved objects loaded from detectedobj.log'
                for obj in self.sid.broad_object_manager.detected_objects:
                    print obj.collision_name
                myfile.close()
                self.sid.pick_and_place_manager.detected_objects = self.sid.broad_object_manager.detected_objects
                (detected, mixing) = self.sid.filter_ingredients(self.sid.pick_and_place_manager.detected_objects)
                self.sid.detected_ingredients = detected
                self.sid.mixing_bowl = mixing
                self.sid.deal_all_ingredients()

            elif choice == 31:
                self.sid.pick_and_place_manager.reset_collision_map()
                self.sid.move_both_arms_to_side()
                myfile = open('detectedobj.log', 'rb')
                (self.sid.broad_object_manager.detected_objects, self.sid.pick_and_place_manager.table_height)  = pickle.load(myfile)
                print 'saved objects loaded from detectedobj.log'
                for obj in self.sid.broad_object_manager.detected_objects:
                    print obj.collision_name
                myfile.close()
                self.sid.pick_and_place_manager.detected_objects = self.sid.broad_object_manager.detected_objects
                (detected, mixing) = self.sid.filter_ingredients(self.sid.pick_and_place_manager.detected_objects)
                self.sid.detected_ingredients = detected
                self.sid.mixing_bowl = mixing
                for i, ingred in enumerate(self.sid.detected_ingredients):
                    print i, ingred.collision_name, ingred.pose.pose.position.x, ingred.pose.pose.position.y, (ingred.object.type == GraspableObject.POINT_CLUSTER)
                #try: 
                choice = 0
                print 'choice = ', choice
                print self.sid.detected_ingredients
                print self.sid.detected_ingredients[choice]
                ingred = self.sid.detected_ingredients[choice]
                ingred_name = ingred.collision_name
                ingred_pose = ingred.pose
                mixing_bowl_pose = self.sid.mixing_bowl.pose.pose.position
                mbpx = mixing_bowl_pose.x
                mbpy = mixing_bowl_pose.y
                mbpz = mixing_bowl_pose.z
                mixing_bowl_pose = (mbpx, mbpy, mbpz)
                bowl_dealer.main(ingred_name, ingred_pose, arms_to_try = (0, 1), mixing_bowl_centroid = mixing_bowl_pose)
                self.sid.pick_and_place_manager.open_gripper(0)
                self.sid.pick_and_place_manager.open_gripper(1)
                self.sid.pick_and_place_manager.reset_collision_map()
                self.sid.move_both_arms_to_side()
                print '*(((((((((((((((((((((((((((((((((((((((((  DOING NEXT BOWL*********************************'
                ingred = self.sid.detected_ingredients[1]
                ingred_name = ingred.collision_name
                ingred_pose = ingred.pose
                bowl_dealer.main(ingred_name, ingred_pose, arms_to_try = (0, 1), mixing_bowl_centroid = mixing_bowl_pose)
                self.sid.pick_and_place_manager.open_gripper(0)
                self.sid.pick_and_place_manager.open_gripper(1)
                self.sid.pick_and_place_manager.reset_collision_map()
                self.sid.move_both_arms_to_side()
                print '*(((((((((((((((((((((((((((((((((((((((((  DOING NEXT BOWL*********************************'
                ingred = self.sid.detected_ingredients[2]
                ingred_name = ingred.collision_name
                ingred_pose = ingred.pose
                bowl_dealer.main(ingred_name, ingred_pose, arms_to_try = (0, 1), mixing_bowl_centroid = mixing_bowl_pose)
                self.sid.pick_and_place_manager.open_gripper(0)
                self.sid.pick_and_place_manager.open_gripper(1)
                self.sid.pick_and_place_manager.reset_collision_map()
                self.sid.move_both_arms_to_side()
                print '*(((((((((((((((((((((((((((((((((((((((((  DOING NEXT BOWL*********************************'
                ingred = self.sid.detected_ingredients[3]
                ingred_name = ingred.collision_name
                ingred_pose = ingred.pose
                bowl_dealer.main(ingred_name, ingred_pose, arms_to_try = (0, 1), mixing_bowl_centroid = mixing_bowl_pose)
                self.sid.pick_and_place_manager.open_gripper(0)
                self.sid.pick_and_place_manager.open_gripper(1)
                self.sid.pick_and_place_manager.reset_collision_map()
                self.sid.move_both_arms_to_side()

            elif choice == 40:
                for i, ingred in enumerate(self.sid.detected_ingredients):
                    print i, ingred.collision_name, ingred.pose.pose.position.x, ingred.pose.pose.position.y, (ingred.object.type == GraspableObject.POINT_CLUSTER)
                #try: 
                choice = int(raw_input('\n\n\t\t\tingred choice: '))
                print choice
                print self.sid.detected_ingredients
                print self.sid.detected_ingredients[choice]
                ingred = self.sid.detected_ingredients[choice]
                ingred_name = ingred.collision_name
                ingred_pose = ingred.pose
                mixing_bowl_pose = self.sid.mixing_bowl.pose.pose.position
                mbpx = mixing_bowl_pose.x
                mbpy = mixing_bowl_pose.y
                mbpz = mixing_bowl_pose.z
                mixing_bowl_pose = (mbpx, mbpy, mbpz)
                bowl_dealer.main(ingred_name, ingred_pose, arms_to_try = (0, 0), mixing_bowl_centroid = mixing_bowl_pose)
                #except Exception as e:
                #print 'caught exception!'
                #print str(e)

            elif choice == 95:
                print 'saving detected objects to a file'
                filename = raw_input('enter filename: ')
                myfile = open(filename, 'wb')
                pickle.dump((self.sid.broad_object_manager.detected_objects, self.sid.pick_and_place_manager.table_height), myfile)
                myfile.close()
                print 'file written to ' + filename
            elif choice == 96:
                filename = raw_input('enter filename: ')
                myfile = open(filename, 'rb')
                (self.sid.broad_object_manager.detected_objects, self.sid.pick_and_place_manager.table_height)  = pickle.load(myfile)
                print 'saved objects loaded from ' + filename
                for obj in self.sid.broad_object_manager.detected_objects:
                    print obj.collision_name
                myfile.close()
                self.sid.pick_and_place_manager.detected_objects = self.sid.broad_object_manager.detected_objects
                (detected, mixing) = self.sid.filter_ingredients(self.sid.pick_and_place_manager.detected_objects)
                self.sid.detected_ingredients = detected
                self.sid.mixing_bowl = mixing

            elif choice == 97:
                print 'NOT attempting to reload sid'
                #from sequential_ingredient_dealer import SequentialIngredientDealer
                #self.sid = SequentialIngredientDealer()
            elif choice == 98:
                print 'going to arm client ui, quit to return to sid tester...'
                acu = ArmClientUI(self.sid.arm_client)
                acu.ui_loop()
            elif choice == 99:
                print 'python interpreter (input invalid anythin to return)'
                while True:
                    try:
                        print input('> ')
                    except Exception as e:
                        print 'invalid input'
                        print e
                        break
            else:
                print 'out of range'
                continue
    
if __name__ == '__main__':
    rospy.init_node('sequential_ingredient_dealer', anonymous=True)
    sidt = SIDTest()
    sidt.ui()
