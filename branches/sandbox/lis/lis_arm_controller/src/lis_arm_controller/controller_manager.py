#! /usr/bin/python
import roslib
roslib.load_manifest('lis_arm_controller')
import rospy

# import msgs and srvs
from pr2_mechanism_msgs.srv import SwitchController, LoadController, UnloadController, ListControllers
from simple_Jtranspose_controller.srv import CheckMoving, MoveToPose
from std_srvs.srv import Empty

# import libraries
import time

# import utitlies
import set_joint_params
import set_cartesian_params

class ControllerManager():
    def __init__(self,whicharm,using_slip_controller):
        self.whicharm = whicharm
        self.using_slip_controller = using_slip_controller
        load_controller_serv_name = 'pr2_controller_manager/load_controller'
        unload_controller_serv_name = 'pr2_controller_manager/unload_controller'
        switch_controller_serv_name = 'pr2_controller_manager/switch_controller'
        list_controllers_serv_name = 'pr2_controller_manager/list_controllers'
        rospy.wait_for_service(load_controller_serv_name)
        rospy.wait_for_service(unload_controller_serv_name)
        rospy.wait_for_service(switch_controller_serv_name)
        rospy.wait_for_service(list_controllers_serv_name)
        self.load_controller_service = rospy.ServiceProxy(load_controller_serv_name, LoadController)
        self.unload_controller_service = rospy.ServiceProxy(unload_controller_serv_name, UnloadController)
        self.switch_controller_service = rospy.ServiceProxy(switch_controller_serv_name, SwitchController)
        self.list_controllers_service = rospy.ServiceProxy(list_controllers_serv_name, ListControllers)

        # names of the controllers
        self.joint_controller_name = self.whicharm+'_arm_controller'
        self.cartesian_controllers_name = [self.whicharm+x for x in ['_arm_cartesian_pose_controller', '_arm_cartesian_trajectory_controller']]
        if self.using_slip_controller: self.gripper_controller_name = self.whicharm+'_gripper_fingersensor_controller'
        else: self.gripper_controller_name = self.whicharm+'_gripper_controller'

        # parameters
        self.cart_params = set_cartesian_params.JTCartesianParams(self.whicharm)  
        self.joint_params = set_joint_params.JointParams(self.whicharm)

        # load the Cartesian controllers if not already loaded
        rospy.loginfo("loading any unloaded Cartesian controllers")
        self.load_cartesian_controllers()
        time.sleep(2)

        # controller states
        self.gripper_controller_state = ''
        self.joint_controller_state = ''
        self.cartesian_controllers_state = []

        # services for the J-transpose Cartesian controller
        cartesian_check_moving_name = whicharm+'_arm_cartesian_trajectory_controller/check_moving'
        cartesian_move_to_name = whicharm+'_arm_cartesian_trajectory_controller/move_to'
        cartesian_preempt_name = whicharm+'_arm_cartesian_trajectory_controller/preempt'
        rospy.wait_for_service(cartesian_check_moving_name)
        rospy.wait_for_service(cartesian_move_to_name)
        rospy.wait_for_service(cartesian_preempt_name)
        self.cartesian_moving_service = rospy.ServiceProxy(cartesian_check_moving_name, CheckMoving)
        self.cartesian_cmd_service = rospy.ServiceProxy(cartesian_move_to_name, MoveToPose)
        self.cartesian_preempt_service = rospy.ServiceProxy(cartesian_preempt_name, Empty)

        # re-load the Cartesian controllers with the gentle params
        self.cart_params.set_params_to_gentle()
        self.reload_cartesian_controllers()

    # make sure the appropriate controllers (joint or Cartesian) are running/stopped
    # mode is 'joint' or 'cartesian'
    def check_controllers_ok(self, mode):
        modewrong = 0
        if mode == 'joint':
            self.check_controller_states()
            if not self.joint_controller_state == 'running' and any([x == 'running' for x in self.cartesian_controllers_state]):
                self.switch_to_joint_mode()
            elif not self.joint_controller_state == 'running':
                rospy.logwarn("joint controllers aren't running!  Attempting to start")
                self.start_joint_controllers()
            elif any([x == 'running' for x in self.cartesian_controllers_state]):
                rospy.logwarn("Cartesian controllers are running!  Attempting to stop")
                self.stop_controllers(stop_cartesian = 1)
        elif mode == 'cartesian':
            self.check_controller_states()
            if not all([x == 'running' for x in self.cartesian_controllers_state]) and \
                    self.joint_controller_state == 'running':
                self.switch_to_cartesian_mode()
            elif not all([x == 'running' for x in self.cartesian_controllers_state]):
                rospy.logwarn("Cartesian controllers aren't running!  Attempting to start")
                self.start_cartesian_controllers()
            elif self.joint_controller_state == 'running':
                rospy.logwarn("joint controllers are running!  Attempting to stop")
                self.stop_controllers(stop_joint = 1)

    #figure out which controllers are loaded/started (fills in self.joint_controller_state, self.cartesian_controllers_state, and self.gripper_controller_state with 'not loaded', 'stopped', or 'running'
    def check_controller_states(self):
        self.gripper_controller_state = ''
        self.joint_controller_state = ''
        self.cartesian_controllers_state = []
        resp = self.list_controllers_service()
        if self.gripper_controller_name in resp.controllers: self.gripper_controller_state = resp.state[resp.controllers.index(self.gripper_controller_name)]
        else: self.gripper_controller_state = 'not loaded'
        
        if self.joint_controller_name in resp.controllers: self.joint_controller_state = resp.state[resp.controllers.index(self.joint_controller_name)]
        else: self.joint_controller_state = 'not loaded'

        for controller in self.cartesian_controllers_name:
            if controller not in resp.controllers: self.cartesian_controllers_state.append('not loaded')
            else: self.cartesian_controllers_state.append(resp.state[resp.controllers.index(controller)])

    #load the Cartesian controllers with the current set of params on the param server
    def load_cartesian_controllers(self):
        self.check_controller_states()
        for (controller, state) in zip(self.cartesian_controllers_name, self.cartesian_controllers_state):
            if state == 'not loaded':
                success = self.load_controller_service(controller)
                if not success:
                    rospy.logerr("error in loading Cartesian controller!")
                    break  
        else:
            rospy.loginfo("all Cartesian controllers loaded")
        self.check_controller_states()

    #unload the Cartesian controllers (to re-load with new params)
    def unload_cartesian_controllers(self):
        self.check_controller_states()
        if any([x=='running' for x in self.cartesian_controllers_state]): self.stop_controllers(stop_cartesian = 1)
        if all([x=='not loaded' for x in self.cartesian_controllers_state]): return

        cart_controllers_reversed = self.cartesian_controllers_name[:]
        cart_controllers_reversed.reverse()
        state_reversed = self.cartesian_controllers_state[:]
        state_reversed.reverse()
        for (controller, state) in zip(cart_controllers_reversed, state_reversed):
            if state == 'stopped':
                success = self.unload_controller_service(controller)
                if not success:
                    rospy.logerr("error in unloading Cartesian controller!")
                    break
        else:
            rospy.loginfo("Cartesian controllers unloaded")
        self.check_controller_states()

    #load the joint controller with the current set of params on the param server
    def load_joint_controllers(self):
        self.check_controller_states()
        if self.joint_controller_state == 'not loaded':
            success = self.load_controller_service(self.joint_controller_name)
            if not success:
                rospy.logerr("error in loading joint controller!")
        else:
            rospy.loginfo("joint controller already loaded")
        self.check_controller_states()

    #unload the joint controller 
    def unload_joint_controllers(self):
        self.check_controller_states()
        if self.joint_controller_state == 'running': self.stop_controllers(stop_joint = 1)
        elif self.joint_controller_state == 'not loaded': return
        success = self.unload_controller_service(self.joint_controller_name)
        if not success:
            rospy.logerr("error in unloading joint controller!")
        self.check_controller_states()

    # re-load the Cartesian controllers with new parameters (change cart_params before running)
    def reload_cartesian_controllers(self, set_params = 1):
        self.check_controller_states()
        # set the new params on the parameter server
        if set_params: self.cart_params.set_params()
        # if we're running the Cartesian controllers, let the joint controllers hold while we switch
        restart = 0
        if any([x=='running' for x in self.cartesian_controllers_state]): 
            restart = 1
            self.switch_to_joint_mode()
        self.unload_cartesian_controllers()
        self.load_cartesian_controllers()
        # switch back from joint to cartesian
        if restart: self.switch_to_cartesian_mode()
        self.check_controller_states()

    # re-load the joint controller with gains a fraction of the defaults
    def reload_joint_controllers(self, fraction = 1.):
        self.check_controller_states()
        # set the new params on the parameter server
        self.joint_params.set_gains_fraction(fraction)
        # if we're running the joint controllers, let the Cartesian controllers hold while we switch
        restart = 0
        if self.joint_controller_state == 'running':
            restart = 1
            self.switch_to_cartesian_mode()
        self.unload_joint_controllers()
        self.load_joint_controllers()
        # switch back from Cartesian to joint
        if restart: self.switch_to_joint_mode()
        self.check_controller_states()
    
    # stops the joint controllers, starts the Cartesian ones (both need to be loaded already)
    def switch_to_cartesian_mode(self):
        self.check_controller_states()
        if all([x=='stopped' for x in self.cartesian_controllers_state]) and self.joint_controller_state == 'running':
            success = self.switch_controller_service(self.cartesian_controllers_name, [self.joint_controller_name,], 2)
            if success:
                rospy.loginfo("switched joint to Cartesian successfully")
            else:
                rospy.logerr("switching joint to Cartesian failed")
        else:
            self.start_cartesian_controllers()
            self.stop_controllers(stop_joint = 1)
        self.check_controller_states()

    # stops the Cartesian controllers, starts the joint ones (both need to be loaded already)
    def switch_to_joint_mode(self):
        self.check_controller_states()
        if self.joint_controller_state == 'stopped' and any([x=='running' for x in self.cartesian_controllers_state]):
            success = self.switch_controller_service([self.joint_controller_name,], self.cartesian_controllers_name, 2)
            if success:
                rospy.loginfo("switched Cartesian to joint successfully")
            else:
                rospy.logerr("switching Cartesian to joint failed")
        else:
            self.start_joint_controllers()
            self.stop_controllers(stop_cartesian = 1)
        self.check_controller_states()

    # just start the joint controllers (need to be loaded already)
    def start_joint_controllers(self):
        self.check_controller_states()
        if self.joint_controller_state == 'stopped':
            success = self.switch_controller_service([self.joint_controller_name,], [], 2)
            if success:
                rospy.loginfo("started joint controller successfully")
            else:
                rospy.logerr("starting joint controller failed")
        elif self.joint_controller_state == 'not loaded':
            rospy.logerr("joint controller not loaded!")
        self.check_controller_states()

    # start the Cartesian controllers (need to be loaded already)
    def start_cartesian_controllers(self):
        self.check_controller_states()
        if any([x=='not loaded' for x in self.cartesian_controllers_state]):
            print "not all Cartesian controllers are loaded!"
        elif any([x=='stopped' for x in self.cartesian_controllers_state]):
            success = self.switch_controller_service(self.cartesian_controllers_name, [], 2)
            if success:
                rospy.loginfo("started Cartesian controllers successfully")
            else:
                rospy.logerr("starting Cartesian controllers failed")
        self.check_controller_states()

    # start the gripper controller (needs to be loaded already)
    def start_gripper_controller(self):
        self.check_controller_states()
        if self.gripper_controller_state == 'stopped':
            success = self.switch_controller_service([self.gripper_controller_name,], [], 2)
            if success:
                rospy.loginfo("started gripper controller successfully")
            else:
                rospy.logerr("starting gripper controller failed")
        elif self.gripper_controller_state == 'not loaded':
            rospy.logerr("gripper controller isn't loaded!")
        self.check_controller_states()

    # stop all controllers
    def stop_all_controllers(self):
        self.stop_controllers(stop_joint = 1, stop_cartesian = 1, stop_gripper = 1)

    # stop controllers that are currently running
    def stop_controllers(self, stop_joint = 0, stop_cartesian = 0, stop_gripper = 0):
        self.check_controller_states()
        # stop Cartesian controllers
        if stop_cartesian and any([x=='running' for x in self.cartesian_controllers_state]):
            success = self.switch_controller_service([], self.cartesian_controllers_name, 2)
            if success:
                rospy.loginfo("stopped Cartesian controllers successfully")
            else:
                rospy.logerr("stopping Cartesian controllers failed")
        # stop joint controllers
        if stop_joint and self.joint_controller_state == 'running':
            success = self.switch_controller_service([], [self.joint_controller_name,], 2)
            if success:
                rospy.loginfo("stopped joint controller successfully")
            else:
                rospy.logerr("stopping joint controller failed")
        # stop gripper controller
        if stop_gripper and self.gripper_controller_state == 'running':
            success = self.switch_controller_service([], [self.gripper_controller_name,], 2)
            if success:
                rospy.loginfo("stopped gripper controller successfully")
            else:
                rospy.logerr("stopping gripper controller failed")
        self.check_controller_states()
