import objModels as om
import DrawingWindowStandalone as dw
import windowManager as wm
import util
import random
import math
import itertools
import physics_sim as ps

class trajectory:
    def __init__(self, start, goal, size, theta):
        self.start = start
        self.goal = goal
        self.size = size
        self.theta = theta
        self.good = self.good()        

    def discretize(self, step):
        path = []
        x = (self.goal[0] - self.start[0])/step
        y = (self.goal[1] - self.start[1])/step
        z = (self.goal[2] - self.start[2])/step
        for i in range(step):
            point = (self.start[0]+(x*i), self.start[1]+(y*i), self.start[2]+(z*i))
            path.append(point)
        path.append(self.goal)
        return path

    def collides(self, obstacles):
        pts = self.discretize(4)
        for pt in pts:
            traj_pt = om.Box(self.size[0], self.size[1], self.size[2], name='traj_pt', \
                                 pose=util.Pose(pt[0], pt[1], pt[2], self.theta))
            for obs in obstacles:
                if (traj_pt.collides(obs)):
                    return True
        return False

    def get_verts(self):
        return [(0.0,0.0,0.0), (self.size[0],0.0,0.0), \
                    (0.0,self.size[1],0.0), (self.size[0],self.size[1],0.0), \
                    (0.0,0.0,self.size[2]), (self.size[0],0.0,self.size[2]), \
                    (0.0,self.size[1],self.size[2]), \
                    (self.size[0],self.size[1],self.size[2])]

    def good(self):
        m = abs(self.slope())
        (x_theta, y_theta) = self.angle()
        max_x_theta = ps.max_angle(self.get_verts(),"+x")
        max_y_theta = ps.max_angle(self.get_verts(),"+y")
        print "x="+str(x_theta)+" y="+str(y_theta)
        print "max_x="+str(max_x_theta)+" max_y="+str(max_y_theta)
        max_x_theta = (math.pi/2) - max_x_theta
        max_y_theta = (math.pi/2) - max_y_theta
        #print "max_x="+str(max_x_theta*(180/math.pi))+" x="+str(abs(x_theta*(180/math.pi)))
        #print "max_y="+str(max_y_theta*(180/math.pi))+" y="+str(abs(y_theta*(180/math.pi)))
        if (abs(x_theta) > max_x_theta) or (abs(y_theta) > max_y_theta):
            return False
        return True

        # threshold = 0.9
        # if (abs(self.slope()) < threshold):
        #     return False
        # else:
        #     return True

    def angle(self):
        vert = [0.0,0.0,1.0]
        vec = [self.goal[0]-self.start[0], \
                   self.goal[1]-self.start[1], \
                   self.goal[2]-self.start[2]]
        x_angle = math.atan(vec[0]/vec[2])
        y_angle = math.atan(vec[1]/vec[2])
        return (x_angle, y_angle)

    def slope(self):
        vert = [0.0,0.0,1.0]
        vec = [self.goal[0]-self.start[0], \
                   self.goal[1]-self.start[1], \
                   self.goal[2]-self.start[2]]
        mag = math.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
        unit_vec = [vec[0]/mag, vec[1]/mag, vec[2]/mag]
        dot_prod = sum(x*y for x,y in zip(vert, unit_vec))
        #print "dot_prod="+str(dot_prod)
        return dot_prod
    
    def draw_dots(self,window):
        window.drawOval(((self.goal[0]-0.05),(self.goal[1]-0.05)), ((self.goal[0]+0.05),(self.goal[1]+0.05)), color='blue')
        

    def draw(self, window):
        #window.drawPoint(self.goal[0],self.goal[1], color="blue")
        # window.drawRect(((self.goal[0]-self.size[0]/2),(self.goal[1]-self.size[1]/2)),((self.goal[0]+self.size[0]/2), (self.goal[1]+self.size[1]/2)), color='yellow')
        #window.drawOval(((self.goal[0]-0.05),(self.goal[1]-0.05)), ((self.goal[0]+0.05),(self.goal[1]+0.05)), color='blue')


        box = om.Box(self.size[0], self.size[1], self.size[2], name='traj', \
                         pose=util.Pose(self.goal[0], self.goal[1], 0, \
                                            self.theta))
        box.draw(window)
        

        # dim = [0.1, 0.1, (self.goal[2] - self.start[2])]
        # box_pose = [self.goal[0], self.goal[1], self.goal[2], 0] #angle not right
        # box = om.Box(dim[0], dim[1], dim[2], name="traj", \
        #                  pose=util.Pose(box_pose[0], box_pose[1], box_pose[2], box_pose[3]))
        # box.draw(window)

def drange(start, stop, step):
    r = start
    while r < stop:
        yield r
        r += step

class object:
    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max, pose):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max
        self.pose = pose
        
    def make_box():
        x_dim = self.x_max - self.x_min
        y_dim = self.y_max - self.y_min
        z_dim = self.z_max - self.z_min
        obs = om.Box(x_dim, y_dim, z_dim, name='obj', pose=self.pose, \
                         color='red', opacity=0.5) #opacity not working
 
    def get_verts():
        return (self.x_min, self.x_max, self.y_min, \
                    self.y_max, self.z_min, self.z_max)
    
    def get_angle():
        return self.pose.theta

class trajectory_set:
    def __init__(self, start, size):
        self.start = start
        self.goals = []
        self.trajectories = []
        self.size = size

    def add_traj(self, goal, theta):
        self.trajectories.append(trajectory(self.start, goal, self.size, theta))
        self.goals.append(goal)

    def create_set(self, x_min, x_max, y_min, y_max):
        for x in drange(x_min, x_max, 0.2):
            for y in drange(y_min, y_max, 0.2): 
                #for theta in drange(0, (2*3.14), .5):
                self.add_traj((x,y,0), 0)

class world:
    def __init__(self, x_min, x_max, y_min, y_max):
        self.obstacles = []
        self.draw_obstacles = []
        self.trajectories = []
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

    def draw(self):
        window = wm.makeWindow('World', viewPort=(-3, 4, -3, 4, 0, 0), 
                               windowWidth=800)
        #window = dw.DrawingWindow(800,800,-3,4,-3,4,'World')

        print "obs="+str(len(self.draw_obstacles))
        for obs in self.obstacles:
            obs.draw(window) 
        for traj in self.trajectories:
            collision = traj.collides(self.obstacles)
            if not traj.good and not collision:
                pass
                #traj.draw(window) 
            elif traj.good and not collision:
                #pass
                traj.draw(window)
            elif not traj.good and collision:
                pass
                #traj.draw(window) 
            elif traj.good and collision:
                pass
                #traj.draw(window) 


        # for obs in self.draw_obstacles:
        #     print "obs="+str(obs)
        #     window.drawRect(obs[0],obs[1], color="red")

         # for traj in self.trajectories:
         #    collision = traj.collides(self.obstacles)
         #    if not traj.good and not collision:
         #        #pass
         #        traj.draw_dots(window)
         #    elif traj.good and not collision:
         #        pass
         #        #traj.draw_dots(window)
         #    elif not traj.good and collision:
         #        pass
         #        #traj.draw_dots(window) 
         #    elif traj.good and collision:
         #        pass
         #        #traj.draw_dots(window) 

    def add_obs(self, size, pose):
        obs_x = (pose.x-(size[0]/2.0), pose.y-(size[1]/2.0))
        obs_y = (pose.x+(size[1]/2.0), pose.y+(size[1]/2.0))
        self.draw_obstacles.append((obs_x,obs_y))
        
        obs = om.Box(size[0], size[1], size[2], name='new', pose=pose, \
                         color='red', opacity=0.5) #opacity not working
        self.obstacles.append(obs)

    def remove_obs(self):
        self.obstacles = []

    def move_obs(self, obs, new_pose):
        obs.applyPose(new_pose)

    def add_traj(self, traj):
        self.trajectories.append(traj)

    def remove_traj(self, traj):
        for trajectory in self.trajectories:
            if (traj == trajectory):
                self.trajectories.remove(trajectory)

    def check_good_traj(self, traj_set, good_set):
        for traj in traj_set:
            if not traj.good():
                if not traj.collides(self.obstacles):
                    return False
        return True

    def allows_bad(self):
        bad = 0
        good = 0
        total = 0
        for traj in self.trajectories:
            collides = traj.collides(self.obstacles)
            if not traj.good:
                total = total+1
                if not collides:
                    bad = bad+1
            else:
                if not collides:
                    good = good+1
        #print "bad="+str(bad)+" good="+str(good)
        #print "len="+str(len(self.obstacles))
        if (good == 0):
            return 1
        else:
            #print str(bad)+"/"+str(total)
            return bad / float(total)

    def pose_equal(self, first, bad):
        for second in bad:
            if (abs(first[0][0] - second[0][0])<0.00000001) and \
                    (abs(first[0][1] - second[0][1])<0.00000001) and \
                    (abs(first[1][0] - second[1][0])<0.00000001) and \
                    (abs(first[1][1] - second[1][1])<0.00000001):
                # print "FIRST="+str(first)
                # print "SECOND="+str(second)
                return True
        return False

    def permute_gripper(self, bad_poses):
        obs_size = (0.05,0.05,0.05)
        best_combo = ()
        pos_positions = [(x,y) for x in drange(self.x_min, self.x_max,0.05) \
                             for y in drange(self.y_min, self.y_max,0.05)]

        for pos in pos_positions:
            if (pos[0]+(2*obs_size[0])) <= self.x_max:
                combo = (pos, (pos[0]+(2*obs_size[0]), pos[1]))
                self.add_obs(obs_size, util.Pose(pos[0], pos[1], 0, 0))
                self.add_obs(obs_size, util.Pose(pos[0]+(2*obs_size[0]), \
                                                     pos[1], 0, 0))
                if not self.pose_equal(combo, bad_poses):
                    metric = self.allows_bad()
                    #print "combo="+str(combo)+" , "+str(metric)
                    if (best_combo == ()):
                        best_combo = (combo, metric)
                    if (metric < best_combo[1]):
                        best_combo = (combo, metric)
                    # print "best="+str(best_combo[0])+' , '+str(best_combo[1])
                else:
                    print "NECK"
                self.remove_obs()
            if (pos[1]+(2*obs_size[0])) <= self.y_max:
                combo = (pos, (pos[0], pos[1]+(2*obs_size[0])))
                
                self.add_obs((1,1,1), util.Pose(pos[0], pos[1], 0, 0))
                self.add_obs((1,1,1), util.Pose(pos[0],pos[1]+(2*obs_size[0]), 0, 0))
                if not self.pose_equal(combo, bad_poses):
                    metric = self.allows_bad()
                    #print "combo="+str(combo)+" , "+str(metric)
                    if (best_combo == ()):
                        best_combo = (combo, metric)
                    if (metric < best_combo[1]):
                        best_combo = (combo, metric)
                    #print "best="+str(best_combo[0])+' , '+str(best_combo[1])
                else:
                    print "CRACKS"
                self.remove_obs()
        return best_combo

    def permute_obs(self, obs_num):
        pos_combos = [(x,y) for x in range(self.x_min, self.x_max) \
                          for y in range(self.y_min, self.y_max)]
        obs_combos = itertools.combinations(pos_combos, obs_num)
        best_combo = ()
        for combo in obs_combos:
            for i in range(obs_num):
                self.add_obs((1,1,1), util.Pose(combo[i][0], combo[i][1], 0, 0))
            metric = self.allows_bad()
            if (best_combo == ()):
                best_combo = (combo, metric)
            if (metric < best_combo[1]):
                best_combo = (combo, metric)
            print "best="+str(best_combo[0])+' , '+str(best_combo[1])
            self.remove_obs()
        return best_combo    
        
def main():
    #w = world(-1,2,-1,2)
    w = world(-2,3,-2,3)
    #w.add_obs((0.1,0.1,0.1), util.Pose(0,0,0,0)) # origin
    #w.add_obs((0.1,0.1,0.1), util.Pose(1,0,0,0)) # reference pt
    cube = (1,1,1)

    traj_set = trajectory_set((0,0,1), (0.9,0.9,0.9))
    traj_set.create_set(w.x_min, w.x_max, w.y_min, w.y_max)
    for traj in traj_set.trajectories:
        w.add_traj(traj)

    w.add_obs(cube, util.Pose(-1,0,0,0))
    w.add_obs(cube, util.Pose(1,0,0,0))
    # w.add_obs(cube, util.Pose(0,-1,0,0))
    # w.add_obs(cube, util.Pose(0,1,0,0))
    # print w.allows_bad()
    w.draw()

    #print w.permute_gripper([])

def get_gripper(block_pose, world_x_min, world_x_max, \
                    world_y_min, world_y_max, bad):
    w = world(world_x_min, world_x_max, world_y_min, world_y_max)

    traj_set = trajectory_set((block_pose[0],block_pose[1],block_pose[2]), \
                                  (0.9,0.9,0.9))
    traj_set.create_set(w.x_min, w.x_max, w.y_min, w.y_max)
    for traj in traj_set.trajectories:
        w.add_traj(traj)
        
    #print "pose="+str(w.permute_gripper(bad))
    ((pose1, pose2),metric) = w.permute_gripper(bad)
    pose = ((pose1[0]+pose2[0])/2, (pose1[1]+pose2[1])/2)
    used = (pose1, pose2)
    return (pose, used)

main()
raw_input()




    # # surrounded on 4 sides. x=(-7.5, -6.5) y=(0.5,-0.5)
    # w.add_obs(cube, util.Pose(-8,0,0,0))
    # w.add_obs(cube, util.Pose(-7,1,0,0))
    # w.add_obs(cube, util.Pose(-6,0,0,0))
    # w.add_obs(cube, util.Pose(-7,-1,0,0))

    # # surrounded on 3 sides. x=(-2.5, ) y=(0.5,-0.5)
    # w.add_obs(cube, util.Pose(-3,0,0,0))
    # w.add_obs(cube, util.Pose(-2,1,0,0))
    # w.add_obs(cube, util.Pose(-2,-1,0,0))

    # # surrounded on 2 sides with a corner. x=(1.5, ) y=(0.5, )
    # w.add_obs(cube, util.Pose(1,0,0,0))
    # w.add_obs(cube, util.Pose(2,1,0,0))

    # # surrounded on 2 sides with no corner. x=( , ) y=(0.5,-0.5)
    # w.add_obs(cube, util.Pose(5,1,0,0))
    # w.add_obs(cube, util.Pose(5,-1,0,0))

    # w.draw()

    # traj_set = trajectory_set((-7,0,2))
    # traj_set.create_set(-8,-6, -2, 2)
    # for traj in traj_set.trajectories:
    #     w.add_traj(traj)
    # print("4-surround="+str(w.allows_bad())) # 93/293 = 31.7%
    
    # traj_set = trajectory_set((-2,0,2))
    # traj_set.create_set(-3,0, -2, 2)
    # for traj in traj_set.trajectories:
    #     w.add_traj(traj)
    # print("3-surround="+str(w.allows_bad())) # 142/293 = 48.5%
    
    # traj_set = trajectory_set((2,0,2))
    # traj_set.create_set(0,3, -2, 2)
    # for traj in traj_set.trajectories:
    #     w.add_traj(traj)
    # print("2-corner="+str(w.allows_bad())) # 192/293 = 65.5%
    
    # traj_set = trajectory_set((5,0,2))
    # traj_set.create_set(4,6, -2, 2)
    # for traj in traj_set.trajectories:
    #     w.add_traj(traj)
    # print("2-sandwich="+str(w.allows_bad())) # 191/293 = 65.1%





    
    # w.add_traj(trajectory((0,0,2),(0,0,0)))
    # print(w.collision_free())

    #move box
    #box = box.applyPose(util.Pose(1,1,1,(3.14/2)))
    #draw again
    #box.draw(window)

#    print_cluster(make_traj_cluster((0,0,2),(0,0,0)))
    # traj = make_trajectory((0,0,2),(0,0,0))
    # obs = create_obstacles(window, 10) #argument is num of obstacles
    # print(check_trajectory(traj, obs)) #True if no collisions (borders count)
