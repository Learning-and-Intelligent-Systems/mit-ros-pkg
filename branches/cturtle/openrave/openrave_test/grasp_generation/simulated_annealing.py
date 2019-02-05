from numpy import *
import time
from grasp_generation_test import *

class SA:
    def __init__(self, random_draw_function, evaluation_function, random_neighbor_function, temperature_function, kmax,emin=0):
        self.random_draw_function = random_draw_function
        self.evaluation_function = evaluation_function
        self.random_neighbor_function = random_neighbor_function
        self.temperature_function = temperature_function
        self.kmax = kmax
        self.emin = emin
        
    def p(self,a,b,T):
        if b<a:
            return 1
        else:
            return exp((a-b)/T)    
    
    def run(self):
        try:
            state = self.random_draw_function()
            energy = self.evaluation_function(state)
            k = 0
            best = None
            t = 1.0
            best_e = 1000
            history_k = []
            history_e = []
            history_t = []
            while k < self.kmax or best_e< self.emin:
                #if k%100==0:
                #    print 'k',k,'t',t,'e',energy
                state_new = self.random_neighbor_function(state)
                energy_new = self.evaluation_function(state_new)
                t = self.temperature_function(t)
                if energy_new < best_e:
                    best_state = state_new
                    best_e = energy_new
                    best_k = k
                    best_t = t
                if self.p(energy, energy_new, t) > random.uniform():
                    history_k.append(k)
                    history_t.append(t)
                    history_e.append(energy_new)
                    (state, energy) = (state_new, energy_new)
                    if energy<1:
                        print 'k',k,'t',t,'e',energy
                k += 1
            filename = 'log-'+str(time.time())
            file = open(filename,'w')
            print >> file,best_state.getTransform()
            print >> file,best_state.gripper_angle
            print >> file,best_e
            print >> file,best_k
            print >> file,best_t
            print >> file,targetfile
            print >> file,history_k
            print >> file,history_t
            print >> file,history_e
            print >> file,angle_k
            print >> file,kmax
            print >> file,alpha
            print >> file,gripper_angle_std
            print >> file,rotation_z_std
            print >> file,pos_x_std
            print >> file,pos_y_std
            print >> file,pos_z_std
        except openrave_exception, e:
            print e
        return best_state,best_e,best_k,best_t
    
def temperature_function_ecs(t):
    return alpha*t
