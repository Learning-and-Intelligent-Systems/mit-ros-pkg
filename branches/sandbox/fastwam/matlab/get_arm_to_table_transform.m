ja0 = [0.3077, -0.2558, 0.3703, 2.0547, -2.0850, -0.9102, 0.1861];
paddle_normal = [1,0,0]';

table_width = 1.525;

q = arm_inverse_kinematics([0; .75*table_width; .3], paddle_normal, ja0);
p = arm_kinematics(q,7,[0;0;.1],1)
n = arm_kinematics(q,7,[1;0;.1],1) - p

joint_swing(q, .01, paddle_normal);
