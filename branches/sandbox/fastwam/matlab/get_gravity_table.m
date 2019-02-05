paddle_normal = [1,0,0]';
%ja0 = [0.3077, -0.2558, 0.3703, 2.0547, -2.0850, -0.9102, 0.1861];
ja0 = [0.2244, -0.5675, 0.8277, 2.1011, -1.8128, -0.5716, 0.5138];
p0 = arm_kinematics(ja0,7,[0;0;.1],1);
x0 = p0(1);

res = .04;
Y = .5:res:.9;
Z = .2:res:.4;

Q_table = [];
U_table = [];

for y=Y
    
    Qy = [];
    
    for z=Z
        
        Q = arm_inverse_kinematics([x0;y;z], paddle_normal, ja0);
        for x=x0+res:res:x0+.3
            Q(end+1,:) = arm_inverse_kinematics([x;y;z], paddle_normal, Q(end,:));
        end
        Q2 = Q(end:-1:1,:);
        
        Qy = [Qy; Q; Q2];
    end
    Q_table = [Q_table; Qy];
    
    U = get_gravity_torques_for_trajectory(Qy);
    U_table = [U_table; U];
    
    %Z = Z(end:-1:1);  % flip Z to avoid big jumps when y changes
end
