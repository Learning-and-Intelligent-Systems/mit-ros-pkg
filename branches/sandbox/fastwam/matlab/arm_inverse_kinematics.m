function JA = arm_inverse_kinematics(paddle_position, paddle_normal, JA0, joint_costs, plotting)


function d = paddle_dist_fn(joint_angles, normal_weight, plotting)

    if nargin < 2
        normal_weight = 0;
    end
    if nargin < 3
        plotting = 0;
    end
    
    P = arm_kinematics(joint_angles, 7, [0,0,.1], 1);
    N = arm_kinematics(joint_angles, 7, [1,0,.1], 1);
    N = (N-P)/norm(N-P);

    d = norm(paddle_position - P) + normal_weight*norm(paddle_normal - N);
    
    if plotting
        joint_angles
        pose = [P', N']
        [a1,a2] = view(); draw_table, hold on, draw_arm(joint_angles); view(a1,a2); drawnow;
    end
end

if nargin < 4 || isempty(joint_costs)
    cost_fn = @(JA) paddle_dist_fn(JA, .1, 0);
else
    cost_fn = @(JA) paddle_dist_fn(JA, .1, 0) + dot(joint_costs, JA.^2);
end

if nargin < 3 || isempty(JA0)
    JA0 = pi*rand(1,7) - pi/2;
end
JA = fminunc(cost_fn, JA0);

if nargin < 5 || plotting
    paddle_dist_fn(JA, .1, 1);
end

end
