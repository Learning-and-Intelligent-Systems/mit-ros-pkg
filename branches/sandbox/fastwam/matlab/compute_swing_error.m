function [err, err_xyz, err_normal, err_vel] = compute_swing_error(Q,T,B, plotting)
%[err, err_xyz, err_normal, err_vel] = compute_swing_error(Q,T,B, plotting)

if nargin < 4
    plotting = 0;
end


% 20131118/20131119: desired paddle trajectories in this experiment have an offset of [-.04, 0, -.05]
dt = .02;
a = 2;
v = a*dt*(1:25);
%des_paddle_pos = [dt*cumsum(v) - .3; repmat(B(2),[1,25]); repmat(B(3)-.05,[1,25])]';
des_paddle_pos = [dt*cumsum(v) - .26; repmat(B(2),[1,25]); B(3) + .2*dt*cumsum(v) - .05]';
des_paddle_normal = [sqrt(.9), 0, -sqrt(.1)];
des_paddle_vel = [1, 0, 0];

% get actual paddle trajectory
paddle_pos = [];
paddle_normal = [];
paddle_vel = [];
idx = 10:10:250;
for i=1:length(idx)
    qi = idx(i);
    paddle_pos(i,:) = arm_kinematics(Q(qi,:),7,[0;0;.1],1)';
    paddle_normal(i,:) = arm_kinematics(Q(qi,:),7,[1;0;.1],1)' - paddle_pos(i,:);
    
    p0 = arm_kinematics(Q(qi-5,:),7,[0;0;.1],1)';
    p1 = arm_kinematics(Q(qi+5,:),7,[0;0;.1],1)';
    paddle_vel(i,:) = (p1-p0)/dt;
end

paddle_pos_final_err = norm(paddle_pos(end,:) - des_paddle_pos(end,:));
paddle_pos_normal_err = norm(paddle_normal(end,:) - des_paddle_normal(end,:));

err = paddle_pos_final_err; % + .1*paddle_pos_normal_err;
err_xyz = paddle_pos(end,:) - des_paddle_pos(end,:);
err_normal = norm(paddle_normal(end,:) - des_paddle_normal(end,:));
err_vel = paddle_vel(end,:) - des_paddle_vel(end,:);

if plotting
    figure(4); plot(des_paddle_pos); hold on, plot(paddle_pos, '--'); hold off, legend('x', 'y', 'z');
end
