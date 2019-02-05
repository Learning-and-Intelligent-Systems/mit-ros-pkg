%function record_ball_hit(timeout)
timeout = 10;

paddle_width = .0127;
ball_radius = .02;
TABLE_WIDTH = 1.525;
pred_dt = .01;


% get the swing data and ball trajectory data
[swing, ball, pred] = record_swing_and_ball_trajectories(timeout);
Q = swing.Q;
U = swing.U;
T = swing.T;

% load swing plan
name = '~/.ros/swing_plan';
P_plan = read_matrix([name '_paddle.txt']);
Q_plan = read_matrix([name '_joints.txt']);
U_plan = read_matrix([name '_torques.txt']);
T_plan = read_matrix([name '_times.txt']);
n_plan = size(P_plan, 1) / 2;  %dbug

%i = find(traj_times > traj_times(1) + 1.5, 1);
%if ~isempty(i), traj_times = traj_times(1:i-1); end
%i = find(pred_times > pred_times(1) + 1.5, 1);
%if ~isempty(i), pred_times = pred_times(1:i-1); end

% draw the trajectories
draw_swing_and_ball_trajectories(swing, ball, pred);

% draw planned and recorded swings
t0 = T_plan(1);
P_swing = []; for i=1:size(Q,1), P_swing(i,:) = arm_kinematics(Q(i,:), 7, [0,0,.1], 1); end
figure(2); plot(T_plan-t0, P_plan(:,1:3)); hold on, plot(T-t0, P_swing, '--');
plot([.4 .4], [0,1], 'k-');  % desired hit time
hold off, legend('x','y','z');

% segment into incoming and outgoing ball trajectories
[~,i] = max(ball.T(2:end)-ball.T(1:end-1));
%traj_in = 
[~,pred_idx] = min(abs(pred.T - ball.T(i)));
%traj_out = traj{end};
%B_out = [traj_out(:,2), TABLE_WIDTH - traj_out(:,1), traj_out(:,3)];

%dbug
%yzero_pred = [];
%for i=1:pred_idx, yzero_pred(i) = ball.T(i)-t0 + .01*find(pred.traj{i}(:,2)<0,1); end
%figure(3); plot(yzero_pred);


while 1
    j = pred_idx;
    pred_in = pred.traj{j};
    pred_times_in = pred.T(j):pred_dt:pred.T(j)+1;

    % estimate the ball-paddle pre-collision time and state
    P = [];
    N = [];
    B = [];
    D = [];
    Q2 = [];
    for i=1:50
        % interpolate to find the paddle pose at time pred_times_in(i)
        t = pred_times_in(i);
        j = find(T>t,1);
        a = (T(j)-t) / (T(j)-T(j-1));
        q = a*Q(j-1,:) + (1-a)*Q(j,:);
        p = arm_kinematics(q, 7, [0,0,.1], 1);
        n = arm_kinematics(q, 7, [1,0,.1], 1) - p;
        b = [pred_in(i,2), TABLE_WIDTH - pred_in(i,1), pred_in(i,3)];
        paddle_signed_dist = (b - p')*n - paddle_width/2 - ball_radius;
        Q2(i,:) = q;
        P(i,:) = p';
        N(i,:) = n';
        B(i,:) = b;
        D(i) = paddle_signed_dist;
        if paddle_signed_dist < 0
            break
        end
    end
    if i>1
        break
    end
    pred_idx = pred_idx-1;  % if the start of the predicted trajectory is already after the ball collision, try an earlier prediction
end    

a = D(end) / (D(end) - D(end-1));
q_hit = a*Q2(end-1,:) + (1-a)*Q2(end,:);
p_hit = a*P(end-1,:) + (1-a)*P(end,:);
n_hit = a*N(end-1,:) + (1-a)*N(end,:);
b_hit = a*B(end-1,:) + (1-a)*B(end,:);
pv_hit = (P(end,:) - P(end-1,:)) / pred_dt;
bv_hit = (B(end,:) - B(end-1,:)) / pred_dt;
t_hit = a*pred_times_in(i-1) + (1-a)*pred_times_in(i);


% plot the ball contact
%figure(1); [a1,a2] = view(); draw_table
figure(1); hold on, draw_arm(q_hit); hold on, plot3(b_hit(1), b_hit(2), b_hit(3), 'bo'); hold off;
%axis equal, axis vis3d, view(a1,a2);
zlim([0,1]);


% estimate the ball state after collision
i1 = find(ball.T > t_hit, 1);
i2 = i1+30;
b = [ball.pos(i2,:), ball.vel(i2,:)];
B_out = b;
dt = pred_dt;
C = .1;
p = 1.204;
m = .0027;
g = 9.8;
r = .02;
A = pi*r^2;
t = ball.T(i2);
while t >= t_hit + dt/100
    dt = min(dt, t - t_hit);
    v = b(4:6);
    Fd = -.5*C*p*A*norm(v)*v;
    b = b - dt*[v, Fd/m - [0,0,g]];
    B_out = [b; B_out];
    t = t - dt;
end
figure(1); hold on, plot3(B_out(:,2), TABLE_WIDTH - B_out(:,1), B_out(:,3), 'r-'); hold off;
b_out = B_out(1,1:3);
bv_out = B_out(1,4:6);


figure(2); hold on, plot([t_hit t_hit] - t0, [0,1], 'k--'); hold off






