function ball_hit_params = get_hit_from_training_data(ball, pred, T_swing, P_swing, plotting, hit_model)
%ball_hit_params = get_hit_from_training_data(ball, pred, T_swing, P_swing, plotting, hit_model)

if nargin < 5
    plotting = 0;
end

ball_hit_params = [];

TABLE_WIDTH = 1.525;
dt = .01;
paddle_thickness = .0127;
ball_radius = .02;

% use the ball prediction from shortly before the ball bounces
vz = ball.vel(:,3);
i = find((vz(2:end)>0).*(vz(1:end-1)<0), 1) - 5;
if isempty(i)
    return
end
[~,pred_idx] = min(abs(pred.T - ball.T(i)));
pred_in = pred.traj{pred_idx};
pred_times_in = pred.T(pred_idx):dt:pred.T(pred_idx)+1;

% get pre-hit parameters
for i=1:50
    % transform ball to table coordinates
    b = [pred_in(i,2), TABLE_WIDTH - pred_in(i,1), pred_in(i,3)];
    
    % interpolate to find the paddle pose at time pred_times_in(i)
    t = pred_times_in(i);
    j = find(T_swing>t,1);
    if isempty(j)
        return
    elseif j==1
        continue
    end
    a = (T_swing(j)-t) / (T_swing(j)-T_swing(j-1));
    p = a*P_swing(j-1,1:3) + (1-a)*P_swing(j,1:3);
    n = a*P_swing(j-1,4:6) + (1-a)*P_swing(j,4:6);

    paddle_signed_dist = (b - p)*n' - paddle_thickness/2 - ball_radius;    
    if paddle_signed_dist < 0

        % interpolate to find pre-hit params
        d = paddle_signed_dist;
        a = d / (d - d_prev);
        
        b_hit = a*b_prev + (1-a)*b;
        bv_hit = (b - b_prev) / dt;
        p_hit = a*p_prev + (1-a)*p;
        pv_hit = (p - p_prev) / dt;
        n_hit = a*n_prev + (1-a)*n;
        t_hit = a*t_prev + (1-a)*t;
        
        break
    end
    
    t_prev = t;
    b_prev = b;
    p_prev = p;
    n_prev = n;
    d_prev = paddle_signed_dist;
    
    if i==50
        return
    end
end

% estimate the ball state after collision
bv_out = [];
i1 = find(ball.T > t_hit, 1);
if ~isempty(i1)
    for i=i1:min(i1+20,size(ball.vel,1)-1)
        if ball.T(i) == ball.T(i-1)
            continue
        end
        i2 = i;
        if ball.vel(i,3)<0 && ball.vel(i+1,3)>0
            break
        end
    end
    if i2 > i1+3
        b = [ball.pos(i2,:), ball.vel(i2,:)];
        b = [b(2) TABLE_WIDTH-b(1) b(3) b(5) -b(4) b(6)];
        B_out = b;
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
        b_out = B_out(1,1:3);
        if norm(b_out - b_hit) < .2
            bv_out = B_out(1,4:6);
        end
    end
end


% compute the relative ball velocities
u = cross(n_hit, [1,0,0]);
theta = acos(dot(n_hit, [1 0 0]));
R = quaternionToRotationMatrix([cos(theta/2), u*sin(theta/2)]);
bv_rel_in = (bv_hit - pv_hit)*R';
if isempty(bv_out)
    ball_hit_params = bv_rel_in;
else
    bv_rel_out = (bv_out - pv_hit)*R';
    ball_hit_params = [bv_rel_in, bv_rel_out];
end

% use hit model to predict ball bounce
if nargin >= 6
    bv_rel_out = [1, bv_rel_in]*hit_model;
    bv_out = bv_rel_out*R + pv_hit;

    b = [b_hit, bv_out];
    B_pred = b;
    C = .1;
    p = 1.204;
    m = .0027;
    g = 9.8;
    r = .02;
    A = pi*r^2;
    dt = .01;
    while b(3) > 0
        v = b(4:6);
        Fd = -.5*C*p*A*norm(v)*v;
        b = b + dt*[v, Fd/m - [0,0,g]];
        B_pred(end+1,:) = b;
    end
end


if plotting
    [a1,a2] = view();
    draw_ball_trajectories(ball,[],100,20); hold on
    ja0 = [0.2244, -0.5675, 0.8277, 2.1011, -1.8128, -0.5716, 0.5138];
    q = arm_inverse_kinematics(p_hit', n_hit', ja0, [], 0);
    draw_arm(q); hold on
    plot3(b_hit(1), b_hit(2), b_hit(3), 'bo', 'LineWidth', 3);
    if length(ball_hit_params)==6
        plot3(B_out(:,1), B_out(:,2), B_out(:,3), 'ro');
    end
    if nargin >= 6
        plot3(B_pred(:,1), B_pred(:,2), B_pred(:,3), 'mo');
    end
    hold off
    view(a1,a2);
end


