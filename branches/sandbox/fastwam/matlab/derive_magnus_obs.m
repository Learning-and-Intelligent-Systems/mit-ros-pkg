
sigma_y = 1;
sigma_z = 2;


% for y=0:.1:pi/2
%     for z=0:.1:pi/2
%         n=1000000;
%         w = [pi*rand(n,1)-pi/2, normrnd(0,sigma_y,n,1), normrnd(0,sigma_z,n,1)];
%         a = sqrt(sum(w.^2,2));
%         q = [cos(a/2), repmat(sin(a/2),[1,3]).*w./repmat(a,[1,3])];
%         B = bingham_fit(q);
%         B.V
%         {'y', y, 'z', z, 'B.Z', B.Z(1), B.Z(2), B.Z(3)}
%         input(':');
%     end
% end


% n = 50000;
% w = repmat([0, 0, .5], [n,1]) + normrnd(0,.1,n,3);
% q = zeros(n,4);  for i=1:n, q(i,:) = axis_angle_to_quaternion(w(i,:)); end
% B = bingham_fit(q);
% V = B.V
% z = B.Z
% w = quaternion_to_axis_angle(bingham_mode(B))
% q = bingham_sample(B, n);
% w = zeros(n,3);  for i=1:n, w(i,:) = quaternion_to_axis_angle(q(i,:)); end
% mean(w)
% std(w)


% n = 50000;
% z = [-4.6, -4.6, 0];
% w = [0, 1.2, 1.2];
% 
% a = norm(w);
% if a==0
%     mu = [1 0 0 0]';
% else
%     mu = [cos(a/2), sin(a/2)*w/a]';
% end
% v3 = [0,1,0,0]';
% P = (eye(4) - mu*mu')*(eye(4) - v3*v3');
% v2 = P*normrnd(0,1,4,1);
% v2 = v2/norm(v2);
% P = P*(eye(4) - v2*v2');
% v1 = P*normrnd(0,1,4,1);
% v1 = v1/norm(v1);
% V = [v1 v2 v3];
% %det([V mu])
% 
% B = struct('d',4,'Z',z,'V',V);
% [B.F B.dF] = bingham_F(B.Z);
% 
% q = bingham_sample(B, n);
% w = quaternion_to_axis_angle(bingham_mode(B))'
% w = zeros(n,3);  for i=1:n, w(i,:) = quaternion_to_axis_angle(q(i,:)); end
% mean(w)
% std(w)


f = pwd();
I = find(f=='/');
i1 = I(end-1);
i2 = I(end);
spin = sscanf(f(i1+1:i2-1), 'spin%d')
speed = sscanf(f(i2+1:end), 'speed%d')

Cm = .0004; %.0004;

for i=1:length(ball_trajectories)
%for i=1:2:length(ball_trajectories)
    T = ball_trajectories{i}.T;
    X = ball_trajectories{i}.X_table;
    
%    [XV, XV_cov, XV2, XV2_cov, F] = filter_ball_trajectory(X,T);
    [XV, XV_cov] = filter_ball_trajectory(X,T);
    x0 = [XV(1,1:3), XV(6,4:6)];
    [XV, XV_cov] = filter_ball_trajectory(X,T,x0);
    
    if ~isfield(ball_trajectories{i}, 's')
        try
            Q = ball_trajectories{i}.Q_truth;
            [s0,s] = quaternion_regression(Q, [], newgy_to_quaternion(spin, speed), .2);
            ball_trajectories{i}.s0 = s0;
            ball_trajectories{i}.s = s;
        end
    end
    if isfield(ball_trajectories{i}, 's')
        w_true = quaternion_to_axis_angle(ball_trajectories{i}.s);
        w_true = [w_true(1), -w_true(2), -w_true(3)];
    else
        w_true = [];
    end
    B = bingham_new_uniform(4);
    W = [];
    DA = [];
    V = [];
    for j=6:length(T)-1

        dt1 = T(j) - T(j-1);
        b1 = XV(j-1,:);
        b2 = XV(j,:);
        b2_pred = ball_dynamics(b1', dt1)';
        %C1 = XV_cov(:,:,j-1)*F(:,:,j-1)'*inv(XV2_cov(:,:,j-1));
        %b1_smooth = (b1' + C1*(b2 - b2_pred)')';
        %b2_pred2 = ball_dynamics(b1_smooth', dt1)';

        % get observed and predicted (no-spin) accelerations
        %v1 = b1_smooth(4:6);
        %v2 = b2(4:6);
        %v2_pred = b2_pred2(4:6);
        v1 = b1(4:6);
        v2 = b2(4:6);
        v2_pred = b2_pred(4:6);
        a = (v2-v1)/dt1;
        a_pred = (v2_pred-v1)/dt1;
        da = a - a_pred;
        DA(end+1,:) = da;
        V(end+1,:) = v2_pred;
        
        % get a rotation that takes v1 to the v0=[0,-1,0] axis
        v0 = [0,-1,0];
        theta = acos(dot(v1,v0) / norm(v1));
        u = cross(v1, v0);
        vq = [cos(theta/2), sin(theta/2)*u/norm(u)];
        R = quaternionToRotationMatrix(vq);
        
        % get spin observation (in v0's coordinate frame)
        w0 = .005 * cross(v0, R*da') /(Cm*dot(v1,v1));
        w0 = min(max(w0, -pi/2), pi/2);
        
        w0(1) = 0;  %dbug
        
        % transform spin observation back into table coordinates
        w = w0*R;
        W(end+1,:) = w;
        w0
        w
        
        % get Bingham observation
        q = axis_angle_to_quaternion(w);
        v4 = q';        
        v3 = [0,v1/norm(v1)]';
        P = (eye(4) - [v3 v4]*[v3 v4]');
        v2 = P*normrnd(0,1,4,1);
        v2 = v2/norm(v2);
        P = P*(eye(4) - v2*v2');
        v1 = P*normrnd(0,1,4,1);
        v1 = v1/norm(v1);
        V = [v1 v2 v3];
        z = [-4.6, -4.6, -4.6];
        B_obs = struct('d',4,'Z',z,'V',V);
        
        B = bingham_mult(B, B_obs, 0);
        
        mu = bingham_mode(B)
        z = B.Z
        w_bingham = quaternion_to_axis_angle(mu)'
        input(':');
    end
    %figure(1); for j=1:3, subplot(3,1,j); plot(7:length(T)-1, DA(6:end,j)); end
    
    %figure(2); for j=1:3, subplot(3,1,j); plot(V(:,j)); hold on, plot(XV(2:end-1,3+j),'r-'); hold off, end
    %b = regress(XV(:,1), [ones(length(T),1) T(1:end)-T(1)]);
    w_avg = mean(W(5:end,:))
    w_std = std(W(5:end,:))
    %wz_reg = .005 * b(2)/(Cm*dot(v1,v1))
    w_bingham = quaternion_to_axis_angle(bingham_mode(B))'
    w_bingham_z = B.Z
    w_true
    
    %figure(3); for j=1:3, subplot(3,1,j); plot(X(2:end-1,j)); hold on, plot(XV(2:end-1,j),'r-'); hold off, end
    input(':')
end




