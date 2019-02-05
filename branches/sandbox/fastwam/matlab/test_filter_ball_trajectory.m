
for i=1:length(ball_trajectories)

    T = ball_trajectories{i}.T;
    X = ball_trajectories{i}.X_table;
    
    %[B S] = filter_ball_trajectory(X,T);
    %x0 = [B(1,1:3), B(6,4:6), 0,0,0];
    %[B2 S2] = filter_ball_trajectory(B(:,1:3),T,x0,1);
    [B S] = filter_ball_trajectory(X,T,[],1);
    x0 = [B(1,1:3), B(6,4:6), B(end,7:9)];
    [B2 S2] = filter_ball_trajectory(X,T,x0,1);

    % get true spin
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
        w_true = [w_true(1), -w_true(2), -w_true(3)] / .005;
    else
        w_true = [];
    end

    % plot ball positions
    figure(1);
    for j=1:3, subplot(3,1,j); plot(T-T(1), X(:,j), 'k-'); hold on, end
    for j=1:3, subplot(3,1,j); plot(T-T(1), B(:,j), 'b-'); end
    for j=1:3, subplot(3,1,j); plot(T-T(1), B2(:,j), 'r-'); hold off, end
    
    % plot ball velocities
    figure(2);
    for j=1:3, subplot(3,1,j); plot(T-T(1), B(:,3+j), 'b-'); hold on, end
    for j=1:3, subplot(3,1,j); plot(T-T(1), B2(:,3+j), 'r-'); hold off, end

    % plot ball spins
    figure(3);
    for j=1:3, subplot(3,1,j); plot(T-T(1), B(:,6+j), 'b-'); hold on, end
    for j=1:3, subplot(3,1,j); plot(T-T(1), B2(:,6+j), 'r-'); hold on, end
    if ~isempty(w_true), for j=1:3, subplot(3,1,j); plot([0 T(end)-T(1)], [w_true(j), w_true(j)], 'k-'); end, end
    for j=1:3, subplot(3,1,j); hold off, end

    input(':');
end
