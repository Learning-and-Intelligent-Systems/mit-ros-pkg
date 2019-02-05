function [torques kp kd err q_swing] = train_smooth_swing(b,t)
%torques = train_smooth_swing(b,t)

TABLE_WIDTH = 1.525;
dt = .01; %.02;
ja0 = [0.1719, -0.1166, 0.4356, 1.9674, -2.0583, -0.7924, 0.1701];  % p = [-.25, .7, .2]
paddle_normal = [1,0,0];  % this never actually gets used, gain_swing() just needs a dummy

swing_step = 5; %10;

%gains
kp = [500, 200, 200, 200, 10, 2, 2];
kd = [2, 2, 2, 1, .2, .1, .1];
smoothness = .5;
gain_eta = .9;  % gain decay rate
gain_err_thresh = .02;
gain_mult_min = .3;

% take first swing
%T = [t:-.01:t-.999]';
%T(1) = 1;  % so brain.cpp doesn't ignore the ball because it's too close
%[~,ihit] = min(abs(T));
%Z = zeros(100,1);
%Z(ihit) = b(3);
%[Q_swing U_swing T_swing] = hallucination_swing([(TABLE_WIDTH - b(2))*ones(100,1), b(1)+T, Z]);
while 1
    try
        [Q_swing U_swing T_swing] = send_hit_plan(t, b(1:3), b(4:6), b(7:9));
        break
    catch
        'Error in train_smooth_swing (send_hit_plan)...trying again'
        pause(.1);
    end
end
%dbug
length(T_swing)
max(T_swing(2:end) - T_swing(1:end-1))

% load first swing plan
%name = '~/ros/mit-ros-pkg/branches/sandbox/pingpong/swing_plan';
name = '~/.ros/swing_plan';
P_plan = read_matrix([name '_paddle.txt']);
Q_plan = read_matrix([name '_joints.txt']);
U_plan = read_matrix([name '_torques.txt']);
%P_plan = [P_plan; P_plan(end:-1:1,:)];
%Q_plan = [Q_plan; Q_plan(end:-1:1,:)];
%U_plan = [U_plan; U_plan(end:-1:1,:)];
n = size(P_plan,1);
Q_swing = Q_swing(1:swing_step:swing_step*n,:);
P_swing = zeros(n,3);
QP_plan = zeros(n,3);
for i=1:n
    QP_plan(i,:) = arm_kinematics(Q_plan(i,:),7,[0;0;.1],1)';
    P_swing(i,:) = arm_kinematics(Q_swing(i,:),7,[0;0;.1],1)';
end
dQ_plan = [zeros(1,7); (Q_plan(2:end,:)-Q_plan(1:end-1,:))/dt];

% plot first swing and plan
figure(1); plot(P_plan(:,1:3)); hold on, plot(QP_plan, 'o-'); plot(P_swing, '--'); hold off, legend('x','y','z');
figure(2); subplot(1,2,1); plot(Q_plan(:,1:4)); hold on, plot(Q_swing(:,1:4), '--'); hold off, legend('1','2','3','4');
subplot(1,2,2); plot(Q_plan(:,5:7)); hold on, plot(Q_swing(:,5:7), '--'); hold off, legend('5','6','7');
figure(3); subplot(1,2,1); plot(U_plan(:,1:4), '-'); legend('1','2','3','4');
subplot(1,2,2); plot(U_plan(:,5:7), '-'); legend('5','6','7');

err_weights = zeros(1,n);
idx = 1:n;
err_weights(idx<n/4) = 2;
err_weights(logical((idx>=n/4).*(idx<n/2-10))) = 4;
du_weights = err_weights;
du_weights(idx>=n/2-10) = 1;
err_weights = err_weights/sum(err_weights);
du_weights = du_weights/sum(du_weights);

gain_mult = 1;
U = U_plan;
P_err = abs(P_plan(:,1:3) - P_swing);
min_err = mean(err_weights*P_err)
best_gain_mult = gain_mult;
min_Q_swing = Q_swing;
min_U = U;

for iter=1:40
    try
        Q_err = Q_plan - Q_swing;
        dQ_err = [Q_err(2:end,:) - Q_err(1:end-1,:); zeros(1,7)] / dt;
        Q_err = [Q_err(2:end,:); zeros(1,7)];
        dU = Q_err.*repmat(.2*kp, [n,1]) + dQ_err.*repmat(kd, [n,1]);
        dU = dU.*repmat(du_weights', [1,7])*n;
        dU = max(dU, repmat(-kd, [n,1]));
        dU = min(dU, repmat(kd, [n,1]));
        U_prev = U;
        U = U + dU;
        U_smooth = zeros(n,7);
        for i=1:7
            U_smooth(:,i) = smooth(1:n, U(:,i), 5); %3
%             U_smooth(:,i) = conv(U_smooth(:,i), [1 1 1 1 1]/5, 'same');
%             U_smooth([1,end],i) = (5/3)*U_smooth([1,end],i);
%             U_smooth([2,end-1],i) = (5/4)*U_smooth([2,end-1],i);
        end
        U = smoothness*U_smooth + (1-smoothness)*U;

        figure(3); subplot(1,2,1); plot(U_prev(:,1:4)); hold on, plot(U(:,1:4), '--');hold off, legend('1','2','3','4');
        subplot(1,2,2); plot(U_prev(:,5:7)); hold on, plot(U(:,5:7), '--'); hold off, legend('5','6','7');
        drawnow;
        %input('hit enter to swing:');

        kp_plan = gain_mult*kp;
        kd_plan = kd; %gain_mult*kd;

        gains = zeros(7,15,n);
        for i=1:n  % forward swing
            gains(:,1,i) = (U(i,:) + kp_plan.*Q_plan(i,:) + kd_plan.*dQ_plan(i,:))';
            gains(1:7,2:8,i) = diag(-kp_plan);
            gains(1:7,9:15,i) = diag(-kd_plan);
        end
    %     for i=1:n  % backward swing
    %         i2 = n-i+1;
    %         gains(:,1,n+i) = (U(i2,:) + kp_plan.*Q_plan(i2,:))'; %- kd_plan.*dQ_plan(i,:)]]';
    %         gains(1:7,2:8,n+i) = diag(-kp_plan);
    %         %gains(1:7,9:15,n+i) = diag(kd_plan);
    %     end

        [Q_swing U_swing T_swing] = gain_swing(ja0, gains, dt, paddle_normal);

        %dbug
        {'NT', length(T_swing)}
        {'max dt', max(T_swing(2:end) - T_swing(1:end-1))}

        Q_swing = Q_swing(1:swing_step:swing_step*n,:);
        P_swing = zeros(n,3);
        for i=1:n
            P_swing(i,:) = arm_kinematics(Q_swing(i,:),7,[0;0;.1],1)';
        end

        % plot swing and plan
        figure(1); plot(P_plan(:,1:3)); hold on, plot(P_swing, '--'); hold off, legend('x','y','z');
        figure(2); subplot(1,2,1); plot(Q_plan(:,1:4)); hold on, plot(Q_swing(:,1:4), '--'); hold off, legend('1','2','3','4');
        subplot(1,2,2); plot(Q_plan(:,5:7)); hold on, plot(Q_swing(:,5:7), '--'); hold off, legend('5','6','7');
        drawnow;

        P_err = abs(P_plan(:,1:3) - P_swing);
        err = mean(err_weights*P_err)
        if err < min_err
            min_err = err
            min_U = U;
            min_Q_swing = Q_swing;
            best_gain_mult = gain_mult;
        elseif gain_mult < .8*best_gain_mult 
            break
        end
        if err < gain_err_thresh
            if gain_mult < gain_mult_min
                break
            end
            gain_mult = gain_mult*gain_eta
        else
            gain_mult = gain_mult*.98
        end
    %     if gain_mult > gain_mult_min
    %         gain_mult = gain_mult*gain_eta
    %     end

        if err < .005 && gain_mult < .6
            break
        end
    
        %input(':');
    catch
        'Error in train_smooth_swing...trying again'
        pause(.1);
    end
end
iter

torques = min_U;
kp = best_gain_mult*kp;
%kd = gain_mult*kd;
err = min_err;
q_swing = min_Q_swing;


