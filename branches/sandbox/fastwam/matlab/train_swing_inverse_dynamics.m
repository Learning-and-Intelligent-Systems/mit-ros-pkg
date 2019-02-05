cd ~/ros/mit-ros-pkg/branches/sandbox/pingpong/data/

gravity_table = load('gravity_table.mat');

% X = {}; Y = {};
% for j=1:7
%     X{j} = read_matrix(sprintf('swing1_X%d.txt',j-1));
%     Y{j} = read_matrix(sprintf('swing1_Y%d.txt',j-1));
% end
% 

paddle_normal = [1,0,0];
ja0 = [0.3077, -0.2558, 0.3703, 2.0547, -2.0850, -0.9102, 0.1861];
dt = .02;
T = .7;  % duration
NT = round(T/dt);


%dbug: try a new ja0
p0 = arm_kinematics(ja0,7,[0;0;.1],1);
n0 = arm_kinematics(ja0,7,[1;0;.1],1) - p0;
ja0 = arm_inverse_kinematics(p0+[0;.2;0], n0, ja0);



p0 = arm_kinematics(ja0,7,[0;0;.1],1);
n0 = arm_kinematics(ja0,7,[1;0;.1],1) - p0;
v_des = [.8,0,.2]';  % desired paddle velocity
a = [2,2,2]';  % max paddle acceleration

% get a feasible (acceleration-limited) paddle trajectory
P = p0';
V = [0,0,0];
for i=1:NT
    dv = v_des - V(i,:)';
    c = max(abs(dv) ./ (a*dt));
    if c > 1
        dv = dv / c;
    end
    V(i+1,:) = V(i,:) + dv';
    P(i+1,:) = P(i,:) + V(i+1,:)*dt;
end

% compute the joint trajectory
Q = ja0;
for i=1:NT
    Q(i+1,:) = arm_inverse_kinematics(P(i+1,:)', n0, Q(i,:));
end
dQ = [0*ja0; (Q(2:end,:) - Q(1:end-1,:))] / dt;
ddQ = [0*ja0; (dQ(2:end,:) - dQ(1:end-1,:))] / dt;

% get gravity torques
Ug = get_gravity_torques_for_trajectory(Q, gravity_table);

% try a few joint swings to collect dynamics data
X = {[],[],[],[]};
Y = {[],[],[],[]};
% for i=1:3
%     [Qi,Ui,Ti] = joint_swing(Q, dt, paddle_normal);
%     [X,Y] = add_swing_to_dynamics_model(X,Y,Qi,Ui,Ti);
% end

% try a few gain swings (with gravity)
Kp = diag([500,200,200,200,10,2,2]);
Kd = diag([0,0,0,0,0,0,0]);
for xxx=1:5
    gains = zeros(7,15,NT);
    for i=1:NT
        gains(:,:,i) = [Ug(i,:)'+Kp*Q(i,:)'+Kd*dQ(i,:)',  -Kp,  -Kd];
        %gains(:,:,i+1) = [Ug(1,:)',  zeros(7,14)];
    end
    gains2 = gains(:,:,1);
    for i=1:NT-1
        gains2(:,:,2*i) = (gains(:,:,i) + gains(:,:,i+1)) / 2;
        gains2(:,:,2*i+1) = gains(:,:,i+1);
    end
    [Qi,Ui,Ti] = gain_swing(ja0, gains2, dt/2, paddle_normal);
    
%     for i=1:size(Qi,1)
%         q = Qi(i,:);
%         figure(1); [a1,a2] = view(); draw_table, hold on, draw_arm(q); view(a1,a2); drawnow; 
%     end
%     figure(2); plot(Ui(1:end-1,:));
    
    [X,Y] = add_swing_to_dynamics_model(X,Y,Qi,Ui,Ti);

    % plot swing
    Pi = zeros(size(Qi,1),3);
    figure(1);
    for i=1:size(Qi,1)
        q = Qi(i,:);
        Pi(i,:) = arm_kinematics(q, 7, [0,0,.1], 1);
        [a1,a2] = view(); draw_table, hold on, draw_arm(q); view(a1,a2); drawnow; 
    end
    hold on, plot3(P(:,1), P(:,2), P(:,3), 'b-'); plot3(Pi(:,1), Pi(:,2), Pi(:,3), 'r-'); hold off;
    figure(2); plot(Ui(1:end-1,:));
    figure(3); for j=1:4, subplot(4,1,j); plot(dt*[0:NT], Q(:,j),'-'); hold on, plot(Ti-Ti(1),Qi(:,j),'--'); hold off; end
    figure(4); plot(dt*[0:NT], P,'-'); hold on, plot(Ti-Ti(1),Pi,'--'); hold off;
    
    input(':');
end




% convert to inverse dynamics model
X2 = X;
Y2 = Y;
for j=1:length(X)
    U = Y{j};
    Y2{j} = X{j}(:,end);
    X2{j} = [X{j}(:,1:end-1), U];
end


for xxx=1:10,
    
    % look up torques to acheive desired trajectory, (Q,dQ,ddQ)
    U = zeros(NT,7);
    for i=1:NT
        joint_noise = [.1,.05,.05,.05];
        %r = [.1,.1,.1,.1, .2,.2,.2,.2, 10*joint_noise(j)];
        r = 1./[.01,.01,.01,.01, .05,.05,.05,.05, 1];
        for j=1:length(X)
            x = [Q(i,1:4), dQ(i,1:4), ddQ(i,j)];
            [y ab] = lwrsample(X2{j}, Y2{j}, x, r);
            U(i,j) = y;
        end
    end
    
    Kp = diag([10,4,4,4,0,0,0]);
    %Kp = diag([0,0,0,0,0,0,0]);
    Kd = diag([0,0,0,0,0,0,0]);
    gains = zeros(7,15,NT);
    for i=1:NT
        gains(:,:,i) = [U(i,:)'+Kp*Q(i,:)'+Kd*dQ(i,:)',  -Kp,  -Kd];
    end
    [Qi,Ui,Ti] = gain_swing(ja0, gains, dt, paddle_normal);
    [X,Y] = add_swing_to_dynamics_model(X,Y,Qi,Ui,Ti);
    
    % convert to inverse dynamics model
    for j=1:length(X)
        Y2{j} = X{j}(:,end);
        X2{j} = [X{j}(:,1:end-1), Y{j}];
    end
    
    
    % plot swing
    Pi = zeros(size(Qi,1),3);
    figure(1);
    for i=1:size(Qi,1)
        q = Qi(i,:);
        Pi(i,:) = arm_kinematics(q, 7, [0,0,.1], 1);
        [a1,a2] = view(); draw_table, hold on, draw_arm(q); view(a1,a2); drawnow; 
    end
    hold on, plot3(P(:,1), P(:,2), P(:,3), 'b-'); plot3(Pi(:,1), Pi(:,2), Pi(:,3), 'r-'); hold off;
    figure(2); plot(Ui(1:end-1,:));
    figure(3); plot(dt*[0:NT], P,'-'); hold on, plot(Ti-Ti(1),Pi,'--'); hold off;

    input(':');
end
    
    
