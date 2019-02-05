cd ~/ros/mit-ros-pkg/branches/sandbox/pingpong/data/

% X = {}; Y = {};
% for j=1:7
%     X{j} = read_matrix(sprintf('swing1_X%d.txt',j-1));
%     Y{j} = read_matrix(sprintf('swing1_Y%d.txt',j-1));
% end
% 

paddle_normal = [1,0,0];
ja0 = [0.3077, -0.2558, 0.3703, 2.0547, -2.0850, -0.9102, 0.1861];
dt = .02;
T = .5;  % duration
NT = round(T/dt);

p = arm_kinematics(ja0,7,[0;0;.1],1);
n = arm_kinematics(ja0,7,[1;0;.1],1) - p;
v = [.5,0,.2]';

Q = ja0;
for i=1:NT
    Q(end+1,:) = arm_inverse_kinematics(p + i*dt*v, n, Q(end,:));
end
dQ = (Q(2:end,:) - Q(1:end-1,:)) / dt;
Q = Q(2:end,:);

% get gravity torques
Ug = get_gravity_torques_for_trajectory([ja0; Q]);

% try a few joint swings to collect dynamics data
X = {[],[],[],[]};
Y = {[],[],[],[]};
for i=1:3
    [Q2,U,T] = joint_swing([ja0; Q], dt, paddle_normal);
    [X,Y] = add_swing_to_dynamics_model(X,Y,Q,U,T);
end




q = ja0';
dq = zeros(7,1);

dtn = 2;
dt2 = dt/dtn;

%gains = zeros(NT, 7*15);
%gains = zeros(NT, 4*9);
gains = [];

for t=1:NT

%     q_des = Q(t,:);
%     dq_des = dQ(t,:);
%     
%     [~,idx] = min(sum((X{1}(:,1:14) - repmat([q_des,dq_des], [size(X{1},1),1])).^2, 2));
% 
%     AB = [];
%     joint_noise = [.1,.05,.05,.05,.01,.01,.002];
%     for j=1:7
%         r = [.1,.1,.1,.1,.1,.1,.1, .2,.2,.2,.2,.2,.2,.2, 10*joint_noise(j)];
%         [y ab] = lwrsample(X{j},Y{j},[q_des,dq_des,X{j}(idx,15)], r);
%         AB(j,:) = ab;
%     end

    q_des = Q(t,1:4);
    dq_des = dQ(t,1:4);
    
    [~,idx] = min(sum((X4{1}(:,1:8) - repmat([q_des,dq_des], [size(X4{1},1),1])).^2, 2));

    AB = [];
    joint_noise = [.1,.05,.05,.05];
    for j=1:4
        %r = [.1,.1,.1,.1, .2,.2,.2,.2, 10*joint_noise(j)];
        %[y ab] = lwrsample(X4{j},Y4{j},[q_des,dq_des,X4{j}(idx,9)], r, 1, 1);

        u = X4{j}(idx,9);
        ug = Ug(t+1,j);
        Xj = [X4{j}(:,1:8), X4{j}(:,9) - ug];
        %Xj = [X4{j}(:,5:8), X4{j}(:,9) - ug];
        Yj = Y4{j};
        x = [q_des, dq_des, u-ug];
        %x = [dq_des, u-ug];
        
        r = [.1,.1,.1,.1, .2,.2,.2,.2, 10*joint_noise(j)];
        %r = [.2,.2,.2,.2, 10*joint_noise(j)];
        [y ab] = lwrsample(Xj, Yj, x, r, 1, 0);

        AB(j,:) = ab;
        %AB(j,:) = [0, 0,0,0,0, ab(2:end)];
    end

%     A = [zeros(1,15); zeros(7,8), eye(7); AB(:,1:15)];
%     B = [zeros(8,7); diag(AB(:,16))];
% 
%     qdq_des = [Q(t,:) dQ(t,:)]';
%     C = [1 + qdq_des'*qdq_des, -qdq_des'; -qdq_des, eye(14)];

% wam7:
%
%     A1 = AB(:,1);
%     A2 = AB(:,2:8);
%     A3 = AB(:,9:15);
%     A = [zeros(1,15); dq_des', zeros(7), eye(7); A1+A2*q_des'+A3*dq_des', A2, A3];
%     B = [zeros(8,7); diag(AB(:,16))];
%     C = eye(15);
%     D = .0001*eye(7); %R

    A1 = AB(:,1);
    A2 = AB(:,2:5);
    A3 = AB(:,6:9);
    A = [zeros(1,9); dq_des', zeros(4), eye(4); A1+A2*q_des'+A3*dq_des', A2, A3];
    B = [zeros(5,4); diag(AB(:,10))];
    C = diag([.1,  .1 .1 .1 .1,  1 1 1 1]);
    D = .001*diag([1 1 2 2]); %R
    E = diag([.1,  1 1 1 1,  1 1 1 1]);

%     %try
%         K = lqr(A,B,C,D)
%     %end

    %A = eye(15) + dt*A;
    A = eye(9) + dt2*A;
    B = dt2*B;
    
    K_seq = zeros(4,9,dtn);
    for t2=dtn:-1:1
        K = -(D + B'*E*B) \ B'*E*A;
        E = C + K'*D*K + (A'+B*K)'*E*(A'+B*K);
        K_seq(:,:,t2) = K;
    end

    for t2=1:dtn
        K = K_seq(:,:,t2)
        K2 = [K(:,1) - K*[0,q_des,dq_des]', K(:,2:end)];  % make K be a gain on x, not x-x0
        K2(:,1) = K2(:,1) + Ug(t+1,1:4)';  % add gravity torques
        Kt = K2';
        Kt = [Kt zeros(9,3)];
        Kt = [Kt(1:5,:); zeros(3,7); Kt(6:9,:); zeros(3,7)];
        gains(end+1,:) = Kt(:)';

        u = K*[1; q(1:4)-q_des'; dq(1:4)-dq_des']

    %     u = K2*[1; q; dq]

        %u = K2*[1; q(1:4); dq(1:4)]

    %     q = q + dt*dq;
    %     dq = dq + dt*(A1+A2*q+A3*dq) + B(9:end,:)*u;

        q(1:4) = q(1:4) + dt2*dq(1:4);
        dq(1:4) = dq(1:4) + dt2*(A1+A2*q(1:4)+A3*dq(1:4)) + B(6:end,:)*u;
        q(5:7) = Q(t,5:7);
    end
    
    %q_err = q-q_des'
    %dq_err = dq-dq_des'
    
    figure(1); [a1,a2] = view(); draw_table, hold on, draw_arm(q); view(a1,a2); drawnow;
    
    %dbug
    %U(t+1,:)
    %AB(:,[1,10])
    %[-AB(:,10).*U(t+1,1:4)',  A1+A2*q_des']
    
    
    %input(':');
end

write_matrix(gains, 'gains.txt');


input('Ready for chaos:');


% Save library paths
MatlabPath = getenv('LD_LIBRARY_PATH');
% Make Matlab use system libraries
setenv('LD_LIBRARY_PATH', '/opt/ros/fuerte/lib')

system(sprintf('rosrun pingpong swing_arm -g %f %f %f %f %f %f %f %f %f %f %f gains.txt gains_data', ...
       ja0(1), ja0(2), ja0(3), ja0(4), ja0(5), ja0(6), ja0(7), paddle_normal(1), paddle_normal(2), paddle_normal(3), dt2));

% Reassign old library paths
setenv('LD_LIBRARY_PATH',MatlabPath)


swing_T = read_matrix('gains_data_times.txt');
swing_Q = read_matrix('gains_data_angles.txt');
swing_U = read_matrix('gains_data_torques.txt');
swing_U = swing_U(1:end-1,:);

for i=1:size(swing_Q,1)
    q = swing_Q(i,:);
    figure(1); [a1,a2] = view(); draw_table, hold on, draw_arm(q); view(a1,a2); drawnow; 
end
figure(2); plot(swing_U);

% for j=1:7
%     X{j} = [X{j}; read_matrix(sprintf('gains_data_X%d.txt',j-1))];
%     Y{j} = [Y{j}; read_matrix(sprintf('gains_data_Y%d.txt',j-1))];
% end



