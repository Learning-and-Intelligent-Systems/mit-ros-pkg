
% X = .1; %-.1:.1:.1001; %-.1:.05:.1001;
% Y = .4:.1:.9001; %.4:.1:1.001;
% Z = [.15 .2 .3 .4 .5]; %.1:.1:.5001;
% T = .4;


%for x=.1, vx = [0.73 0.77 0.82 0.87 0.94 1.02 1.14 1.29 1.53 1.92 2.73 5.40]
%          trained: [0.7, 1.3, 1.9, 2.7]

%for x=0, vx = [1.22 1.28 1.36 1.45 1.56 1.70 1.89 2.15 2.54 3.20]
%          trained: [1.2, 1.7, 2.15]

%for x=.1, vx = [1.71 1.80 1.90 2.03 2.19 2.39 2.65 3.01 3.56 4.48 6.36 12.60 -70.00 -7.66 -3.56 ]
%          trained: [1.7, 2.0]


% B = [];
% BT = [];
% U = {};
% KP = {};
% KD = {};
% E = [];
% Q = {};

%dbug: removing bad entries
% n = 111;
% B = B(1:n,:);
% BT = BT(1:n);
% U = U(1:n);
% KP = KP(1:n);
% KD = KD(1:n);
% E = E(1:n);
% Q = Q(1:n);


B_new = [-0.12, 0.76, 0.33; ]
T_new = .4;

% for i=1:length(X)
%     for j=1:length(Y)
%         for k=1:length(Z)
%             for l=1:length(T)
%                 b = [X(i) Y(j) Z(k)]
%                 t = T(l)
for i=1:size(B_new,1)
    b = B_new(i,:);
    t = T_new;
                [u,kp,kd,err,q] = train_smooth_swing(b, t);
                B(end+1,:) = b;
                BT(end+1) = t;
                U{end+1} = u;
                KP{end+1} = kp;
                KD{end+1} = kd;
                E(end+1) = err;
                Q{end+1} = q;
                save smooth_swings_table2.mat B BT U KP KD E Q
%             end
%         end
%     end
end


% compute paddle positions, velocities and normals
n = length(BT);
PP = zeros(n,3);
PV = zeros(n,3);
PN = zeros(n,3);
dt = .01;
for i=1:n
    ihit = ceil(BT(i)/dt);
    paddle_pos = arm_kinematics(Q{i}(ihit,:),7,[0;0;.1],1)';
    paddle_pos0 = arm_kinematics(Q{i}(ihit-2,:),7,[0;0;.1],1)';
    paddle_pos1 = arm_kinematics(Q{i}(ihit+2,:),7,[0;0;.1],1)';
    PP(i,:) = paddle_pos;
    PV(i,:) = (paddle_pos1 - paddle_pos0) / (4*dt);
    PN(i,:) = arm_kinematics(Q{i}(ihit,:),7,[1;0;.1],1)' - paddle_pos;
end


write_matrix(BT, 'swing_table_T.txt');
write_matrix(B, 'swing_table_B.txt');
write_matrix(E, 'swing_table_E.txt');
write_matrix(PP, 'swing_table_PP.txt');
write_matrix(PV, 'swing_table_PV.txt');
write_matrix(PN, 'swing_table_PN.txt');
for i=1:length(U)
    write_matrix(U{i}, sprintf('swing_table_U%d.txt', i));
end


