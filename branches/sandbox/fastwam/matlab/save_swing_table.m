function save_swing_table(swing_table, fdir)
%save_swing_table(swing_table, fdir)

if nargin < 2
    fdir = '.';
end

% compute swing table paddle positions, velocities and normals
n = length(swing_table.BT);
swing_table.PP = zeros(n,3);
swing_table.PV = zeros(n,3);
swing_table.PN = zeros(n,3);
dt = .01;
for i=1:n
    ihit = ceil(swing_table.BT(i)/dt);
    paddle_pos = arm_kinematics(swing_table.Q{i}(ihit,:),7,[0;0;.1],1)';
    paddle_pos0 = arm_kinematics(swing_table.Q{i}(ihit-2,:),7,[0;0;.1],1)';
    paddle_pos1 = arm_kinematics(swing_table.Q{i}(ihit+2,:),7,[0;0;.1],1)';
    swing_table.PP(i,:) = paddle_pos;
    swing_table.PV(i,:) = (paddle_pos1 - paddle_pos0) / (4*dt);
    swing_table.PN(i,:) = arm_kinematics(swing_table.Q{i}(ihit,:),7,[1;0;.1],1)' - paddle_pos;
end

% save swing table
write_matrix(swing_table.BT, [fdir '/swing_table_T.txt']);
write_matrix(swing_table.B, [fdir '/swing_table_B.txt']);
write_matrix(swing_table.E, [fdir '/swing_table_E.txt']);
write_matrix(swing_table.PP, [fdir '/swing_table_PP.txt']);
write_matrix(swing_table.PV, [fdir '/swing_table_PV.txt']);
write_matrix(swing_table.PN, [fdir '/swing_table_PN.txt']);
for i=1:length(swing_table.U)
    write_matrix(swing_table.U{i}, sprintf('swing_table_U%d.txt', i));
end
