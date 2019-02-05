function draw_arm(joint_angles)

% plot arm
P(:,1) = arm_kinematics(joint_angles, 1, [0,0,0], 1);
P(:,2) = arm_kinematics(joint_angles, 4, [0,0,0], 1);
P(:,3) = arm_kinematics(joint_angles, 7, [0,0,0], 1);
plot3(P(1,1:2), P(2,1:2), P(3,1:2), 'b-', 'LineWidth', 10);
hold on;
plot3(P(1,2:3), P(2,2:3), P(3,2:3), 'g-', 'LineWidth', 7);


% plot paddle
C = arm_kinematics(joint_angles, 7, [0,0,.1], 1);
Y = arm_kinematics(joint_angles, 7, [0,.1,.1], 1);
Z = arm_kinematics(joint_angles, 7, [0,0,.2], 1);
dy = (Y-C)/norm(Y-C);
dz = (Z-C)/norm(Z-C);
theta = 0:.1:2*pi;
P = repmat(C,[1,length(theta)]) + .07*(dy*cos(theta) + dz*sin(theta));
plot3(P(1,:), P(2,:), P(3,:), 'r-', 'LineWidth', 4);


hold off;
axis vis3d
axis equal
