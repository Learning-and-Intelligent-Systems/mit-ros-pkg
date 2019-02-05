function [Q,U,T] = joint_swing(Q_in, dt, paddle_normal)
%[Q,U,T] = joint_swing(Q_in, dt, paddle_normal)

name = tempname();
write_matrix(Q_in, [name '_angles.txt']);

ja0 = Q_in(1,:);

% Save library paths
MatlabPath = getenv('LD_LIBRARY_PATH');
% Make Matlab use system libraries
setenv('LD_LIBRARY_PATH', '/opt/ros/fuerte/lib')

system(sprintf('rosrun pingpong swing_arm -j %f %f %f %f %f %f %f %f %f %f %f %s_angles.txt %s', ...
       ja0(1), ja0(2), ja0(3), ja0(4), ja0(5), ja0(6), ja0(7), ...
       paddle_normal(1), paddle_normal(2), paddle_normal(3), dt, ...
       name, name));

% Reassign old library paths
setenv('LD_LIBRARY_PATH',MatlabPath)


% load swing
Q = read_matrix([name '_angles.txt']);
U = read_matrix([name '_torques.txt']);
T = read_matrix([name '_times.txt']);
