function [Q,U,T] = send_hit_plan(t, pos, vel, normal)
%[Q,U,T] = send_hit_plan(t, pos, vel, normal)

% Save library paths
MatlabPath = getenv('LD_LIBRARY_PATH');
% Make Matlab use system libraries
setenv('LD_LIBRARY_PATH', '/opt/ros/fuerte/lib')

name = tempname();
system(sprintf('rosrun pingpong send_hit_plan %f %f %f %f %f %f %f %f %f %f %s', ...
               t, pos(1), pos(2), pos(3), vel(1), vel(2), vel(3), normal(1), normal(2), normal(3), name));

% Reassign old library paths
setenv('LD_LIBRARY_PATH',MatlabPath)

% load swing
Q = read_matrix([name '_angles.txt']);
U = read_matrix([name '_torques.txt']);
T = read_matrix([name '_times.txt']);
