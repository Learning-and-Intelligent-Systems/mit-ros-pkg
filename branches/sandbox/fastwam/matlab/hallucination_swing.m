function [Q,U,T] = hallucination_swing(ball_traj)
%[Q,U,T] = hallucination_swing(ball_traj)

name = tempname();
write_matrix(ball_traj, [name '_ball.txt']);


% Save library paths
MatlabPath = getenv('LD_LIBRARY_PATH');
% Make Matlab use system libraries
setenv('LD_LIBRARY_PATH', '/opt/ros/fuerte/lib')

system(sprintf('rosrun pingpong hallucinate_ball %s_ball.txt %s', name, name));

% Reassign old library paths
setenv('LD_LIBRARY_PATH',MatlabPath)


% load swing
Q = read_matrix([name '_angles.txt']);
U = read_matrix([name '_torques.txt']);
T = read_matrix([name '_times.txt']);

% just return the first half (forward) swing
% n = round(size(Q,1)/2);
% Q = Q(1:n,:);
% U = U(1:n,:);
% T = T(1:n,:);
