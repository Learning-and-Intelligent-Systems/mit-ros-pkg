function [swing, ball, pred] = record_swing_and_ball_trajectories(timeout)
%[swing, ball, pred] = record_swing_and_ball_trajectories(timeout)
%
%  swing: Q,U,T
%  ball: T,obs,pos,vel,cov
%  pred: T,traj


% Save library paths
MatlabPath = getenv('LD_LIBRARY_PATH');
% Make Matlab use system libraries
setenv('LD_LIBRARY_PATH', '/opt/ros/fuerte/lib')

f = tempname();
swing_file = [f '_swing'];
ball_file = [f '_ball.txt'];
pred_file = [f '_pred.txt'];

if nargin==0
    system(sprintf('rosrun pingpong record_swing_and_ball_trajectories %s %s %s', swing_file, ball_file, pred_file));
else
    system(sprintf('rosrun pingpong record_swing_and_ball_trajectories %s %s %s %f', swing_file, ball_file, pred_file, timeout));
end

% Reassign old library paths
setenv('LD_LIBRARY_PATH',MatlabPath)

% load swing
Q = read_matrix([swing_file '_angles.txt']);
U = read_matrix([swing_file '_torques.txt']);
T = read_matrix([swing_file '_times.txt']);
n = round(size(Q,1)/2);  % just return the first half (forward) swing
Q = Q(1:n,:);
U = U(1:n,:);
T = T(1:n,:);
swing.Q = Q;
swing.U = U;
swing.T = T;

% load ball state trajectory
if nargout >= 2
    ball_matrix = read_matrix(ball_file);
    ball.T = ball_matrix(:,1);
    ball.obs = ball_matrix(:,2:4);
    ball.pos = ball_matrix(:,5:7);
    ball.vel = ball_matrix(:,8:10);
    ball.cov = reshape(ball_matrix(:,11:46)', [6,6,size(ball_matrix,1)]);
end

% load predicted trajectories
if nargout >= 3
    pred_traj_2d = read_matrix(pred_file);
    pred = [];
    pred.T = unique(pred_traj_2d(:,1));
    pred.traj = [];
    for i=1:length(pred.T)
        pred.traj{i} = pred_traj_2d(pred_traj_2d(:,1)==pred.T(i), 2:4);
    end
end

