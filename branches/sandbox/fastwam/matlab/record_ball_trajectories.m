function [traj, pred_traj, traj_times, pred_traj_times] = record_ball_trajectories(timeout)
%[traj, pred_traj, traj_times, pred_traj_times] = record_ball_trajectories(timeout)

% Save library paths
MatlabPath = getenv('LD_LIBRARY_PATH');
% Make Matlab use system libraries
setenv('LD_LIBRARY_PATH', '/opt/ros/fuerte/lib')

f = tempname();
traj_file = [f '_traj.txt'];
pred_traj_file = [f '_pred_traj.txt'];

if nargin==0
    system(sprintf('rosrun pingpong record_ball_trajectories %s %s', traj_file, pred_traj_file));
else
    system(sprintf('rosrun pingpong record_ball_trajectories %s %s %f', traj_file, pred_traj_file, timeout));
end

% Reassign old library paths
setenv('LD_LIBRARY_PATH',MatlabPath)


% load ball trajectories
traj_2d = read_matrix(traj_file);
pred_traj_2d = read_matrix(pred_traj_file);

if isempty(traj_2d) %dbug
    traj = {};
    traj_times = [];
else
    traj_times = unique(traj_2d(:,1));
    traj = {};
    for i=1:length(traj_times)
        traj{i} = traj_2d(traj_2d(:,1)==traj_times(i), 2:4);
    end
end

pred_traj_times = unique(pred_traj_2d(:,1));
pred_traj = [];
for i=1:length(pred_traj_times)
    pred_traj{i} = pred_traj_2d(pred_traj_2d(:,1)==pred_traj_times(i), 2:4);
end
