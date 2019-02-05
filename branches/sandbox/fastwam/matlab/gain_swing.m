function [Q,U,T] = gain_swing(ja0, gains, dt, paddle_normal)
%[Q,U,T] = gain_swing(ja0, gains, dt, paddle_normal) --
%
%  Note:  size(gains) = [UD,XD,NT]

UD = size(gains,1);
XD = size(gains,2);
NT = size(gains,3);

% forward swing
gains2 = zeros(NT, UD*XD);
for i=1:NT
    Kt = gains(:,:,i)';
    gains2(i,:) = Kt(:)';
end

% backward swing
%gains2 = [gains2; gains2(end:-1:1,:)];

name = tempname();
write_matrix(gains2, [name '_gains.txt']);

% Save library paths
MatlabPath = getenv('LD_LIBRARY_PATH');
% Make Matlab use system libraries
setenv('LD_LIBRARY_PATH', '/opt/ros/fuerte/lib')

system(sprintf('rosrun pingpong swing_arm -g %f %f %f %f %f %f %f %f %f %f %f %s_gains.txt %s', ...
       ja0(1), ja0(2), ja0(3), ja0(4), ja0(5), ja0(6), ja0(7), ...
       paddle_normal(1), paddle_normal(2), paddle_normal(3), dt, ...
       name, name));

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
