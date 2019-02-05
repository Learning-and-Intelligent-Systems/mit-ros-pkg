function move_arm(ja0)
%move_arm(ja0)

% Save library paths
MatlabPath = getenv('LD_LIBRARY_PATH');
% Make Matlab use system libraries
setenv('LD_LIBRARY_PATH', '/opt/ros/fuerte/lib')

system(sprintf('rosrun pingpong swing_arm -j %f %f %f %f %f %f %f', ...
       ja0(1), ja0(2), ja0(3), ja0(4), ja0(5), ja0(6), ja0(7)));

% Reassign old library paths
setenv('LD_LIBRARY_PATH',MatlabPath)
