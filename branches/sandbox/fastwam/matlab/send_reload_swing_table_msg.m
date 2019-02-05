function send_reload_swing_table_msg()
%send_reload_swing_table_msg()

% Save library paths
MatlabPath = getenv('LD_LIBRARY_PATH');
% Make Matlab use system libraries
setenv('LD_LIBRARY_PATH', '/opt/ros/fuerte/lib')

system('rosrun pingpong reload_swing_table');

% Reassign old library paths
setenv('LD_LIBRARY_PATH',MatlabPath)
