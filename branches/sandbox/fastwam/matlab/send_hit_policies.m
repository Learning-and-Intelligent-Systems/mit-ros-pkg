function send_hit_policies(HP_in, HP_out)
%send_hit_policies(HP_in, HP_out) -- sends HP to brain

f_in = tempname();
f_out = tempname();
write_matrix(HP_in, f_in);
write_matrix(HP_out, f_out);


% Save library paths
MatlabPath = getenv('LD_LIBRARY_PATH');
% Make Matlab use system libraries
setenv('LD_LIBRARY_PATH', '/opt/ros/fuerte/lib')

system(sprintf('rosrun pingpong send_hit_policies %s %s', f_in, f_out));

% Reassign old library paths
setenv('LD_LIBRARY_PATH',MatlabPath)
