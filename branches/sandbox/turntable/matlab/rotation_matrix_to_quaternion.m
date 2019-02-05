function q = rotation_matrix_to_quaternion(R)
% q = rotation_matrix_to_quaternion(R)

tr = trace(R);
if tr > 0
  S = sqrt(tr+1.0) * 2;  % S=4*qw
  qw = 0.25 * S;
  qx = (R(3,2) - R(2,3)) / S;
  qy = (R(1,3) - R(3,1)) / S; 
  qz = (R(2,1) - R(1,2)) / S; 
elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
  S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; % S=4*qx 
  qw = (R(3,2) - R(2,3)) / S;
  qx = 0.25 * S;
  qy = (R(1,2) + R(2,1)) / S; 
  qz = (R(1,3) + R(3,1)) / S; 
elseif R(2,2) > R(3,3)
  S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; % S=4*qy
  qw = (R(1,3) - R(3,1)) / S;
  qx = (R(1,2) + R(2,1)) / S; 
  qy = 0.25 * S;
  qz = (R(2,3) + R(3,2)) / S; 
else
  S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; % S=4*qz
  qw = (R(2,1) - R(1,2)) / S;
  qx = (R(1,3) + R(3,1)) / S;
  qy = (R(2,3) + R(3,2)) / S;
  qz = 0.25 * S;
end

q = [qw qx qy qz];

