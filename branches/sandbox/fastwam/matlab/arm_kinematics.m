function [p0,A] = arm_kinematics(joint_angles, k, pk, table)
%[p0,A] = arm_kinematics(joint_angles, k, pk) -- Transforms point 'pk' in the
%coordinate frame of joint 'k' into the arm joint 0 coordinate frame,
%given 'joint_angles'.  Also returns the affine transformation matrix.
%
%[p0,A] = arm_kinematics(joint_angles, k, pk, table) -- Transform to table
%coordinates.


c = cos(joint_angles);
s = sin(joint_angles);

p = [pk(1); pk(2); pk(3); 1];
if nargout >= 2
    A = eye(4);
end

a = [0, 0, .045, -.045, 0, 0, 0];
alpha = [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, 0];
ca = round(cos(alpha));
sa = round(sin(alpha));
d = [0, 0, .55, 0, .3, 0, .06];

for i=k:-1:1
    Ai = [c(i), -s(i)*ca(i),  s(i)*sa(i), a(i)*c(i); ...
          s(i),  c(i)*ca(i), -c(i)*sa(i), a(i)*s(i); ...
             0,       sa(i),       ca(i),      d(i); ...
             0,           0,           0,         1];

    p = Ai*p;
    if nargout >= 2
        A = Ai*A;
    end
end



if nargin >= 4 && table
    
    % handle the rotation of the WAM mounting board
    %theta = .1587;  % about pi/20, or 9 degrees
    theta = .0457;  % 01/29/14
    R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
    
    % rotation from arm coordinates to table coordinates
    R = [1,0,0; 0,0,1; 0,-1,0]*R;
    
    % translation
    %v = [-.44; .28; .5];
    %v = [-.44; .28; .55];
    v = [-.47; .27; .53];  % 01/29/14
    
    T = [R, v; 0,0,0,1];
    
    %T = [1,0,0,-.71; 0,0,1,.28; 0,-1,0,.5; 0,0,0,1];
    %T = [1,0,0,-.44; 0,0,1,.28; 0,-1,0,.5; 0,0,0,1];
    p = T*p;
    if nargout >= 2
        A = T*A;
    end
end

p0 = p(1:3);
