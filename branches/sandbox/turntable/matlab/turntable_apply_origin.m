function clouds2 = turntable_apply_origin(clouds, origin, dtheta)
%clouds2 = turntable_apply_origin(clouds, origin, dtheta) -- undo rotations about
%the origin in a list of point clouds

clouds2 = [];
for i=1:length(clouds),
    theta = -(i-1)*dtheta;
    R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0,0,1];
    clouds2{i} = (clouds{i} - repmat(origin, [size(clouds{i},1),1]))*R';
end
