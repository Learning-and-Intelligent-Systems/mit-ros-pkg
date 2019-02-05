function d = point_box_dist_3d(p, bmin, bmax, box_has_bottom)
%d = point_box_dist_3d(p, bmin, bmax)
%d = point_box_dist_3d(p, bmin, bmax, box_has_bottom)-- computes the
% distance from a 3D point to an axis-aligned box.

if nargin < 4
    box_has_bottom = 1;
end

x0 = bmin(1);
y0 = bmin(2);
z0 = bmin(3);
x1 = bmax(1);
y1 = bmax(2);
z1 = bmax(3);

D = repmat(inf, [1,6]);
D(1) = point_rect_dist_3d(p, [x0 y0 z0], [x1, y0, z1]);  % front
D(2) = point_rect_dist_3d(p, [x0 y0 z0], [x0, y1, z1]);  % left
D(3) = point_rect_dist_3d(p, [x1 y0 z0], [x1, y1, z1]);  % right
D(4) = point_rect_dist_3d(p, [x0 y1 z0], [x1, y1, z1]);  % back
D(5) = point_rect_dist_3d(p, [x0 y0 z1], [x1, y1, z1]);  % top
if box_has_bottom
    D(6) = point_rect_dist_3d(p, [x0 y0 z0], [x1, y1, z0]);  % bottom
end

d = min(D);
