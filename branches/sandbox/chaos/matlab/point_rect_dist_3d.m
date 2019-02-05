function d = point_rect_dist_3d(p,rmin,rmax)
%d = point_rect_dist_3d(p,rmin,rmax) -- computes the distance from a 3D
%point to an axis-aligned rectangle.


% transpose coordinate frame so that rectangle is aligned with the XY plane
if rmin(1)==rmax(1)
    rmin = rmin([2,3,1]);
    rmax = rmax([2,3,1]);
    p = p([2,3,1]);
elseif rmin(2)==rmax(2)
    rmin = rmin([3,1,2]);
    rmax = rmax([3,1,2]);
    p = p([3,1,2]);
end

x = p(1);
y = p(2);
x0 = rmin(1);
x1 = rmax(1);
y0 = rmin(2);
y1 = rmax(2);

dp = [0,0,p(3)-rmin(3)];
if x < x0
    dp(1) = x0-x;
elseif x > x1
    dp(1) = x-x1;
end
if y < y0
    dp(2) = y0-y;
elseif y > y1
    dp(2) = y-y1;
end

d = norm(dp);


