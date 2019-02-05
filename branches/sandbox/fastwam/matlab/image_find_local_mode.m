function p2 = image_find_local_mode(I,p,r,rmax)
%p2 = image_find_local_mode(I,p,r,rmax)

h = size(I,1);
w = size(I,2);

if nargin < 4
    rmax = max(w,h);
end

x = round(p(1));
y = round(p(2));

by0 = max(y-rmax, 1);
by1 = min(y+rmax, h);
bx0 = max(x-rmax, 1);
bx1 = min(x+rmax, w);

while 1
    y0 = max(y-r, by0);
    y1 = min(y+r, by1);
    x0 = max(x-r, bx0);
    x1 = min(x+r, bx1);
    
    I2 = I(y0:y1,x0:x1);
    
    [~,i] = max(I2(:));
    [y2,x2] = ind2sub(size(I2), i);
    x2 = x2+x0-1;
    y2 = y2+y0-1;
    
    if x==x2 && y==y2
        break
    end
    x = x2;
    y = y2;
end

p2 = [x2 y2];
