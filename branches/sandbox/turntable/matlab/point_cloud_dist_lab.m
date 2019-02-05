function d = point_cloud_dist_lab(cloud1, cloud2, lab1, lab2, dmax)
%d = point_cloud_dist_lab(cloud1, cloud2, lab1, lab2) -- computes a sum of squared
% nearest neighbor (and color) distances from cloud1 to cloud2

n1 = size(cloud1,1);
n2 = size(cloud2,1);

lab1 = lab1/5000;
lab2 = lab2/5000;

d_tot = 0;
d_tot_color = 0;
for i=1:n1
    P1 = repmat(cloud1(i,:), [n2 1]);
    D = P1 - cloud2;
    [d,j] = min(sum(D.*D, 2));
    if nargin >= 5
        d = min(d, dmax^2);
    end
    d_color = norm(lab1(i,:) - lab2(j,:))^2;
    d_tot = d_tot + d;
    d_tot_color = d_tot_color + d_color;
end

[d_tot, d_tot_color]

d = d_tot + d_tot_color;
