function d = point_cloud_dist(cloud1, cloud2)
%d = point_cloud_dist(cloud1, cloud2) -- computes a sum of squared
% nearest neighbor distances from cloud1 to cloud2

n1 = size(cloud1,1);
n2 = size(cloud2,1);

d_tot = 0;
for i=1:n1
    P1 = repmat(cloud1(i,:), [n2 1]);
    D = P1 - cloud2;
    d = min(min(sum(D.*D, 2)), .0001);
    d_tot = d_tot + d;
end

d = d_tot;
