function [cloud2 A] = transform_point_cloud(x, q, cloud1)
%[cloud2 A] = transform_point_cloud(x, q, cloud1)

n = size(cloud1,1);

M = repmat(mean(cloud1), [n 1]);
T = repmat([x(1) x(2) x(3)], [n 1]);
cloud2 = cloud1 - M;
R = quaternionToRotationMatrix(q);
cloud2 = cloud2*R';
cloud2 = cloud2 + M + T;

mu = mean(cloud1)';
t = [x(1); x(2); x(3)];
A1 = [eye(3) -mu; 0,0,0,1];
A2 = [R t+mu; 0,0,0,1];
A = A2*A1;
