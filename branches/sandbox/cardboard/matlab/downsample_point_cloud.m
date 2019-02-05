function cloud2 = downsample_point_cloud(cloud1, res)
% cloud2 = downsample_point_cloud(cloud1, res) -- grid-based downsampler

pmin = min(cloud1) - res/4;
pmax = max(cloud1) + res/4;

grid_size = ceil((pmax-pmin)/res);

grid = cell(grid_size);
grid_cnts = zeros(grid_size);

n1 = size(cloud1,1);
for i=1:n1
    p = ceil((cloud1(i,:) - pmin)/res);
    cnt = grid_cnts(p(1),p(2),p(3));
    if (cnt == 0)
        grid{p(1),p(2),p(3)} = cloud1(i,:);
    else
        grid{p(1),p(2),p(3)} = (cloud1(i,:) + cnt*grid{p(1),p(2),p(3)}) / (cnt+1);
    end
    grid_cnts(p(1),p(2),p(3)) = cnt+1;
end

I = find(grid_cnts~=0);
n2 = length(I);
cloud2 = zeros(n2,3);
for i=1:n2
    cloud2(i,:) = grid{I(i)};
end

% figure(3);
% clf;
% hold on;
% plot3(cloud1(:,1), cloud1(:,2), cloud1(:,3), 'b.');
% plot3(cloud2(:,1), cloud2(:,2), cloud2(:,3), 'r.');
% hold off;
% axis vis3d;
% axis equal;
