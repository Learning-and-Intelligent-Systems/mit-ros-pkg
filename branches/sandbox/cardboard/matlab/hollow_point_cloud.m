function cloud2 = hollow_point_cloud(cloud, grid, viewpoints)
%cloud2 = hollow_point_cloud(cloud, grid, viewpoints) -- hollow out a
%point cloud by ray-tracing in an occupancy grid to a set of viewpoints.

n = size(cloud,1);
G = (grid.occ > .5).*(grid.cnt >= 1);

mask = zeros(1,n);
for i=1:n
    if mod(i,100)==0
        fprintf('o');
    end
    c0 = ceil((cloud(i,:) - grid.min)/grid.res);  % point cell
    for j=1:size(viewpoints,1)
        vp = viewpoints(j,:);
        ray_dir = cloud(i,:) - vp;
        ray_len = norm(ray_dir);
        ray_dir = ray_dir/ray_len;  % unit vector
        
        visible = 1;
        for j=0:grid.res/2:ray_len  % discretize ray from vp to cloud(i,:)
            p = vp + j*ray_dir;                 % point along ray
            c = ceil((p - grid.min)/grid.res);  % ray point cell
            if (c >= [1,1,1]) .* (c <= size(grid.occ))
                if c == c0
                    continue;
                end
                if G(c(1),c(2),c(3))
                    visible = 0;
                    break;
                end
            end
        end
        
        % if point is visible from any viewpoint, keep it
        if visible
            mask(i) = 1;
            break;
        end
    end
end
fprintf('\n');

cloud2 = cloud(find(mask),:);

