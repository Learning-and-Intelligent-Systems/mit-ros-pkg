function grid2 = update_occupancy_grid(grid, cloud, vp)
% grid2 = update_occupancy_grid(grid, cloud, vp) --
% updates a 3D occupancy grid (grid.min, grid.res, grid.occ, grid.cnt)
% with a point cloud 'cloud' taken from view point 'vp'.


if nargin < 3
    vp = [];
end


obs_occ = zeros(size(grid.occ));
obs_cnt = zeros(size(grid.occ));


for i=1:length(cloud)
    
    if mod(i,100)==0
        fprintf('o');
    end
    
    % negative observations
    if ~isempty(vp)
        ray_dir = cloud(i,:) - vp;
        ray_len = norm(ray_dir);
        ray_dir = ray_dir/ray_len;  % unit vector
        for j=0:grid.res/2:ray_len  % discretize ray from vp to cloud(i,:)
            p = vp + j*ray_dir;                 % point along ray
            c = ceil((p - grid.min)/grid.res);  % point cell
            if (c >= [1,1,1]) .* (c <= size(grid.occ))
                if obs_occ(c(1), c(2), c(3)) ~= 1  % not already marked as occupied
                    obs_occ(c(1), c(2), c(3)) = 0;
                    obs_cnt(c(1), c(2), c(3)) = 1;
                end
            else
                continue;
            end
            neighbors = zeros(6,3);
            neighbors(1,:) = c - [1,0,0];
            neighbors(2,:) = c + [1,0,0];
            neighbors(3,:) = c - [0,1,0];
            neighbors(4,:) = c + [0,1,0];
            neighbors(5,:) = c - [0,0,1];
            neighbors(6,:) = c + [0,0,1];
            for k=1:size(neighbors,1)
                n = neighbors(k,:);
                if (n >= [1,1,1]) .* (n <= size(grid.occ))
                    if obs_cnt(n(1), n(2), n(3)) == 0  % not already observed
                        obs_occ(n(1), n(2), n(3)) = 0;
                        obs_cnt(n(1), n(2), n(3)) = .5;
                    end
                end
            end
        end
    end
    
    % positive observations
    p = cloud(i,:);                     % observed point
    c = ceil((p - grid.min)/grid.res);  % point cell
    if (c >= [1,1,1]) .* (c <= size(grid.occ))
        obs_occ(c(1), c(2), c(3)) = 1;
        obs_cnt(c(1), c(2), c(3)) = 1;
    else
        continue;
    end
    neighbors = zeros(6,3);
    neighbors(1,:) = c - [1,0,0];
    neighbors(2,:) = c + [1,0,0];
    neighbors(3,:) = c - [0,1,0];
    neighbors(4,:) = c + [0,1,0];
    neighbors(5,:) = c - [0,0,1];
    neighbors(6,:) = c + [0,0,1];
    for k=1:size(neighbors,1)
        n = neighbors(k,:);
        if (n >= [1,1,1]) .* (n <= size(grid.occ))
            if obs_cnt(n(1), n(2), n(3)) == 0 || obs_occ(n(1), n(2), n(3)) == 0
                obs_occ(n(1), n(2), n(3)) = 1;
                obs_cnt(n(1), n(2), n(3)) = .5;
            end
        end
    end

end
        

% update occupancy grid
grid2 = grid;
grid2.occ = (grid.occ.*grid.cnt + obs_occ.*obs_cnt) ./ (grid.cnt + obs_cnt);
grid2.cnt = grid.cnt + obs_cnt;

fprintf('\n');



