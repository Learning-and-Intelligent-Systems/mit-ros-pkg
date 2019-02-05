function cloud = occupancy_grid_to_point_cloud(grid, occ_min)
%cloud = occupancy_grid_to_point_cloud(grid) -- convert an occupancy grid
%to a point cloud

grid_size = size(grid.occ);

if nargin >= 2
    if occ_min >= 0
        G = grid.occ > occ_min;
    else
        G = grid.occ < -occ_min;
    end
else
    G = (grid.occ > .5).*(grid.cnt >= 1);
end

cloud = [];
n = 0;
for i=1:grid_size(1)
    for j=1:grid_size(2)
        for k=1:grid_size(3)
            if G(i,j,k)
                n = n+1;
                cloud(n,:) = grid.min + [i-.5, j-.5, k-.5]*grid.res;
            end
        end
    end
end
    
