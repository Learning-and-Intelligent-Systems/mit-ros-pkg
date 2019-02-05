function grid = new_occupancy_grid(grid_min, grid_max, grid_res, grid_occ, grid_cnt)
%grid = new_occupancy_grid(grid_min, grid_max, grid_res, grid_occ, grid_cnt)

grid_size = ceil((grid_max-grid_min)/grid_res);
if nargin < 4
    grid_occ = .5;  %.99
end
if nargin < 5
    grid_cnt = .25;
end

grid.min = grid_min;
grid.res = grid_res;
grid.occ = repmat(grid_occ, grid_size);
grid.cnt = repmat(grid_cnt, grid_size);  % dirichlet prior
