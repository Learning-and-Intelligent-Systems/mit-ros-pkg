function occ_grid = scans_to_occupancy_grid(scans, views, resolution, grid_occ, grid_cnt)
% occ_grid = scans_to_occupancy_grid(scans, views, resolution, grid_occ, grid_cnt)

% find the first non-empty scan
for i0=1:length(scans)
    if ~isempty(scans{i0})
        break;
    end
end

% create occupancy grid
pmin = min(scans{i0});
pmax = max(scans{i0});
for i=i0+1:length(scans)
    if ~isempty(scans{i})
        pmin = min(pmin, min(scans{i}));
        pmax = max(pmax, max(scans{i}));
    end
end
if ~isempty(views)
    if nargin >= 5
        occ_grid = new_occupancy_grid(pmin-resolution, pmax+resolution, resolution, grid_occ, grid_cnt);
    elseif nargin >= 4
        occ_grid = new_occupancy_grid(pmin-resolution, pmax+resolution, resolution, grid_occ);
    else
        occ_grid = new_occupancy_grid(pmin-resolution, pmax+resolution, resolution);
    end
else
    occ_grid = new_occupancy_grid(pmin-resolution, pmax+resolution, resolution, .1, .1);
end

for i=i0:length(scans)
    if ~isempty(views)
        occ_grid = update_occupancy_grid(occ_grid, scans{i}, views(i,:));
    else
        occ_grid = update_occupancy_grid(occ_grid, scans{i});
    end
end

