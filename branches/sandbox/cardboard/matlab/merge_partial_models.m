function merge_partial_models

addpath('/home/jglov/ros/branches/sandbox/cardboard/matlab');
addpath('/home/jglov/bingham/matlab/tools');
addpath('/home/jglov/bingham/matlab');

pcd1 = load_pcd('1/full_cloud.pcd');
pcd2 = load_pcd('2/full_cloud.pcd');

[scans1 views1] = load_aligned_scans('1');
[scans2 views2] = load_aligned_scans('2');

% get object scans
obj_scans1 = {};
for i=1:length(scans1)
    if ~isempty(scans1{i})
        cloud = scans1{i};
        h = find_supporting_plane_height(cloud);
        obj_scans1{i} = cloud(cloud(:,3)>h,:);
    end
end
obj_scans2 = {};
for i=1:length(scans2)
    if ~isempty(scans2{i})
        cloud = scans2{i};
        h = find_supporting_plane_height(cloud);
        obj_scans2{i} = cloud(cloud(:,3)>h,:);
    end
end

% move pcd1, scans1, obj_scans1, and views1 to origin
u1 = mean(pcd1.data);
for i=1:length(scans1)
    if ~isempty(scans1{i})
        scans1{i} = scans1{i} - repmat(u1, [size(scans1{i},1),1]);
        obj_scans1{i} = obj_scans1{i} - repmat(u1, [size(obj_scans1{i},1),1]);
    end
end
views1 = views1 - repmat(u1, [size(views1,1),1]);
pcd1.data = pcd1.data - repmat(u1, [size(pcd1.data,1),1]);

resolution = .006;
occ_grid1 = scans_to_occupancy_grid(scans1, views1, resolution);
%occ_grid2 = scans_to_occupancy_grid(scans2, views2, resolution);

[full_cloud alignment] = fit_partial_models(pcd1.data, pcd2.data, occ_grid1);

[xxx A] = transform_point_cloud(alignment(1:3), alignment(4:7), pcd2.data);
v2 = [views2 ones(size(views2,1),1)]*A';
views2 = v2(:,1:3);

plot_camera_views(views1);
plot_camera_views(views2);

% get aligned scans and views
n = 0;
obj_scans = {};
views = [];
for i=1:length(scans1)
    if ~isempty(scans1{i})
        n = n+1;
        obj_scans{n} = obj_scans1{i};
        views(n,:) = views1(i,:);
    end
end
for i=1:length(scans2)
    if ~isempty(scans2{i})
        n = n+1;
        cloud = [obj_scans2{i} ones(size(obj_scans2{i},1),1)]*A';
        obj_scans{n} = cloud(:,1:3);
        views(n,:) = views2(i,:);
    end
end

% save aligned model scans and camera views
f = fopen('views.m', 'w');
fprintf(f, 'camera_views = [];\n');
for i=1:length(obj_scans)
    cloud_pcd.columns = {'x', 'y', 'z'};
    cloud_pcd.data = obj_scans{i};
    save_pcd(sprintf('cloud%d.pcd',i), cloud_pcd);
    fprintf(f, 'camera_views(%d,:) = [%.4f, %.4f, %.4f];\n', i, ...
        views(i,1), views(i,2), views(i,3));
end
fclose(f);

% compute full occupancy grid and convert to point cloud
occ_grid = scans_to_occupancy_grid(obj_scans, views, resolution);
occ_cloud = occupancy_grid_to_point_cloud(occ_grid);
occ_cloud = hollow_point_cloud(occ_cloud, occ_grid, views);

% remove outliers
L = simple_cluster(occ_cloud, 3*resolution);
h = hist(L,length(unique(L)));
[biggest_cluster_size biggest_cluster] = max(h);
occ_cloud = occ_cloud(L==biggest_cluster,:);

plot_scans({occ_cloud});

% save full cloud
cloud_pcd.columns = {'x', 'y', 'z'};
cloud_pcd.data = occ_cloud;
save_pcd(sprintf('full_cloud.pcd'), cloud_pcd);







