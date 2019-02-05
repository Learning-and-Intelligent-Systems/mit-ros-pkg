function build_model

addpath('/home/jglov/ros/branches/sandbox/chaos/matlab');
addpath('/home/jglov/bingham/matlab/tools');
addpath('/home/jglov/bingham/matlab');


resolution = .006;

% load scans
[scans camera_views] = load_scans(resolution);

% align scans
[aligned_scans aligned_camera_views alignments full_cloud] = ...
    align_scans(scans, camera_views, resolution, 0, 1);
while 1
    figure(1);
    plot_scans(aligned_scans);
    action = input('is model good enough? [y/n]', 's');
    if action=='n'
        [aligned_scans aligned_camera_views alignments full_cloud] = ...
            align_scans(aligned_scans, aligned_camera_views, resolution, 1, 1);
    elseif action=='y'
        break
    else
       fprintf('invalid action\n');
    end
end

% plot aligned camera views (sanity check)
plot_camera_views(aligned_camera_views);
axis tight

% cut off supporting plane
h = find_supporting_plane_height(full_cloud);
full_cloud = full_cloud(full_cloud(:,3)>h,:);
for i=1:length(aligned_scans)
    if ~isempty(aligned_scans{i})
        aligned_scans{i} = aligned_scans{i}(aligned_scans{i}(:,3)>h,:);
    end
end

% save model point clouds
full_cloud_pcd.columns = {'x', 'y', 'z'};
full_cloud_pcd.data = full_cloud;
save_pcd('full_cloud.pcd', full_cloud_pcd);
for i=1:length(aligned_scans)
    if ~isempty(aligned_scans{i})
        cloud_pcd.columns = {'x', 'y', 'z'};
        cloud_pcd.data = aligned_scans{i};
        save_pcd(sprintf('aligned_cloud%d.pcd',i), cloud_pcd);
    end
end

% save aligned camera views
f = fopen('aligned_views.m', 'w');
fprintf(f, 'aligned_camera_views = [];\n');
for i=1:size(aligned_camera_views,1)
    fprintf(f, 'aligned_camera_views(%d,:) = [%.4f, %.4f, %.4f];\n', i, ...
        aligned_camera_views(i,1), aligned_camera_views(i,2), aligned_camera_views(i,3));
end

