function full_cloud = build_partial_model()


% make model point cloud
fprintf('Building model\n');

files = dir('aligned_cloud*.pcd');
num_clouds = length(files);

% save aligned scans
fprintf('Loading aligned scans');
full_pcd = load_pcd(sprintf('aligned_cloud%03d.pcd',1));
for i=2:num_clouds
    pcd = load_pcd(sprintf('aligned_cloud%03d.pcd',i));
    full_pcd.data = [full_pcd.data; pcd.data];
    fprintf('.');
end
fprintf('done\n');


full_pcd_orig = full_pcd;

% downsample and remove outliers
n = 10000;
step = round(size(full_pcd.data,1) / n);
full_pcd.data = full_pcd.data(1:step:end,:);
full_pcd = populate_pcd_fields(full_pcd.columns, full_pcd.data);
n = size(full_pcd.data,1);
density = zeros(1,n);
density_radius = .005;
full_cloud = [full_pcd.X, full_pcd.Y, full_pcd.Z];
for i=1:n
    X = repmat(full_cloud(i,:), [n 1]);
    density(i) = sum(sum((X - full_cloud).^2, 2) < density_radius^2) / n;
    if mod(i,100)==0
        fprintf('.');
    end
end
D = sort(density);
dthresh = D(round(n/20));  % remove bottom 5 percent of points
outliers = full_cloud(density < dthresh, :);
full_pcd.data = full_pcd.data(density >= dthresh, :);
full_pcd = populate_pcd_fields(full_pcd.columns, full_pcd.data);
fprintf('done\n');

% save full cloud
fprintf('Saving full cloud...');
save_pcd('full_cloud.pcd', full_pcd);
fprintf('done\n');

% plot full cloud
figure(2);
plot_pcd_color(full_pcd);
set(gca, 'Color', [0,0,.5]);
%hold on;
%outliers = full_cloud(density < dthresh, :);
%plot3(outliers(:,1), outliers(:,2), outliers(:,3), 'r.'); axis vis3d; axis equal;
%hold off;

%input('Hit enter to finish:');

