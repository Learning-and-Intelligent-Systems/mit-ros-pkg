function align_turntable_scans(true_origin)
%align_turntable_scans() -- aligns point clouds on a turn table

%table_pos;  % load origin estimate

% find turntable transform
pcd = load_pcd(sprintf('cloud%03d.pcd',1));
[origin, n] = find_turntable(pcd);
r3 = n;
r2 = ((eye(3) - n'*n)*rand(3,1))';
r2 = r2/norm(r2);
r1 = cross(r2,r3);
R = [r1;r2;r3];

dtheta = pi/18;
table_radius = .1;
z_cutoff = .015;

files = dir('cloud*.pcd');
num_clouds = length(files);

% load scans
fprintf('Loading scans');
clouds = [];
clouds_idx = [];
for i=1:num_clouds
    pcd = load_pcd(sprintf('cloud%03d.pcd',i));
    cloud = [pcd.X pcd.Y pcd.Z];
    clouds_idx{i} = 1:length(pcd.X);
    
    % transform into table coordinates
    cloud = cloud - repmat(origin, [size(cloud,1),1]);
    cloud = cloud*R';
    
    % get points above tabletop
    clouds_idx{i} = clouds_idx{i}(cloud(:,3) > z_cutoff);
    clouds{i} = cloud(clouds_idx{i}, :);
    clouds_idx{i} = clouds_idx{i}(sqrt(sum([clouds{i}(:,1), clouds{i}(:,2)].^2, 2)) < table_radius);
    
    % erode (remove boundary points)
    w = pcd.width;
    h = pcd.height;
    B = zeros(w,h);
    B(clouds_idx{i}) = 1;
    B = imerode(B,strel('disk',2));
    clouds_idx{i} = find(B);
    
    % extract cloud
    clouds{i} = cloud(clouds_idx{i}, :);
    fprintf('.');
end
fprintf('done\n');

% get viewpoint (same for every scan): cam -> model
%vp = pcd.vp;
vp(1:3) = -origin*R';
vp(4:7) = rotation_matrix_to_quaternion(R);

% optimize origin
if nargin < 1
    fprintf('Optimizing origin');
    xy_origin = fminsearch(@(xy) origin_eval([xy 0], clouds, dtheta), [0,0]);
    fprintf('done\n');
    origin = [xy_origin 0]
else
    origin = true_origin;
end

% align scans based on new origin
aligned_clouds = turntable_apply_origin(clouds, origin, dtheta);

% update viewpoints for aligned scans
viewpoints = zeros(num_clouds, 7);
for i=1:num_clouds,
    theta = -(i-1)*dtheta;
    q = [cos(theta/2), 0, 0, sin(theta/2)];
    R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0,0,1];
    viewpoints(i,:) = [(vp(1:3) - origin)*R', quaternion_mult(q, vp(4:7))];
end

% save aligned scans
fprintf('Saving aligned scans');
for i=1:num_clouds
    pcd = load_pcd(sprintf('cloud%03d.pcd',i));
    ch_x = find(strcmp(pcd.columns, 'x'));
    ch_y = find(strcmp(pcd.columns, 'y'));
    ch_z = find(strcmp(pcd.columns, 'z'));
    pcd.data = pcd.data(clouds_idx{i},:);
    pcd.data(:,ch_x) = aligned_clouds{i}(:,1);
    pcd.data(:,ch_y) = aligned_clouds{i}(:,2);
    pcd.data(:,ch_z) = aligned_clouds{i}(:,3);
    pcd.vp = viewpoints(i,:);
    save_pcd(sprintf('aligned_cloud%03d.pcd', i), pcd);
    fprintf('+');
end
fprintf('done\n');

% plot aligned clouds and viewpoints
figure(1);
[SX SY SZ] = sphere(20);
C = colormap;
plot3([0 0], [0 0], [-.1 .2], 'k-', 'LineWidth', 5);
hold on;
for i=1:num_clouds
    X = aligned_clouds{i};
    plot3(X(:,1), X(:,2), X(:,3), '.', 'Color', C(mod(ceil(i*2),64)+1,:));
    surf(SX/100 + viewpoints(i,1), SY/100 + viewpoints(i,2), SZ/100 + viewpoints(i,3));
    axis vis3d;
    axis equal;
end
hold off;


