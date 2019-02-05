function merge_partial_models(dir1, dir2, dir3, dir4)
%merge_partial_models -- fit partial scans in multiple (2-4) folders together

n = nargin;

if n < 2
    fprintf('Error: need at least two partial models to merge!\n');
    return
end

dirs = [];
dirs{1} = dir1;
dirs{2} = dir2;
if nargin >= 3
    dirs{3} = dir3;
elseif nargin >= 4
    dirs{4} = dir4;
end

n = length(dirs);

% load full point clouds
pcds = [];
for i=1:n
    pcds{i} = load_pcd(sprintf('%s/full_cloud.pcd', dirs{i}));
end

% align full point clouds
full_pcd = pcds{1};
alignments = zeros(n,7);
alignments(1,:) = [0,0,0,1,0,0,0];
for i=2:n
    alignments(i,:) = align_pcds(full_pcd, pcds{i});
    t = alignments(i,1:3);
    T = repmat(t, [length(pcds{i}.X), 1]);
    q = alignments(i,4:7);
    R = quaternion_to_rotation_matrix(q);
    cloud = T + [pcds{i}.X, pcds{i}.Y, pcds{i}.Z]*R';
    pcds{i}.X = cloud(:,1);  pcds{i}.Y = cloud(:,2);  pcds{i}.Z = cloud(:,3);
    pcds{i}.data = populate_pcd_data(pcds{i});
    full_pcd.data = [full_pcd.data; pcds{i}.data];
    full_pcd = populate_pcd_fields(full_pcd.columns, full_pcd.data);
end

% save merged full cloud
save_pcd('full_cloud.pcd', full_pcd);

% save alignments
save alignments.mat alignments
f = fopen('alignments.txt', 'w');
for i=1:n
    fprintf(f, '%f ', alignments(i,:));
    fprintf(f, '\n');
end
fclose(f);

% load scans, apply alignments, and save
fprintf('Aligning scans');
cnt = 0;
for i=1:n
    q = alignments(i,4:7);
    R = quaternion_to_rotation_matrix(q);
    t = alignments(i,1:3);
    files = dir([dirs{i} '/aligned_cloud*.pcd']);
    num_clouds = length(files);
    for j=1:num_clouds
        cnt = cnt+1;
        pcd = load_pcd(sprintf('%s/aligned_cloud%03d.pcd',dirs{i},j));
        save_pcd(sprintf('cloud%03d.pcd', cnt), transform_pcd(pcd, t, q));
%         cloud = [pcd.X pcd.Y pcd.Z];
%         vp = pcd.vp;
%         %pcd = [];
%         pcd.vp = [t + vp(1:3)*R', quaternion_mult(vp(4:7), q)];
%         T = repmat(t, [size(cloud,1) 1]);
%         cloud = T + cloud*R';
%         pcd.X = cloud(:,1);
%         pcd.Y = cloud(:,2);
%         pcd.Z = cloud(:,3);
%         pcd.data = populate_pcd_data(pcd);
%         save_pcd(sprintf('cloud%03d.pcd', cnt), pcd);
        fprintf('.');
    end
end
fprintf('done\n');






