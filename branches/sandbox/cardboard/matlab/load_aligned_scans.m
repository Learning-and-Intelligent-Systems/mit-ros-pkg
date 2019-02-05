function [aligned_scans aligned_camera_views] = load_aligned_scans(fdir)
%[aligned_scans aligned_camera_views] = load_aligned_scans(fdir)

run(sprintf('%s/aligned_views.m', fdir));  % loads aligned_camera_views

n = size(aligned_camera_views,1);
aligned_scans = cell(1,n);
for i=1:n
    try
        pcd = load_pcd(sprintf('%s/aligned_cloud%d.pcd', fdir, i));
        aligned_scans{i} = [pcd.X pcd.Y pcd.Z];
    end
end
