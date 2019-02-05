function [scans camera_views] = load_scans(resolution)
%[scans camera_views] = load_scans()  -- loads scans from the current
% directory in files cloud*.pcd and views.m.

views;  % load camera_views

scans = {};
figure(2);
clf;
for i=1:size(camera_views,1)
    P = load_pcd(sprintf('cloud%d.pcd',i));
    X = [P.X P.Y P.Z];
    Y = X(logical((abs(X(:,3)-1.05)<.20)),:);  %.09
    if size(Y,1) < 1000
        continue;
    end
    Y = downsample_point_cloud(Y, resolution);
    if size(Y,1) < 1000
        continue;
    end

    % cut off supporting plane
%     h = .95:.001:1.05;
%     A = zeros(size(h));
%     for j=1:length(h)
%         Z = Y(Y(:,3)>h(j), 1:2);
%         A(j) = prod(max(Z) - min(Z)) + j/1000;
%     end
%     figure(4);
%     plot(h, A);
%     [amin jmin] = min(A);
%     hmin = h(jmin);
%     Y = Y(Y(:,3)>hmin, :);

    % extract biggest cluster
    L = simple_cluster(Y, .05);
    h = hist(L,length(unique(L)));
    [biggest_cluster_size biggest_cluster] = max(h);
    if biggest_cluster_size < 1000
        continue;
    end
    scans{i} = Y(L==biggest_cluster,:);
    %scans{i} = Y;
    
    %figure(2);
    hold on;
    c = colormap('jet');
    plot3(scans{i}(:,1), scans{i}(:,2), scans{i}(:,3), '.', 'Color', c(mod(20*i,size(c,1))+1,:));
    hold off;
    axis vis3d;
    axis equal;
    drawnow;
    %input(':');
end

