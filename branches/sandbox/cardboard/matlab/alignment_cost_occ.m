function f = alignment_cost_occ(w, cloud1, cloud2, occ_grid)
% f = alignment_cost_occ(w, cloud1, cloud2, occ_grid), where w = (x,y,z,theta), computes
% the alignment cost between cloud 1 and cloud2 transformed by w.


theta = w(4);
q = [cos(theta/2); 0; 0; sin(theta/2)];
cloud2w = transform_point_cloud(w(1:3), q, cloud2);

n1 = size(cloud1,1);
n2 = size(cloud2,1);


%fprintf('.');
%dbug
%figure(3);
%clf;
%hold on;
%plot3(cloud1(:,1), cloud1(:,2), cloud1(:,3), 'b.');
%plot3(cloud2w(:,1), cloud2w(:,2), cloud2w(:,3), 'r.');
%axis vis3d;
%axis equal;
%hold off;

d_tot = 0;
for i=1:n2
    p = cloud2w(i,:);                             % observed point
    c = ceil((p - occ_grid.min)/occ_grid.res);    % point cell
    if (c >= [1,1,1]) .* (c <= size(occ_grid.occ))
        d = -log(occ_grid.occ(c(1), c(2), c(3)));
        d_tot = d_tot + d;
    end

%    P2 = repmat(cloud2w(i,:), [n1 1]);
%    D = P2 - cloud1;
%    d = min(sum(D.*D, 2));
%    d_tot = d_tot + d;
end

f = d_tot;

end