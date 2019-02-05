function f = alignment_cost(w, cloud1, cloud2)
% f = alignment_cost(w, cloud1, cloud2), where w = (x,y,z,theta), computes
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
% for i=1:n1
%     P1 = repmat(cloud1(i,:), [n2 1]);
%     D = P1 - cloud2w;
%     d = min(sum(D.*D, 2));
%     d_tot = d_tot + d;
% end
for i=1:n2
    P2 = repmat(cloud2w(i,:), [n1 1]);
    D = P2 - cloud1;
    d = min(sum(D.*D, 2));
    d_tot = d_tot + d;
end

f = d_tot;

end