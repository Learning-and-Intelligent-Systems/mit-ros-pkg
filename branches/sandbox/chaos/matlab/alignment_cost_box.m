function f = alignment_cost_box(w, box, cloud)
%f = alignment_cost_box(w, box, cloud) -- computes the alignment score for
%a transformed cloud w.r.t to a box (box.pmin, box.dims, box.theta)

% transform the point cloud by w
theta = w(4);
q = [cos(theta/2); 0; 0; sin(theta/2)];
cloud2 = transform_point_cloud(w(1:3), q, cloud);

% rotate the box and the point cloud by -box.theta
q = [cos(-box.theta/2); 0; 0; sin(-box.theta/2)];
cloud = cloud2 - repmat(box.pmin', [size(cloud,1) 1]);
cloud = transform_point_cloud([0;0;0], q, cloud);
R = quaternionToRotationMatrix(q);
bmin = [0;0;0];   %R*box.pmin;
bmax = box.dims;  %bmin + box.dims;

% compute the sum of squared point distances from the cloud to the box
d_tot = 0;
for i=1:size(cloud,1)
    d = point_box_dist_3d(cloud(i,:), bmin, bmax, 0);
    d_tot = d_tot + d*d;
end

f = d_tot;

%dbug
%fprintf('f = %f + %f\n', f, prod(box.dims));
%figure(3);
clf;
hold on;
%plot3(cloud2(:,1), cloud2(:,2), cloud2(:,3), 'b.');
%plot_box(box);
plot3(cloud(:,1), cloud(:,2), cloud(:,3), 'b.');
box2 = box;  box2.pmin = bmin;  box2.theta = 0;  plot_box(box2);
axis vis3d;
axis equal;
hold off;
axis tight;
drawnow;

