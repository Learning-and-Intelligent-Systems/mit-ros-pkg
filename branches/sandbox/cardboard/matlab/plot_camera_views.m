function plot_camera_views(views)
%plot_camera_views(views)

[SX SY SZ] = sphere();

hold on;
for i=1:size(views,1)
    surf(.1*SX + views(i,1), .1*SY + views(i,2), .1*SZ + views(i,3));
end
hold off;

