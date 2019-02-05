function plot_box(box)
%plot_box(box)


x0 = box.pmin(1);
y0 = box.pmin(2);
z0 = box.pmin(3);

dx = box.dims(1);
dy = box.dims(2);
dz = box.dims(3);

q = [cos(box.theta/2); 0; 0; sin(box.theta/2)];
R = quaternionToRotationMatrix(q);

p000 = [x0;y0;z0];
p100 = p000 + R*[dx;0;0];
p010 = p000 + R*[0;dy;0];
p001 = p000 + R*[0;0;dz];
p110 = p100 + R*[0;dy;0];
p101 = p100 + R*[0;0;dz];
p011 = p010 + R*[0;0;dz];
p111 = p110 + R*[0;0;dz];

e = {};
e{1} = [p000 p100];
e{2} = [p000 p010];
e{3} = [p000 p001];
e{4} = [p100 p110];
e{5} = [p100 p101];
e{6} = [p010 p110];
e{7} = [p010 p011];
e{8} = [p110 p111];
e{9} = [p011 p111];
e{10} = [p001 p101];
e{11} = [p001 p011];
e{12} = [p101 p111];

hold on;
for i=1:length(e)
    plot3(e{i}(1,:), e{i}(2,:), e{i}(3,:), 'k-', 'LineWidth', 3);
end
hold off;
