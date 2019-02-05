function [pose poses costs] = align_pcds(pcd1, pcd2, q0, viewpoint, range1_full)
%pose = align_pcds(pcd1, pcd2) -- aligns cloud2 with cloud1:
%cloud2_aligned = cloud2*R' + t, where pose = [t,q]

cloud1 = [pcd1.X, pcd1.Y, pcd1.Z];
cloud2 = [pcd2.X, pcd2.Y, pcd2.Z];
%rgb1 = [pcd1.R, pcd1.G, pcd1.B];
%rgb2 = [pcd2.R, pcd2.G, pcd2.B];
lab1 = reshape(RGB2Lab(pcd1.R, pcd1.G, pcd1.B), [length(pcd1.R),3]);
lab2 = reshape(RGB2Lab(pcd2.R, pcd2.G, pcd2.B), [length(pcd2.R),3]);

if nargin < 3
    q0 = [];
end

range1 = [];
if nargin >= 4
    range1 = cloud_to_range_image(cloud1, viewpoint, pi/1000);
else
    viewpoint = [];
end

if nargin < 5
    range1_full = [];
end
    

n1 = size(cloud1,1);
n2 = size(cloud2,1);
u1 = mean(cloud1);
u2 = mean(cloud2);
cloud1 = cloud1 - repmat(u1, [n1 1]);
cloud2 = cloud2 - repmat(u2, [n2 1]);
cloud1_full = cloud1;
cloud2_full = cloud2;
pcd1.X = cloud1_full(:,1);  pcd1.Y = cloud1_full(:,2);  pcd1.Z = cloud1_full(:,3);
pcd1.data = populate_pcd_data(pcd1);
pcd2.X = cloud2_full(:,1);  pcd2.Y = cloud2_full(:,2);  pcd2.Z = cloud2_full(:,3);
pcd2.data = populate_pcd_data(pcd2);


if ~isempty(viewpoint)
    viewpoint(1:3) = viewpoint(1:3) - u1;
end

% convert cloud1 to occ_grid
grid1_orig = cloud_to_occ(cloud1, .003);
grid1_orig.occ = grid1_orig.occ * (n2/n1);
grid1 = smooth_occ_grid(grid1_orig);
grid1 = smooth_occ_grid(grid1);
grid1.occ = min(grid1.occ + grid1_orig.occ, 1);

% convert cloud2 to occ_grid
grid2_orig = cloud_to_occ(cloud2, .003);
grid2 = smooth_occ_grid(grid2_orig);
grid2 = smooth_occ_grid(grid2);
grid2.occ = min(grid2.occ + grid2_orig.occ, 1);

cloud1 = cloud1(1:10:end,:);
cloud2 = cloud2(1:10:end,:);
lab1 = lab1(1:10:end,:);
lab2 = lab2(1:10:end,:);
n1 = size(cloud1,1);
n2 = size(cloud2,1);

qinv = @(q) [q(1) -q(2:4)];
q2R = @(q) quaternion_to_rotation_matrix(q/norm(q));
t1 = @(pose) (cloud1 - repmat(pose(1:3), [n1 1]))*q2R(qinv(pose(4:7)))';
t2 = @(pose) repmat(pose(1:3), [n2 1]) + cloud2*q2R(pose(4:7))';
%print_pose_fn = @(pose) 0*(fprintf('%f ', pose) + fprintf('\n'));
%cost_fn = @(pose) point_cloud_dist(cloud1, t2(pose)) + point_cloud_dist(t2(pose), cloud1);   % + .1*(norm(pose(4:7))-1)^2
cost_fn = @(pose) point_cloud_dist_lab(cloud1, t2(pose), lab1, lab2) + point_cloud_dist_lab(t2(pose), cloud1, lab2, lab1) ...
                 - .2*point_cloud_occ_fitness2(t2(pose));
%cost_fn = @(pose) -point_cloud_occ_fitness1(t1(pose)) - point_cloud_occ_fitness2(t2(pose)) - cloud_range_image_fitness(t2(pose));
%cost_fn = @(pose) -point_cloud_occ_fitness1(t1(pose)); % - cloud_range_image_fitness(t2(pose));
%cost_fn = @(pose) -cloud_range_image_fitness(t2(pose));

params.sigma = .5*[.02 .02 .02 .1 .1 .1 .1];
params.iter = 300;
%params.eta = .97;
%params.sigma_eta = .995;
%params.plot_fn = @plot_fn;

if ~isempty(q0)
    q0_iter = 1;
    min_pose = [];
    min_cost = inf;
    poses = [];
    costs = [];
end

while 1
    theta = 0;
    if isempty(q0)
        q = [1,0,0,0];
        while 1
            plot_fn([0,0,0,q]);
            if (input('done rotating?')==1)
                break;
            end
            q = q_input(q);
        end
        q = q/norm(q);
        t = [0,0,0];
        while 1
            plot_fn([t,q]);
            if (input('done translating?')==1)
                break;
            end
            t = t_input(t);
        end
    else
        t = [0,0,0];
        q = q0(q0_iter,:);
    end
    pose = [t,q];
    %t = mean(cloud1) - mean(t2(pose));
    %pose = [t,q];

    %d = point_cloud_dist(cloud1, t2(pose))
    
    %options = optimset('TypicalX', [.02;.02;.02;.1;.1;.1;.1], 'PlotFcns', @plot_fn, 'MaxIter', 200);
    %[pose, cost] = fminsearch(cost_fn, pose, options);
    
    [pose, cost, pose_history, cost_history] = randomized_gradient_descent(pose, cost_fn, params);
    pose(4:7) = pose(4:7)/norm(pose(4:7));
    
    if isempty(q0)
        while 1
            plot_fn(pose);
            if input(sprintf('cost = %f, align more? ', cost))==1
                params2.sigma = .5*[.02 .02 .02 .1 .1 .1 .1];
                params2.plot_fn = @plot_fn;
                [pose, cost] = randomized_gradient_descent(pose, cost_fn, params2);
                pose(4:7) = pose(4:7)/norm(pose(4:7));
            else
                break;
            end
        end

        % plot current alignment
        plot_fn(pose);
        if input('good enough? ')==1
            break;
        end
        
    else
        for i=1:length(cost_history)
            pose_history(i,4:7) = pose_history(i,4:7)/norm(pose_history(i,4:7));
        end
        
        poses = [poses; pose_history];
        costs = [costs, cost_history];
        
        %plot_fn(pose);
        %drawnow;
        %fprintf('\n  --> cost = %.2f, occ_cost1 = %.2f, occ_cost2 = %.2f, range_cost = %.2f\n', ...
        %        cost, -point_cloud_occ_fitness1(t1(pose)), -point_cloud_occ_fitness2(t2(pose)), -cloud_range_image_fitness(t2(pose)));
        fprintf('\n  --> cost = %.2f, occ_cost1 = %.2f, range_cost = %.2f\n', ...
                cost, -point_cloud_occ_fitness1(t1(pose)), -cloud_range_image_fitness(t2(pose)));
        if cost < min_cost
            min_cost = cost;
            min_pose = pose;
        end
        if q0_iter == size(q0,1)
            break;
        else
            q0_iter = q0_iter + 1;
        end
    end
end

if ~isempty(q0)
    fprintf(' --> min_cost = %.2f\n', min_cost);
    pose = min_pose;
    cost = min_cost;
    for i=1:size(poses,1)
        poses(i,:) = pose_to_output(poses(i,:));
    end
end

pose = pose_to_output(pose);


% compute affine transformation from original cloud2 to original cloud1
function pose_out = pose_to_output(pose_in)
    A1 = [eye(3), -u2'; 0,0,0,1];
    R = quaternion_to_rotation_matrix(pose_in(4:7));
    t = pose_in(1:3);
    A2 = [R, t'; 0,0,0,1];
    A3 = [eye(3), u1'; 0,0,0,1];
    A = A3*A2*A1;
    R = A(1:3,1:3);
    q = rotation_matrix_to_quaternion(R);
    t = A(1:3,4)';
    pose_out = [t,q];
end


function stop = plot_fn(pose, arg2, arg3)
    figure(1);
    %X1 = cloud1_full;
    X2 = repmat(pose(1:3), [size(cloud2_full,1) 1]) + cloud2_full*q2R(pose(4:7))';
    pcd2.X = X2(:,1);  pcd2.Y = X2(:,2);  pcd2.Z = X2(:,3);
    plot_pcd_color(pcd1,4);
    hold on;
    colormap = [zeros(257,1), (0:256)'/256, (0:256)'/256];
    plot_pcd_color(pcd2,4,colormap);
    hold off;
    %plot3(X1(:,1), X1(:,2), X1(:,3), 'b.');
    %hold on;
    %plot3(X2(:,1), X2(:,2), X2(:,3), 'r.');
    %hold off;
    %axis vis3d;
    %axis equal;
    title(sprintf('cost = %f', cost_fn(pose)));
    camorbit(-30,0); %dbug
    stop = 0;
end

function f = point_cloud_occ_fitness1(cloud) %, grid2)
    f = 0;
    for i=1:size(cloud,1)
        c = ceil((cloud(i,:) - grid2.min)/grid2.res);  % point cell
        if (c >= [1,1,1]) .* (c <= size(grid2.occ))
            f = f + min(grid2.occ(c(1),c(2),c(3)), 1);
            if c(3) < .25*size(grid2.occ,3)
                f = f + 1; %.1;
            end
        end
    end
    f = f / size(cloud,1);
end

function f = point_cloud_occ_fitness2(cloud) %, grid1)
    f = 0;
    for i=1:size(cloud,1)
        c = ceil((cloud(i,:) - grid1.min)/grid1.res);  % point cell
        if (c >= [1,1,1]) .* (c <= size(grid1.occ))
            f = f + min(grid1.occ(c(1),c(2),c(3)), 1);
            if c(3) < .25*size(grid1.occ,3)
                f = f + 1; %.1;
            end
        end
    end
    f = f / size(cloud,1);
end

% function f = range_image_fitness(cloud)
%     neg_cost = .01;
%     f = 0;
%     if isempty(viewpoint)
%         return;
%     end
%     origin = viewpoint(1:3);
%     P = cloud - repmat(origin, [size(cloud,1) 1]);
%     D = sqrt(sum(P.^2,2));
%     X = atan2(P(:,1), P(:,3));
%     Y = asin(P(:,2)./D);
%     for i=1:size(cloud,1)
%         c = ceil(([X(i),Y(i)] - range1.min)/range1.res);
%         if (c >= [1,1]) .* (c <= size(range1.image))
%             if range1.image(c(1),c(2)) < 0
%                 f = f - neg_cost;
%             elseif D(i) < range1.image(c(1),c(2))
%                 f = f - min((D(i) - range1.image(c(1),c(2)))^2, neg_cost);
%             end
%         else
%             f = f - neg_cost;
%         end
%     end
% end

function f = range_image_fitness(R1, R2, neg_cost)
    f = 0;
    if nargin < 3
        neg_cost = .003; %.01;
    end
    w = size(R1.image,1);
    h = size(R1.image,2);
    for x=1:w
        for y=1:h
            d1 = R1.image(x,y);
            if d1 > 0
                c = ceil((([x-.5,y-.5]*R1.res + R1.min) - R2.min)/R2.res);
            %if R1.image(x,y) < 0
            %    if (c >= [1,1]) .* (c <= size(R2.image))
            %        if R2.image(c(1),c(2)) >= 0
            %            f = f - neg_cost;
            %        end
            %    end
            %else
                if c(1) >= 1 && c(2) >= 1 && c(1) <= size(R2.image,1) && c(2) <= size(R2.image,2)  %(c >= [1,1]) .* (c <= size(R2.image))
                    d2 = R2.image(c(1),c(2));
                    if d2 < 0 || d1-d2 > .1
                        f = f - neg_cost;
                    else
                        f = f - min((d1 - d2)^2, .003);
                    end
                else
                    f = f - neg_cost;
                end
            end
        end
    end

end

function f = cloud_range_image_fitness(cloud)
    f = 0;
    if isempty(viewpoint)
        return;
    end
    RI = cloud_to_range_image(cloud, viewpoint, pi/1000);
    if isempty(range1_full)
        f = range_image_fitness(RI, range1) + range_image_fitness(range1, RI);
    else
        f = range_image_fitness(RI, range1_full, .0001) + range_image_fitness(range1, RI);
    end
end


function q2 = q_input(q)
    action = input('rotate? [1/2/3/4/5/6]: ');
    theta = pi/8;
    c = cos(theta/2);
    s = sin(theta/2);
    if action==1
        q2 = quaternion_mult([c,s,0,0], q);
    elseif action==2
        q2 = quaternion_mult([c,-s,0,0], q);
    elseif action==3
        q2 = quaternion_mult([c,0,s,0], q);
    elseif action==4
        q2 = quaternion_mult([c,0,-s,0], q);
    elseif action==5
        q2 = quaternion_mult([c,0,0,s], q);
    elseif action==6
        q2 = quaternion_mult([c,0,0,-s], q);
    else
        q2 = q;
    end
end


function t2 = t_input(t)
    action = input('translate? [1/2/3/4/5/6]: ');
    dt = .01;
    if action==1
        t2 = t + [-dt,0,0];
    elseif action==2
        t2 = t + [dt,0,0];
    elseif action==3
        t2 = t + [0,-dt,0];
    elseif action==4
        t2 = t + [0,dt,0];
    elseif action==5
        t2 = t + [0,0,-dt];
    elseif action==6
        t2 = t + [0,0,dt];
    else
        t2 = t;
    end
end



end