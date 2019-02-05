function [full_cloud, alignment] = fit_partial_models(cloud1, cloud2, occ_grid1)%, occ_grid2)
%[full_cloud, alignment] = fit_partial_models(cloud1, cloud2, occ_grid1) -- fits
%cloud2 to cloud1

cloud1_orig = cloud1;
cloud2_orig = cloud2;

cloud1 = cloud1(1:5:end,:);
cloud2 = cloud2(1:5:end,:);

t = @(pose,cloud) transform_point_cloud(pose(1:3),pose(4:7)/norm(pose(4:7)),cloud);
t2 = @(pose) t(pose,cloud2);
t2_orig = @(pose) t(pose,cloud2_orig);
cost_fn1 = @(pose) point_cloud_dist(cloud1, t2(pose)) + point_cloud_dist(t2(pose), cloud1);
cost_fn2 = @(pose) .0001 * alignment_cost_occ([0;0;0;0], [], t2(pose), occ_grid1);
cost_fn = @(pose) cost_fn1(pose) + cost_fn2(pose);

params.sigma = [.02;.02;.02;.1;.1;.1;.1];
%params.iter = 400;
%params.eta = .97;
%params.sigma_eta = .995;
params.plot_fn = @(pose) plot_scans({cloud1_orig, t2_orig(pose)});

u1 = mean(cloud1);
u2 = mean(cloud2);
du = u1-u2;

max_cost = inf;
done_aligning = 0;
while ~done_aligning
    q = normrnd(0,1,4,1);
    q = q/norm(q);
    pose = [du';q];
    
    pose = randomized_gradient_descent(pose, cost_fn, params);
    cost = cost_fn(pose)
    if cost > max_cost
        continue;
    end
    
    while 1
        action = input('Good enough? [y/n]','s');
        if action == 'y'
            done_aligning = 1;
            break;
        elseif action=='n'
            max_cost = cost;
            break;
        else
            fprintf('Invalid action\n');
        end
    end
end

% refine alignment
fprintf('Refining alignment...\n');
pose(4:7) = pose(4:7)/norm(pose(4:7));
pose = fminsearch(cost_fn, pose);
pose(4:7) = pose(4:7)/norm(pose(4:7));
plot_scans({cloud1_orig, t2_orig(pose)});

alignment = pose;
full_cloud = [cloud1_orig; t2_orig(alignment)];



