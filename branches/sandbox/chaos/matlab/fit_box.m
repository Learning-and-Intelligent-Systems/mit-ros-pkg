function box = fit_box(cloud, init_box)
%box = fit_box(cloud, init_box) -- fit a box to a point cloud

if nargin < 2
    bmin = min(cloud)';
    bmax = max(cloud)';
    bdims = bmax-bmin;
    btheta = 0;
    wbox = [bmin(1:2); bdims(1:2); bmax(3)-.8; btheta];  % constraint: z0=.8
    w2vec = @(w) [w(1:2);.8;w(3:6)];
    cost_fn = @(w) w(3)*w(4)*w(5) + alignment_cost_box([0;0;0;0], ...
              vector_to_box(w2vec(w)), cloud);
    params.sigma = [.02;.02;.02;.02;.01;.05];
    params.iter = 400;
    params.eta = .97;
    params.sigma_eta = .995;
    wbox = randomized_gradient_descent(wbox, cost_fn, params);
    wbox = fminsearch(cost_fn, wbox);
else
    bmin = init_box.pmin;
    bdims = init_box.dims;
    btheta = init_box.theta;
    wbox = [bmin(1); bmin(2); bdims(1); bdims(2); init_box.theta];
    w2vec = @(w) [w(1:2);.8;w(3:4);bdims(3);w(5)];
    cost_fn = @(w) w(3)*w(4)*bdims(3) + alignment_cost_box([0;0;0;0], ...
              vector_to_box(w2vec(w)), cloud);
    %params.sigma = .5*[.005;.005;.005;.005;.08];
    %wbox = randomized_gradient_descent(wbox, cost_fn, params);
    wbox = fminsearch(cost_fn, wbox);
end    

box = vector_to_box(w2vec(wbox));

