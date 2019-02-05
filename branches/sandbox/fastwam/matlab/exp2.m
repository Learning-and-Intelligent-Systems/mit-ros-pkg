%function exp2


%[T,Q,dQ,ddQ,U,L] = load_all_arm_data('.');

num_swings = max(L)+1;

r = .05;
for i=20:num_swings
%     for j=1:3
%         X_train = [Q(L~=i-1,1:3), dQ(L~=i-1,1:3), U(L~=i-1,j)];
%         Y_train = ddQ(L~=i-1,j);
%         X_test = [Q(L==i-1,1:3), dQ(L==i-1,1:3), U(L==i-1,j)];
%         Y_test = ddQ(L==i-1,j);
%         
%         % compare accelerations
%         Y = llsample(X_train, Y_train, X_test, r);
%         figure(1);
%         plot(Y_test);
%         hold on;
%         plot(Y, 'r-');
%         hold off;
%         
%         input(sprintf('i=%d,j=%d:', i, j));
%     end

    % build flann indices
    fprintf('building flann indices...\n');
    params.algorithm = 'kdtree';
    params.trees = 8;
    params.checks = 64;
    flann_set_distance_type(1);
    
    %params.algorithm = 'kmeans';
    %params.branching = 2;
    %params.iterations = -1;
    %params.cb_index = 0;
    
    findices = {};
    fparams = {};
    for j=1:3
        F = [Q(L~=i-1,1:3), dQ(L~=i-1,1:3), U(L~=i-1,j)];
        [index, p] = flann_build_index(F', params);
        findices{j} = index;
        fparams{j} = p;
    end
    findices

    % compare trajectories
    fprintf('simulating trajectory...\n');
    t = find(L==i-1, 1);
    [q dq] = simulate_trajectory_ll(Q(t,1:3), dQ(t,1:3), T(L==i-1), U(L==i-1,1:3), ...
                                    findices, fparams, Q(L~=i-1,1:3), dQ(L~=i-1,1:3), ddQ(L~=i-1,1:3), U(L~=i-1,1:3), r, Q(L==i-1,1:3));
%                                    Q(L~=i-1,1:3), dQ(L~=i-1,1:3), ddQ(L~=i-1,1:3), U(L~=i-1,1:3), r, Q(L==i-1,1:3));

end

