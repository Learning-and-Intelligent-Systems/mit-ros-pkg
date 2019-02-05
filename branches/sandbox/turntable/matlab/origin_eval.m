function f = origin_eval(origin, clouds, dtheta)

points_per_cloud = 300;

X = turntable_apply_origin(clouds, origin, dtheta);

d = 0;
for i=2:2:length(clouds)
    step1 = max(round(size(clouds{i},1) / points_per_cloud), 1);
    X1 = X{i}(1:step1:end,:);
    n1 = size(X1,1);
    for j=[i-1,i+1] %2:2:length(clouds)
        if j>length(clouds)
            j = j - length(clouds);
        end
        step2 = max(round(size(clouds{j},1) / points_per_cloud), 1);
        X2 = X{j}(1:step2:end,:);
        d = d + point_cloud_dist(X1, X2, .01) / n1;
    end
end
fprintf('.');

f = d;
