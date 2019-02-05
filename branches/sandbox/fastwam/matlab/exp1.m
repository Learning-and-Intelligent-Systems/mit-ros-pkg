
i = 2;  % which joint to model
alpha = .05;  % smoothing


% load and clean the data set
load; %[Q,u] = load_all_arm_data('../data/1025'); Q = Q(50000:end,:); u = u(50000:end,:); save;
[Q,dQ,ddQ,u] = differentiate_arm_data(Q,u);
[Q,dQ,ddQ,u] = clean_arm_data(Q,dQ,ddQ,u);

% split the data into two halves
n = floor(size(Q,1)/2);
Q2 = Q(n+1:end,:);
dQ2 = dQ(n+1:end,:);
ddQ2 = ddQ(n+1:end,:);
u2 = u(n+1:end,:);
Q = Q(1:n,:);
dQ = dQ(1:n,:);
ddQ = ddQ(1:n,:);
u = u(1:n,:);

% get features on the first half
X = get_arm_model_features(Q(:,1:4), dQ(:,1:4), u(:,1:4));
y = ddQ(:,1:4);

% get features on the second half
X2 = get_arm_model_features(Q2(:,1:4), dQ2(:,1:4), u2(:,1:4));
y2 = ddQ2(:,1:4);

% train on first half
basis = [];
E = zeros(1,50);
E2 = zeros(1,50);
for j=1:50
    [basis, err] = adaptive_selection(basis, X(1:10:end,:), y(1:10:end,i), 10)
    E(j) = err;
    
    % plot current model
    b = regress(y(:,i), X(:,basis));
    figure(1);
    plot(smooth_signal(y(:,i), alpha), 'b-');
    hold on;
    plot(smooth_signal(X(:,basis)*b, alpha), 'r-');
    hold off;
    xlim([0,1000]);
    drawnow;
    
    E(j) = sqrt(mean((X(:,basis)*b - y(:,i)).^2));
    E2(j) = sqrt(mean((X2(:,basis)*b - y2(:,i)).^2));

    %figure(2);
    %plot(E(1:j));
    
    % plot test signal
    figure(3);
    plot(smooth_signal(y2(:,i), alpha), 'b-');
    hold on;
    plot(smooth_signal(X2(:,basis)*b, alpha), 'r-');
    hold off;
    xlim([0,1000]);
    drawnow;

    figure(4);
    plot(E2(1:j));

    input(':');
end

% get training and testing error
train_err = sqrt(mean((X(:,basis)*b - y(:,i)).^2))
test_err = sqrt(mean((X2(:,basis)*b - y2(:,i)).^2))

% plot test model
figure(3);
plot(smooth_signal(y2(:,i), alpha), 'b-');
hold on;
plot(smooth_signal(X2(:,basis)*b, alpha), 'r-');
hold off;
xlim([0,1000]);


