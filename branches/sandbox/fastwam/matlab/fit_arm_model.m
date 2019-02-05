function fit_arm_model(Q,u)

% compute derivatives
dQ = Q(2:end,:) - Q(1:end-1,:);
ddQ = dQ(2:end,:) - dQ(1:end-1,:);
Q = Q(2:end-1,:);
dQ = dQ(1:end-1,:);
u = u(2:end-1,:);

n = size(Q,1);
d = size(Q,2);
cQ = cos(Q);
sQ = sin(Q);

X = ones(n,1);  % position features
for i=1:d
    m = size(X,2);
    C = repmat(cQ(:,i), [1,m]);
    S = repmat(sQ(:,i), [1,m]);
    X = [X, X.*C, X.*S];
end
dx = size(X,2);

XV = [];  % squared-velocity features
for i=1:d
    for j=1:d
        XV = [XV, X.*repmat(dQ(:,i).*dQ(:,j), [1,dx])];
    end
end

XA = [];
for i=1:d
    XA = [XA, X.*repmat(ddQ(:,i), [1,dx])];
end

F = [X,XV,XA];  % features
df = size(F,2);
W = zeros(df,d);
for i=1:d
    W(:,i) = ard_regression(F, u(:,i), .1);
end




