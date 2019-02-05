function F = get_arm_model_features(Q,dQ,u)

n = size(Q,1);
d = size(Q,2);
%cQ = cos(Q);
%sQ = sin(Q);
Q2 = Q.^2;

X = ones(n,1);  % position features
for i=1:d
    m = size(X,2);
    %C = repmat(cQ(:,i), [1,m]);
    %S = repmat(sQ(:,i), [1,m]);
    Qi = repmat(Q(:,i), [1,m]);
    Q2i = repmat(Q2(:,i), [1,m]);
    X = [X, X.*Qi, X.*Q2i];
end
dx = size(X,2);

XV = [];  % squared-velocity features
for i=1:d
    for j=1:d
        XV = [XV, X.*repmat(dQ(:,i).*dQ(:,j), [1,dx])];
    end
end

XU = [];  % torque features
for i=1:d
    %XA = [XA, X.*repmat(ddQ(:,i), [1,dx])];
    XU = [XU, X.*repmat(u(:,i), [1,dx])];
end

F = [X,XV,XU];  % features
%df = size(F,2);
%W = zeros(df,d);
%for i=1:d
%    W(:,i) = ard_regression(F, u(:,i), .1);
%end




