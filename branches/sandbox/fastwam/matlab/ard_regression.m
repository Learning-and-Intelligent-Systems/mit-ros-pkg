function w = ard_regression(X,y,lambda)
%w = ard_regression(X,y,lambda) -- regresses y = X*w + N(0,sigma)
%with an ARD (automatic relevance detection) prior on the weights to
%encourage sparsity.

n = size(X,1);
d = size(X,2);
I = eye(n);

a = ones(d,1);  % gammas


mu_prev = zeros(d,1);
while 1
    A = diag(a);
    Sy = lambda*I + X*A*X';  % Sigma_y
    Sy_inv = inv(Sy);
    mu = A*X'*Sy_inv*y;
    
    bar(-sort(-abs(mu)));
    input(':');
    
    if norm(mu - mu_prev) < eps
        break
    end
    
    S = A - A*X'*Sy_inv*X*A;
    
    %a = mu.^2 + diag(S);  % EM update
    a = mu.^2 ./ (1 - diag(S)./a);  % MacKay update
    
    mu_prev = mu;
end

w = mu;

