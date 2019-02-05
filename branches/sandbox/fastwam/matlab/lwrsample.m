function [y b] = lwrsample(X,Y,x,r,lambda,affine)
% [y b] = lwrsample(X,Y,x,r) -- samples from a Gaussian Kernel
% Locally-Weighted Regression model with data points X,Y, gaussian kernel with width 1/r,
% and sample points x (in the rows).

if nargin < 5
    lambda = 1;
end
if nargin < 6
    affine = 1;
end

nx = size(X,2);
%ny = size(Y,2);
N = size(X,1);

if length(r)==1
    r = repmat(r, [1,nx]);
end
r2 = r.^2;

y = zeros(size(x,1),1);
b = zeros(size(x,1),nx+1);
    
for i=1:size(x,1)
    if mod(i,10)==0, fprintf('.'); end
    
    xi = x(i,:);
    
    dx = repmat(xi, [N 1]) - X;
    dx2 = (dx.*dx) * r2';
    
    dx2 = dx2 - min(dx2);  % offset for numerical stability
    
    w = exp(-dx2);        % weights
    wi = find(w>=max(w)/50);
    w = w(wi);                  % truncate small weights
    X2 = X(wi,:);
    Y2 = Y(wi);
    wtot = sum(w);

    % compute the offset, b(1), and center the Y's
    if affine
        b(i,1) = dot(Y2, w) / wtot;
        Y2 = Y2 - b(1);
    end
    
    b(i,2:end) = weighted_ridge_regression(X2, Y2, w, lambda);
    y(i) = b(i,1) + x(i,:)*b(i,2:end)';
end
fprintf('\n');

end


