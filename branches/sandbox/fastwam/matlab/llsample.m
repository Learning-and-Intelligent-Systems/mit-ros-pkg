function [y s] = llsample(X,Y,x,r,y0,s0,w0)
% [y s] = llsample(X,Y,x,r) -- samples from a Gaussian Kernel
% Local Likelihood model with data points X,Y, gaussian kernel with width r,
% and sample points x (in the rows).
% 
% [y s] = llsample(X,Y,x,r,y0,s0,w0)  -- samples with prior mean y0,
% covariance s0, and weight w0.


nx = size(X,2);
ny = size(Y,2);
N = size(X,1);

if nargin < 7
    y0 = zeros(1,ny);
    s0 = zeros(ny);
    w0 = 0;
end

y = zeros(size(x,1),ny);     % sample means
if nargout >= 2
    s = zeros(ny,ny,size(x,1));  % sample covariance matrices
end
for i=1:size(x,1)
    xi = x(i,:);
    
    dx = repmat(xi, [N 1]) - X;
    dx = sqrt(sum(dx.*dx, 2));
    %dx = acos(abs(sum(repmat(xi, [N 1]).*X, 2)));
    
    w = exp(-(dx/r).^2);        % weights
    wi = find(w>=max(w)/50);
    w = w(wi);                  % truncate small weights
    Ywi = Y(wi,:);
    wtot = sum(w) + w0;
    wy = repmat(w, [1 ny]).*Ywi;
    y(i,:) = (y0*w0 + sum(wy)) / wtot;
    
    if nargout >= 2
        dy = Ywi - repmat(y(i,:), [length(w) 1]);
        dwy = repmat(w, [1 ny]).*dy;
        S = 0;
        for j=1:size(dwy,1)
            S = S + dwy(j,:)'*dwy(j,:);
        end
        s(:,:,i) = (s0*w0 + S) / wtot;
    end
end
