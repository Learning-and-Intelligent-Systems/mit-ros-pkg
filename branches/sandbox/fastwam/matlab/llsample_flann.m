function [y s] = llsample_flann(X,findex,fparams,Y,x,r,y0,s0,w0)
% [y s] = llsample_flann(findex,fparams,Y,x,r) -- samples from a Gaussian Kernel
% Local Likelihood model with data points Y,Y, gaussian kernel with width r,
% and sample points y (in the rows).
% 
% [y s] = llsample_flann(findex,fparams,Y,x,r,y0,s0,w0)  -- samples with prior mean y0,
% covariance s0, and weight w0.

knn = 100;

%nx = size(X,2);
ny = size(Y,2);
N = size(X,1);

if nargin < 7
    y0 = zeros(1,ny);
    s0 = zeros(ny);
    w0 = 0;
end

y = zeros(size(x,1),ny);     % sample means
s = zeros(ny,ny,size(x,1));  % sample covariance matrices
for i=1:size(x,1)
    xi = x(i,:);
    
    %[nn_idx, dx2] = flann_search(findex, xi', knn, fparams);
    nn_idx = flann_search(findex, xi', knn, fparams);
    dx2 = sum((X(nn_idx,:) - repmat(xi, [knn 1])).^2, 2);
    
    %dx = repmat(xi, [N 1]) - X;
    %dx = sqrt(sum(dx.*dx, 2));
    %dx = acos(abs(sum(repmat(xi, [N 1]).*X, 2)));
    
    logw = -dx2/r^2;
    logw = logw - max(logw);
    w = exp(logw);
    
    %w = exp(-(dx/r).^2);        % weights
    %w = exp(-dx2/r^2);        % weights
    wi = find(w>=max(w)/50);
    w = w(wi);                  % truncate small weights
    Ywi = Y(nn_idx(wi),:);
    wtot = sum(w) + w0;
    wy = repmat(w, [1 ny]).*Ywi;
    y(i,:) = (y0*w0 + sum(wy)) / wtot;
    dy = Ywi - repmat(y(i,:), [length(w) 1]);
    dwy = repmat(w, [1 ny]).*dy;
    S = 0;
    for j=1:size(dwy,1)
        S = S + dwy(j,:)'*dwy(j,:);
    end
    s(:,:,i) = (s0*w0 + S) / wtot;
end
