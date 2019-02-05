% [X1,X2] = meshgrid(-pi:pi/16:+pi, -pi:pi/16:+pi);
% 
% Y = sin(X1).*sin(X2) + 0.1*randn(size(X1));
% 
% figure(1); imagesc(Y); drawnow;
% 
% x = [X1(:) X2(:)];
% y = Y(:);
% 
% covfunc = @covSEiso; 
% likfunc = @likGauss; sn = 0.1; hyp.lik = log(sn);
% 
% hyp2.cov = [0 ; 0];    
% hyp2.lik = log(0.1);
% %hyp2 = minimize(hyp2, @gp, -100, @infExact, [], covfunc, likfunc, x, y);
% %exp(hyp2.lik)
% %nlml2 = gp(hyp2, @infExact, [], covfunc, likfunc, x, y)
% 
% tic;
% [m s2] = gp(hyp2, @infExact, [], covfunc, likfunc, x, y, x);
% toc;
% 
% m = reshape(m, size(Y));
% 
% figure(2); imagesc(m);

d = 4;
n = 1000;
hyp = struct('cov', [0;0], 'lik', log(.1));
x = rand(n,d);
y = x(:,1) + x(:,2).^2 - sin(pi*x(:,3)) + exp(x(:,4));
for i=1:d
    z = repmat(i/d, [100,d]);
    j = 4;
    z(:,j) = (1:100)'/100;
    tic; [m s2] = gp(hyp, @infExact, [], @covSEiso, @likGauss, x, y, z); toc;
    figure(i);
    f = [m+2*sqrt(s2/d); flipdim(m-2*sqrt(s2/d),1)];
    fill([z(:,j); flipdim(z(:,j),1)], f, [7 7 7]/8);
    hold on; plot(z(:,j), m, 'LineWidth', 2); hold off
end




