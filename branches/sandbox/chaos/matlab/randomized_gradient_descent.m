function [xmin fmin] = randomized_gradient_descent(x, cost_fn, params)
%[xmin fmin] = randomized_gradient_descent(x, cost_fn, params)

max_step_size = .01;
eta = .95;
iter = 200;
sigma = [.005;.005;.005;.08];
sigma_eta = .99;
%max_convergence_cnt = 30;
plot_fn = [];

if nargin >= 3
    if isfield(params, 'max_step_size')
        max_step_size = params.max_step_size;
    end
    if isfield(params, 'eta')
        eta = params.eta;
    end
    if isfield(params, 'iter')
        iter = params.iter;
    end
    if isfield(params, 'sigma')
        sigma = params.sigma;
    end
    if isfield(params, 'sigma_eta')
        sigma_eta = params.sigma_eta;
    end
    if isfield(params, 'plot_fn')
        plot_fn = params.plot_fn;
    end
end
    

f = cost_fn(x);
xmin = x;
fmin = f;
step_size = max_step_size;
convergence_cnt = 0;
for j=1:iter

    if mod(j,10)==0
        fprintf('.');
    end
    
    %convergence_cnt = convergence_cnt + 1;
    %if convergence_cnt == max_convergence_cnt
    %    break;
    %end

    gdir = normrnd(0,sigma);  % pick a random gradient direction
    gdir = gdir/norm(gdir);
    df = gdir*((cost_fn(x + step_size*gdir) - f));

    x2 = x - df;  % take a step along the gradient
    x2 = normrnd(x2,sigma);
    f2 = cost_fn(x2);
    
    if ~isempty(plot_fn)
        plot_fn(x2);
    end
    
    if f2<f %|| rand()<step_size/max_step_size
        x = x2;
        f = f2;
        if f2 < fmin
            xmin = x2;
            fmin = f2;
            %convergence_cnt = 0;
        end
    end
    step_size = step_size*eta;
    sigma = sigma*sigma_eta;
end



