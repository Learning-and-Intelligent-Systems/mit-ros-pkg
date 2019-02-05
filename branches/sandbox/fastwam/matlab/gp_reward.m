function hit_policies = gp_reward(hit_policies, hits, rewards, advice, explore)
%hit_policies = gp_reward(hit_policies, hits, rewards, advice, explore)

if nargin < 4 %|| isempty(find(advice))  %dbug
    advice = [];
end
if nargin < 5
    explore = 1;
end

% paddle param range (x y z vx vy vz nx ny nz)
ppmin = [-.1, -.0001, -.0001,  1.0, -.0001,   0,   .999, -.1, -.5];
ppmax = [ .1,  .0001,  .0001,  2.0,  .0001,  .5,  1.001,  .1,  .5];

if explore
    bkts = [5 1 1 5 1 5 1 3 5];
else
    bkts = [10 1 1 10 1 10 1 5 10];
end

bbmin = [.2 .3 -4.0 -1 -3.5 -300 -300 -300];
bbmax = [.8 .9 -2.5  1 -2.5  300  300  300];
%bbmax = bbmin + 10*(bbmax - bbmin);

hpmin = [bbmin ppmin];
hpmax = [bbmax ppmax];

% %dbug!
% hit_policies = hits;
% for i=1:size(hits,1)
%     pp = ppmin + rand(1,8).*(ppmax-ppmin);
%     hit_policies(i,9:end) = [pp(1:6) 1 pp(7:8)];
% end
% %return


% GP-UCB params
%delta = 0.1;
advice_memory_length = 100;

i0 = max(1, size(advice,1) - advice_memory_length);
has_advice = ~isempty(find(advice(i0:end,:)));


x = hits;
%y = rewards' - .5;
y = rewards';

for i=1:size(x,2)
    x(:,i) = 2*(x(:,i) - hpmin(i)) / (hpmax(i) - hpmin(i)) - 1;
end

likfunc = @likGauss;
covfunc = @covSEiso;
hyp2.cov = log([2 .5]);
hyp2.lik = log(.1);
%hyp2 = minimize(hyp2, @gp, -100, @infExact, [], covfunc, likfunc, x, y);
%nlml2 = gp(hyp2, @infExact, [], covfunc, likfunc, x, y);


if ~has_advice

    % compute the hit param combinations
    comb = [];
    for i=1:length(ppmin)
        if bkts(i)==1
            values = 0;
        else
            values = 2/bkts(i)*(rand(1,bkts(i)) + (0:bkts(i)-1)) - 1;
        end
        if i==1
            values = values';
        else
            values = repmat(values, size(comb,1), 1);
            values = reshape(values, numel(values), 1);
        end
        comb = [repmat(comb, bkts(i), 1), values];
    end

    % compute a new hit policy for every past hit
    t = size(hits,1);
    hit_policies = zeros(t,17);
    for i=1:t
        i
        z = [repmat(x(i,1:8), size(comb, 1), 1), comb];

        [~, ~, m, s2] = gp(hyp2, @infExact, [], covfunc, likfunc, x, y, z);

        if explore
            beta = log(t);  %2*log(prod(bkts)*t^2*pi^2 / (6*delta));
            [~,index] = max(m(:) + sqrt(beta)*sqrt(s2(:)));
        else
            [~,index] = max(m(:));
        end

        hit_policies(i,:) = .5*(z(index, :) + 1).*(hpmax - hpmin) + hpmin;
    end
    last_hp = z(index,:)

    
else  % use advice

    % just change the hit policies table for balls similar to the last ball
    bbthresh = (bbmax - bbmin)/2;
    t = size(hits,1);
    for i=1:t
        i
        if 1 %abs(hits(i,1:8) - hits(end,1:8)) < bbthresh
            
            % find advice for this ball
            np = length(bkts);
            advice_bounds = [-ones(1,np); ones(1,np)];
            j0 = max(1, size(advice,1) - advice_memory_length);
            for j=j0:size(advice,1)
                for k=1:np
                    if abs(advice(j,k)) > 0
                        if abs(advice(j,k)) > 1 || min(abs(hits(i,1:8) - hits(j,1:8)) < bbthresh)
                            if advice(j,k) > 0  % increase hit param k
                                advice_bounds(1,k) = max(advice_bounds(1,k), x(j,8+k));
                            else  % decrease hit param k
                                advice_bounds(2,k) = min(advice_bounds(2,k), x(j,8+k));
                            end
                        end
                    end
                end
            end
            
            % make advice bounds be valid (min <= max)
            for k=1:np
                if advice_bounds(1,k) > advice_bounds(2,k)
                    a = (advice_bounds(1,k) + advice_bounds(2,k)) / 2;
                    advice_bounds(1,k) = a;
                    advice_bounds(2,k) = a;
                end
            end
            
            % compute the hit param combinations
            comb = [];
            for k=1:np
                if bkts(k)==1
                    values = 0;
                else
                    v0 = advice_bounds(1,k);
                    v1 = advice_bounds(2,k);
                    values = (v1-v0)/bkts(k)*(rand(1,bkts(k)) + (0:bkts(k)-1)) + v0;
                end
                if k==1
                    values = values';
                else
                    values = repmat(values, size(comb,1), 1);
                    values = reshape(values, numel(values), 1);
                end
                comb = [repmat(comb, bkts(k), 1), values];
            end

            % update hit policy with GP-UCB heuristic
            z = [repmat(x(i,1:8), size(comb, 1), 1), comb];

            [~, ~, m, s2] = gp(hyp2, @infExact, [], covfunc, likfunc, x, y, z);

            if explore
                beta = log(t);  %2*log(prod(bkts)*t^2*pi^2 / (6*delta));
                [~,index] = max(m(:) + sqrt(beta)*sqrt(s2(:)));
            else
                [~,index] = max(m(:));
            end
            hit_policies(i,:) = .5*(z(index, :) + 1).*(hpmax - hpmin) + hpmin;
        end
    end
end



% else  % use advice
%     
%     ai = find(advice,1);
%     a = advice(ai);
% 
%     % just change the hit policies table for balls similar to the last ball
%     bbthresh = (bbmax - bbmin)/2;
%     t = size(hits,1);
%     for i=1:t
% 
%         if abs(hits(i,1:8) - hits(end,1:8)) < bbthresh
%             if a > 0
%                 irange = linspace(min(x(i,8+ai)+.02, 1), 1, 100);
%             else
%                 irange = linspace(-1, max(x(i,8+ai)-.02, -1), 100);
%             end
% 
%             % keep policy except for advice hit param
%             z = repmat(x(i,:), [length(irange),1]);
%             z(:,8+ai) = irange;
% 
%             [~, ~, m, s2] = gp(hyp2, @infExact, [], covfunc, likfunc, x, y, z);
% 
%             beta = log(t);  %2*log(prod(bkts)*t^2*pi^2 / (6*delta));
%             [~,index] = max(m(:) + sqrt(beta)*sqrt(s2(:)));
% 
%             hit_policies(i,:) = .5*(z(index, :) + 1).*(hpmax - hpmin) + hpmin;
%         end
%     end
% end




X=x;
Y=y;
for iter=size(X,1)
    x = X(1:iter,:);
    y = Y(1:iter);
    for i=1:9
    %     pp = (ppmax + ppmin) / 2;
        %irange = linspace(ppmin(:,i), ppmax(:,i), 100);

        %pp = zeros(size(ppmin));
        %pp = x(1,9:end);
        pp = x(end,9:end);
        %pp = last_hp(9:end);
        %bb = x(1,1:8);
        %bb = mean(x(:,1:8));
        bb = x(end,1:8);
        hp = [bb pp];

        irange = linspace(-1, 1, 100);

        z = repmat(hp, [length(irange),1]);
        z(:,8+i) = irange;

        [~, ~, m, s2] = gp(hyp2, @infExact, [], covfunc, likfunc, x, y, z);

        figure(10+i);
        set(gca, 'FontSize', 24)

        f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];
        plot([z(:,8+i); flipdim(z(:,8+i),1)], f, 'x');
        hold on;
        plot(z(:,8+i), m, '.k');
        plot(x(:,8+i), y, 'rx');
        plot(x(end,8+i), y(end), 'mx', 'LineWidth', 3, 'MarkerSize', 10);
        hold off
        xlim([-1,1]);
        ylim([-1,1]);
    end
end
