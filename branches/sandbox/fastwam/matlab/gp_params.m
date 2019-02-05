hit_policies = [];

% get initial data points
A = read_matrix('data_points.txt');

% ball x y vx vy vz sx sy sz paddle x y z vx vy vz nx ny nz
% add spin columns since they weren't previously saved
A = [A(:, 1:6) zeros(size(A,1), 3) A(:, 7:end)];

A = get_data_points(A);
reward = [0 0 0 1 0 1 1 0 1 0 1 0 1 1 1 1 1 1 1 0 1 1 0 1 ...
    1 0 1 1 1 0 0 0 0 0 0 1 0 0 0 1 0 0 1 1 0 0 ...
    1 0 0 0 1 0 0 0 0 1 0 1 0 1 1 0 1 0 1 1 0 ...
    1 1 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 ...
    0 0 0 0 0 0 0 0 1 0 1 0 1 0 0 0 0 0 0 0]';

data_pt = [A(:,1).*reward A(:,2:end)];
zero_index = find(data_pt(:,1) == 0);
data_pt(zero_index, :) = [];

num = 100;
delta = 0.1;
sigma = 1.0;
bkts = 5;

likfunc = @likGauss;
covfunc = @covSEiso;
hyp2.cov = log([1.0 1.0]);
hyp2.lik = log(sigma);
%hyp2 = minimize(hyp2, @gp, -100, @infExact, [], covfunc, likfunc, x, y);
%nlml2 = gp(hyp2, @infExact, [], covfunc, likfunc, x, y);

for t=1:10000
    input(':');
    
    last_swing = read_matrix('~/ros/last_swing.txt');
    %last_swing = data_pt(4,:);

    disp('Last swing:');
    disp(last_swing);
    temp = input('Enter 0 or 1 for last swing result: ');
    
    if temp == 0
        disp('Not a good swing, ignoring.');
    else
        data_pt = [data_pt; last_swing];
        x = data_pt(:, 1:8);

        hp = last_swing(1:8);
        
        % paddle x y z vx vy vz nx ny nz (9:17)
        for i=9:17
            y = data_pt(:,i);
            z = last_swing(1:8);
                        
            [~, ~, m, s2] = gp(hyp2, @infExact, [], covfunc, likfunc, x, y, z);
           
            hp = [hp normrnd(m, sqrt(s2))];
            
%             figure(2)
%             set(gca, 'FontSize', 24)
% 
%             f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];
%             plot([z_temp(:,i); flipdim(z_temp(:,i),1)], f, 'x');
%             hold on;
%             plot(z_temp(:,i), m, '.k');
%             hold off
% 
%             grid on
%             xlabel('input, x')
%             ylabel('output, y')

        end
        
        % update hit policies matrix
        ind = 0;
        % look in hit_policies for any previous policies that are close enough
        for i=1:size(hit_policies,1)
            temp = hit_policies(i, 1:8);
            
            % tolerances for ball params
            thresh = [0.6/bkts, 0.6/bkts, 1.5/bkts, 2.0/bkts, ...
                1.0/bkts, 1000000, 200, 1000000];
            
            if abs(temp - hp(1:8) < thresh)
                ind = i;
                break;
            end
        end
        
        if ind == 0
            hit_policies = [hit_policies; hp];
        else
            hit_policies(ind, 9:17) = hp(9:17);
        end
        
        disp('Ball parameters:');
        disp(hp(1:8));
        disp('Hit policy:');
        disp(hp(9:17));
        
        send_hit_policies(hit_policies(:, 1:8), hit_policies(:, 9:17));
        
    end
    

end