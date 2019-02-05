X = [];
n = 1;
iter = 200;
max_step_size = 5;  % new
eta = .95;
sigma = [2;1;1];
for i=1:size(C,1)
    x = [i;10;10];
    xmin = x;
    fmin = cost(x);
    step_size = max_step_size;
    for j=1:iter
        %[f,df] = cost(x);
        %if norm(df) > 0
        %    df = df/norm(df);

        % new
        f = cost(x);
        gdir = normrnd(0,sigma);  % pick a random gradient direction
        gdir = 3*gdir/norm(gdir);
        if sum(((x+gdir) < 1) + ((x+gdir) > size(C)'))
            continue;
        end
        df = gdir*((cost(x + gdir) - f)/9);
        
        %df = dot(gdir,df)*gdir;
            
        x2 = x - step_size*df;  % take a step along the gradient
        %end
        x2 = normrnd(x2,sigma);
        x2 = min(max(x2,[1;1;1]),size(C)');
        f2 = cost(x2);
        %if f2<f || rand()<step_size/max_step_size
        if f2<f %|| rand()<step_size/max_step_size
            x = x2;
            if f2 < fmin
                xmin = x2;
                fmin = f2;
            end
        end
        step_size = step_size*eta;
    end
    X(:,n) = xmin; n=n+1;
    fprintf('.');
end
fprintf('\n');

figure(2);
plot(X(1,:),X(2,:),'b.');
hold on;
plot(X(1,:),X(3,:),'r.');
hold off;
