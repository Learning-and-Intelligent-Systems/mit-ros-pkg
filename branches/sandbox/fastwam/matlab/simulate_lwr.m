function [Q,dQ,A,B] = simulate_lwr(q0,dq0,dt,U,X,Y)

joint_noise = [.1,.05,.05,.05,.01,.01,.002];
r = zeros(7,15);
for j=1:7
    r(j,:) = [.1,.1,.1,.1,.1,.1,.1, .2,.2,.2,.2,.2,.2,.2, 10*joint_noise(j)];
end
lambda = 100;


n = size(U,1);

Q = q0;
dQ = dq0;

if nargout>=3
    A = zeros(7,15,n);
end
if nargout>=3
    B = zeros(7,7,n);
end

for i=1:n
    q = Q(end,:);
    dq = dQ(end,:);
    ddq = zeros(1,7);
    for j=1:7
        x = [q,dq,U(i,j)];
        [ddq(j), b] = lwrsample(X{j},Y{j},x,r(j,:),lambda);
        
        if nargout>=3
            A(j,:,i) = b(1:end-1);
        end
        if nargout>=4
            B(j,j,i) = b(end);
        end
    end
    Q(end+1,:) = q + dt*dq;
    dQ(end+1,:) = dq + dt*ddq;
end
