function [traj_q, traj_dq] = simulate_trajectory_ll(q0,dq0,T,u,findices,fparams,Q,dQ,ddQ,U,r, traj_q_true)
%[traj_q, traj_dq] = simulate_trajectory_ll(q0,dq0,T,u,Q,dQ,ddQ,U,r, traj_q_true)




t = T(1);

n = length(T);
d = length(q0);

traj_q = zeros(n,d);
traj_dq = zeros(n,d);
traj_q = q0;
traj_dq = dq0;
q = q0;
dq = dq0;

for i=2:n
    dt = T(i) - t;
    t = T(i);
    
    ddq = zeros(1,d);
    for j=1:d
        %ddq(j) = llsample([Q,dQ,U(:,j)], ddQ(:,j), [q,dq,u(i-1,j)], r);
        ddq(j) = llsample_flann([Q,dQ,U(:,j)], findices{j}, fparams{j}, ddQ(:,j), [q,dq,u(i-1,j)], r);
    end
    
    q = q + dt*dq;
    dq = dq + dt*ddq;
    
    traj_q(i,:) = q;
    traj_dq(i,:) = dq;
    
    if nargin >= 9 && mod(i,100)==0
        for j=1:d
            subplot(d,1,j);
            plot(T(1:i)-T(1), traj_q_true(1:i,j), 'b-');
            hold on;
            plot(T(1:i)-T(1), traj_q(1:i,j), 'r-');
            hold off;
            drawnow
        end
    end
end
