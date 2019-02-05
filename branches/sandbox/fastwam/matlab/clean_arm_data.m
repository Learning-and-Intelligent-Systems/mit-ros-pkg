function [Q,dQ,ddQ,u] = clean_arm_data(Q,dQ,ddQ,u)
%[Q,dQ,ddQ,u] = clean_arm_data(Q,dQ,ddQ,u)

%du = u(2:end,:) - u(1:end-1,:);
mask = min(abs(u(:,2:4))<5, [], 2);
mask2 = min(abs(ddQ(:,2:4))<.001, [], 2);
mask = logical(mask.*mask2);
Q = Q(mask,:);
dQ = dQ(mask,:);
ddQ = ddQ(mask,:);
u = u(mask,:);
