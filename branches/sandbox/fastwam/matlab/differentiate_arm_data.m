function [T,Q,dQ,ddQ,U] = differentiate_arm_data(T,Q,U)
%[T,Q,dQ,ddQ,U] = differentiate_arm_data(T,Q,U)

dT = repmat((T(2:end) - T(1:end-1)), [1,size(Q,2)]);
%dT2 = (dT(1:end-1,:) + dT(2:end,:)) / 2;
dQ = (Q(2:end,:) - Q(1:end-1,:)) ./ dT;
ddQ = (dQ(2:end,:) - dQ(1:end-1,:)) ./ dT(2:end,:); %dT2;

T = T(2:end-1);
Q = Q(2:end-1,:);
dQ = dQ(1:end-1,:);
U = U(2:end-1,:);


