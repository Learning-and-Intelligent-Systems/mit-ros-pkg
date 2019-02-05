function [X,Y] = add_swing_to_dynamics_model(X,Y,Q,U,T)
%[X,Y] = add_swing_to_dynamics_model(X,Y,Q,U,T)

d = length(X);  % num joints to model

[T,Q,dQ,ddQ,U] = differentiate_arm_data(T,Q,U);

for i=1:d
    dQ(:,i) = smooth(dQ(:,i));
    %dQ(:,i) = smooth(dQ(:,i));

    ddQ(:,i) = smooth(ddQ(:,i));
    ddQ(:,i) = smooth(ddQ(:,i));
    %ddQ(:,i) = smooth(ddQ(:,i));
    %ddQ(:,i) = smooth(ddQ(:,i));
end
    

for j=1:d
    X{j} = [X{j};  Q(:,1:d), dQ(:,1:d), U(:,j)];
    Y{j} = [Y{j};  ddQ(:,j)];
end
