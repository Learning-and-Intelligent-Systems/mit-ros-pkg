function U = get_gravity_torques_for_trajectory(Q, gravity_table)
%U = get_gravity_torques_for_trajectory(Q)
%U = get_gravity_torques_for_trajectory(Q, gravity_table)


if nargin < 2
    dt = 1; %2;
    paddle_normal = [1,0,0]; %dbug
    [~,U,~] = joint_swing(Q, dt, paddle_normal);

    %U = U(900:1000:end,:);
    U = U(450:500:end,:);
    
else
    n = size(Q,1);
    ng = size(gravity_table.Q_table,1);
    U = zeros(n,7);
    for i=1:n
        q = Q(i,:);
        DQ2 = (repmat(q, [ng,1]) - gravity_table.Q_table).^2;
        [~,idx] = min(DQ2*[1 1 1 1 .1 .1 .1]');
        U(i,:) = gravity_table.U_table(idx,:);
    end
    
    % smooth gravity torques
    for j=1:7
        U(:,j) = smooth(U(:,j));
    end
end
