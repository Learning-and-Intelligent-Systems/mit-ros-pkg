function [C, dCdU] = compute_swing_cost(q0,dq0,dt,U,X,Y, v0_des, v_des, n_des)
%[C, dCdU] = compute_swing_cost(q0,dq0,dt,U,X,Y)

if size(v0_des,1) < 3
    v0_des = v0_des';
end
if size(v_des,1) < 3
    v_des = v_des';
end
if size(n_des,1) < 3
    n_des = n_des';
end

[Q,dQ,A,B] = simulate_lwr(q0,dq0,dt,U,X,Y);

figure(1); [a1,a2] = view(); draw_table(); for i=1:5:size(Q,1), hold on, draw_arm(Q(i,:)); end, hold off, view(a1,a2); drawnow  %dbug

[x,dfdx,dfdu] = state_gradients(Q,dQ,A,B);
[g,dgdx,dgdu] = cost_gradients(Q,dQ,U,dt,v0_des,v_des,n_des);

UD = 7;
XD = 15;
N = size(U,1);

%dudalpha = zeros(UD,UD*N); dudalpha(:,UD*(N-1)+1:UD*N) = eye(UD); %gradient of u w.r.t. parameters for open loop policy at N
%F_alpha = dfdu(:,:,N)*dudalpha;
%G_alpha = dgdu(N,:)*dudalpha;
%y = zeros(XD,1);
%dJdalpha = (G_alpha'-F_alpha'*y).*dt; % dJdalpha for first time step

y = zeros(XD,1);
dJdalpha = zeros(UD*N,1);
for n=N:-1:1 %integrate adjoint equations backwards in time
    F_x = dfdx(:,:,n);
    G_x = dgdx(n,:);
    dudalpha = zeros(UD,UD*N); dudalpha(:,UD*(n-1)+1:UD*n) = eye(UD); %gradient of u w.r.t. parameters for open loop policy at N
    F_alpha = dfdu(:,:,n)*dudalpha;
    G_alpha = dgdu(n,:)*dudalpha;
    %y = y + (F_x'*y - G_x')*dt; %SOLVE for y
    y = (eye(XD) - F_x'.*dt) \ (y - G_x'.*dt);
    dJdalpha = dJdalpha + (G_alpha'-F_alpha'*y).*dt; %ADD this step's contribution to dJdalpha
end

C = sum(g);
dCdU = dJdalpha';

end


function [x,dfdx,dfdu] = state_gradients(Q,dQ,A,B)

    n = size(Q,1) - 1;
    nq = size(Q,2);
    
    x = [ones(n,1), Q(1:n,:), dQ(1:n,:)];
    nx = size(x,2);
    
    dfdx = zeros(nx,nx,n);
    dfdu = zeros(nx,nq,n);
    for i=1:n
        dfdx(:,:,i) = [zeros(1,nx); zeros(nq,1+nq), eye(nq); A(:,:,i)];
        dfdu(:,:,i) = [zeros(1+nq,nq); B(:,:,i)];
    end
end


function [g,dgdx,dgdu] = cost_gradients(Q,dQ,U,dt, v0_des, v_des, n_des)

    line_weight = 1;
    speed_weight = 0.1;
    normal_weight = 0.01;
    u_weight = .1;

    s_des = norm(v_des);
    v_des = v_des/s_des;

    n = size(Q,1) - 1;
    m = size(Q,2);
    
    g = zeros(n,1);
    dgdx = zeros(n,2*m+1);
    dgdu = zeros(n,m);

    %dbug
    g_line = zeros(n,1);
    g_speed = zeros(n,1);
    g_normal = zeros(n,1);
    %g_u = zeros(n,1);

    for i=1:n
        q = Q(i,:);
        q2 = Q(i,:) + dt*dQ(i,:);
        P = arm_kinematics(q, 7, [0,0,.1], 1);
        N = arm_kinematics(q, 7, [1,0,.1], 1) - P;
        P2 = arm_kinematics(q2, 7, [0,0,.1], 1);
        dPNdq = paddle_jacobian(q);
        dPNdq2 = paddle_jacobian(q2);
        dPdq = dPNdq(1:3,:);
        dNdq = dPNdq(4:6,:);
        dPdq2 = dPNdq2(1:3,:);
        
        % compute squared error terms
        line_err = sum((P - v0_des).^2) - dot(v_des, P - v0_des)^2;
        speed_err = (norm(P2-P)/dt - s_des)^2;
        normal_err = sum((N - n_des).^2);
        %u_err = U(i,:)*U(i,:)';
        
        % compute error gradients (w.r.t. q, dq)
        line_err_gradient = [0, 2*(P - v0_des)'*(eye(3) - v_des*v_des')*dPdq, zeros(1,7)];
        if norm(P2-P) < eps
            speed_err_gradient = zeros(1,2*m+1);
        else
            speed_err_gradient = [0, 2*(1 - dt*s_des/norm(P2-P))*(P2-P)'*[dPdq2 - dPdq, dt*dPdq2]] / dt^2;
        end
        normal_err_gradient = [0, 2*(N - n_des)'*dNdq, zeros(1,7)];
        %u_err_gradient = 2*U(i,:);
        
        % cost at time i
        g(i) = line_weight*line_err + speed_weight*speed_err + normal_weight*normal_err; % + u_weight*u_err;
        
        %dbug
        g_line(i) = line_weight*line_err;
        g_speed(i) = speed_weight*speed_err;
        g_normal(i) = normal_weight*normal_err;
        %g_u(i) = u_weight*u_err;
        
        % cost gradient at time i
        dgdx(i,:) = line_weight*line_err_gradient + speed_weight*speed_err_gradient + normal_weight*normal_err_gradient;
        %dgdu(i,:) = u_weight*u_err_gradient;
    end

    %dbug
    g_line([1:20,40:end]) = 0;
    g_speed([1:20,40:end]) = 0;
    g_normal([1:20,40:end]) = 0;
    %g_u([1:20,40:end]) = 0;

    figure(2); plot([g_line, g_speed, g_normal]); drawnow  %dbug
    %figure(2); plot([g_line, g_speed, g_normal, g_u]); drawnow  %dbug
    
    %dbug
    g([1:20,40:end]) = 0;
    dgdx([1:20,40:end],:) = 0;
    %dgdu([1:20,40:end],:) = 0;
end



