%function [B,B_cov,B2,B2_cov,BF] = filter_ball_trajectory(X,T,x0,track_spin)
function [B,B_cov] = filter_ball_trajectory(X,T,x0,track_spin)
%[B,B_cov] = filter_ball_trajectory(X,T,x0,track_spin)

if nargin < 4
    track_spin = 0;
end

% all units are SI
r = .02;
g = 9.8;
ball_coeff_rest = .88;  % coefficient of restitution
p = 1.204;              % air density at 20 degrees celcius
C = .1;                 % drag coeff. for a smooth sphere
m = .0027;              % ball mass
A = pi*r*r;
Cd = .5*C*p*A/m;
Cm = .0004;


ball_filter_measurement_noise = [.006, .006, .02];
if track_spin
    ball_filter_process_noise_ballistic = [0, 0, 0, 1, 1, 1, 1, 1, 1];
else
    ball_filter_process_noise_ballistic = [0, 0, 0, 1, 1, 1];
    %ball_filter_process_noise_bounce = [0, 0, 0, 50, 50, 50];
end

n = size(X,1);

% save prediction states (i+1 | i) for smoothing
%B2 = zeros(n-1,6);
%B2_cov = zeros(6,6,n-1);
%BF = zeros(6,6,n-1);

if track_spin
    B = zeros(n,9);
    B_cov = zeros(9,9,n);
    B_cov(:,:,1) = diag([.006 .006 .02 100 100 100 1e8 1e8 1e8]);
    %ball_filter_measurement_noise = .1*[.006, .006, .02];
else
    B = zeros(n,6);
    B_cov = zeros(6,6,n);
    B_cov(:,:,1) = diag([.006 .006 .02 100 100 100]);
end

if nargin >= 3 && ~isempty(x0)
    B(1,:) = x0;
else
    B(1,1:3) = X(1,:);
end

for i=2:n
    dt = T(i) - T(i-1);
    
    % prediction update
    x_pred = ball_dynamics(B(i-1,:)', dt);
    
    vx = B(i-1,4);
    vy = B(i-1,5);
    vz = B(i-1,6);
    v2 = vx*vx + vy*vy + vz*vz;
    v = sqrt(v2);
    if v > 1e-8
        v_v = [vx vy vz]' / v;
    else
        v_v = [0 0 0]';
    end
    
    dvvdv = v*eye(3) + v_v*[vx vy vz];
    
    if track_spin
        wx = B(i-1,7);
        wy = B(i-1,8);
        wz = B(i-1,9);
        W = [0 -wz wy; wz 0 -wx; -wy wx 0];
        V = [0 vz -vy; -vz 0 vx; vy -vx 0];
        
        dadv = (Cm*W - Cd)*dvvdv;
        dadw = Cm*v*V;
        
        F = [ eye(3,3)       dt*eye(3)       zeros(3,3) ;
             zeros(3,3)   eye(3) + dt*dadv    dt*dadw   ;
             zeros(3,3)      zeros(3,3)       eye(3,3)  ];
         
        H = [eye(3) zeros(3) zeros(3)];
    else
        F = [eye(3,3),  dt*eye(3);  zeros(3,3),  eye(3) - dt*Cd*dvvdv];
        H = [eye(3) zeros(3)];
    end
     
    Q = dt*diag(ball_filter_process_noise_ballistic);
    P = F*B_cov(:,:,i-1)*F' + Q;
    
    %B2(i-1,:) = x_pred';
    %B2_cov(:,:,i-1) = P;
    %BF(:,:,i-1) = F';
    
    % measurement update
    y = X(i,1:3)' - x_pred(1:3);
    R = diag(ball_filter_measurement_noise);
    S = H*P*H' + R;
    K = P*H'*inv(S);
    
    x = K*y + x_pred;
    P = P - K*H*P;
    
    B(i,:) = x';
    B_cov(:,:,i) = P;
end
