function JA_traj = jacobian_swing(JA, paddle_traj, plotting)
%JA_traj = jacobian_swing(JA, paddle_traj) -- Compute a sequence of joint
%angles to follow the 6xN paddle trajectory matrix 'paddle_traj' with
%paddle positions in the top 3 rows and paddle normals in the bottom 3 rows.
%JA should be a set of joint angles that results in the first column
%of paddle_traj.

if nargin < 3
    plotting = 0;
end

n = size(paddle_traj,2);
JA_traj = zeros(7,n);
JA_traj(:,1) = JA;

for i=2:n
    J = paddle_jacobian(JA_traj(:,i-1))
    Jinv = pinv(J);
    dp = paddle_traj(:,i) - paddle_traj(:,i-1)
    JA_traj(:,i) = JA_traj(:,i-1) + Jinv*dp;
end

if plotting
    for i=1:n
        [a1,a2] = view(); draw_table, hold on, draw_arm(JA_traj(:,i)); view(a1,a2); drawnow;
    end
end
