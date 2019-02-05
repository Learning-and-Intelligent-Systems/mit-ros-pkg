function draw_swing_and_ball_trajectories(swing, ball, pred)
%draw_swing_and_ball_trajectories(swing, ball, pred)

TABLE_WIDTH = 1.525;

Q = swing.Q;
U = swing.U;
T = swing.T;
traj = ball.pos;
traj_times = ball.T;
pred_times = pred.T;
pred = pred.traj;

% align Q, traj, and pred by time
swing_times = T;
t0 = min([min(T),min(traj_times),min(pred_times)]);
t1 = max([max(T),max(traj_times),max(pred_times)]);
T = t0:.01:t1;
Q2 = {};
traj2 = {};
pred2 = {};
for i=1:length(T)
    [d,j] = min(abs(swing_times - T(i)));
    if d < .01, Q2{i} = Q(j,:); else Q2{i} = []; end
    [d,j] = min(abs(traj_times - T(i)));
    if d < .01, traj2{i} = traj(1:j,:); else traj2{i} = []; end
    [d,j] = min(abs(pred_times - T(i)));
    if d < .01, pred2{i} = pred{j}; else pred2{i} = []; end
end
Q = Q2;
traj = traj2;
pred = pred2;

% q = [];
% for i=1:length(T)
%     figure(1); [a1,a2] = view(); draw_table
%     if ~isempty(traj{i}), hold on, plot3(traj{i}(:,2), TABLE_WIDTH - traj{i}(:,1), traj{i}(:,3), 'b-'); hold off, end
%     if ~isempty(Q{i}), q = Q{i}; end, if ~isempty(q), hold on, draw_arm(q); hold off, end
%     axis equal, axis vis3d, view(a1,a2);
%     title(sprintf('t = %.3f', T(i)-T(1)));
%     zoom(2); %input(':');
% end

q = [];
for i=1:length(T)
    if ~isempty(pred{i}) && pred{i}(1,2) > 1.5 && pred{i}(2,2) > pred{i}(1,2)
        break
    end
    figure(1); [a1,a2] = view(); draw_table
    if ~isempty(traj{i}), hold on, plot3(traj{i}(:,2), TABLE_WIDTH - traj{i}(:,1), traj{i}(:,3), 'bo', 'LineWidth', 2); hold off, end
    if ~isempty(Q{i}), q = Q{i}; end, if ~isempty(q), hold on, draw_arm(q); hold off, end
    axis equal, axis vis3d, view(a1,a2);
    if ~isempty(pred{i}), hold on, plot3(pred{i}(:,2), TABLE_WIDTH - pred{i}(:,1), pred{i}(:,3), 'r-', 'LineWidth', 2); hold off, end
    title(sprintf('t = %.3f', T(i)-T(1)));
    %zoom(2); %input(':');
end


