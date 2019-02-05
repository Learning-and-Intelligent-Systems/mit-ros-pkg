function draw_ball_trajectories(ball, pred, framerate, skip)

TABLE_WIDTH = 1.525;

if nargin < 2
    pred = [];
end
if nargin < 3
    framerate = 10;
end
if nargin < 4
    skip = 1;
end

if ~isempty(pred)
    % align pred with ball by time
    pred2 = {};
    for i=1:length(ball.T)
        [~,j] = min(abs(pred.T - ball.T(i)));
        pred2{i} = pred.traj{j};
    end
    pred = pred2;
end

ball.pos = ball.obs; %dbug

% figure(1); [a1,a2] = view(); draw_table, axis equal, axis vis3d, view(a1,a2);
% hold on
% for i=1:length(ball.T)
%     plot3(ball.pos(1:i,2), TABLE_WIDTH - ball.pos(1:i,1), ball.pos(1:i,3), 'bo');
%     drawnow
%     pause(.1);
% end
% hold off

for i=1:skip:length(ball.T)
    %figure(1);
    [a1,a2] = view(); draw_table, axis equal, axis vis3d, view(a1,a2);
    hold on
    plot3(ball.pos(:,2), TABLE_WIDTH - ball.pos(:,1), ball.pos(:,3), 'b-');
    plot3(ball.pos(1:i,2), TABLE_WIDTH - ball.pos(1:i,1), ball.pos(1:i,3), 'bo');
    if ~isempty(pred)
        n = find((pred{i}(:,3) < -.2)+(pred{i}(:,2) < -.2)+(pred{i}(:,2) > 2.5), 1);
        if isempty(n), n = size(pred{i},1); end
        plot3(pred{i}(1:n,2), TABLE_WIDTH - pred{i}(1:n,1), pred{i}(1:n,3), 'ro');
    end
    hold off
    drawnow
    pause(1/framerate);
    %input(':');
end


