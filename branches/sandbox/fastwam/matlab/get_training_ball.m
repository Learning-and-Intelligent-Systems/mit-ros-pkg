function [hp, swing, ball, pred] = get_training_ball(ball_params, tolerances)
%[hp, swing, ball, pred] = get_training_ball(ball_params, tolerances) -- Call this function before you hit
%the ball to the robot--it won't return until you hit a ball within the
%tolerance of the desired ball params (x,y,vx,vy,vz,spin) where spin > 0 is
%underspin and spin < 0 is topspin.

TABLE_WIDTH = 1.525;

s = '';
if ball_params(5) < -3,  s = [s ' HIGH'];  else  s = [s ' LOW'];  end
if ball_params(4) < 0,  s = [s ' FOREHAND'];  else  s = [s ' BACKHAND'];  end
if ball_params(6) < 0,  s = [s ' TOPSPIN'];  else  s = [s ' UNDERSPIN'];  end
disp(s);

if nargin < 2
    tolerances = [.3, .3, 2, 2, 1];
end

% get the time of the last ball (so we know when a new ball has arrived)
try
    hp = read_matrix('~/.ros/last_swing.txt');
    t = hp(1);
catch
    t = 0;
end

figure(1);
draw_table(); hold on
[XI,YI] = meshgrid(.2:.2:.8, .3:.2:.9);
n = numel(XI);
plot3(XI(:), YI(:), zeros(n,1), 'o', 'MarkerSize', 5);
if ball_params(6) > 0
    plot3(ball_params(1), ball_params(2), 0, 'gx', 'MarkerSize', 10, 'LineWidth', 3);
else
    plot3(ball_params(1), ball_params(2), 0, 'rx', 'MarkerSize', 10, 'LineWidth', 3);
end
hold off, axis equal, axis vis3d, view(90,90);
drawnow
pause(.1)

while 1
    %try
        [swing ball pred] = record_swing_and_ball_trajectories(100);
    %catch
    %    pause(.1);
    %    continue
    %end
    pause(.1);
    
    
    hp = read_matrix('~/.ros/last_swing.txt');
    if hp(1) > t
        t = hp(1);
        ball_params
        b = hp(2:9)
        
        % read the ball bounce pos and vel from ball
        i_bounce = find((ball.vel(2:end,3)>0).*(ball.vel(1:end-1,3)<0), 1);
        b(1:5) = [ball.pos(i_bounce,2), TABLE_WIDTH - ball.pos(i_bounce,1), ball.vel(i_bounce,2), -ball.vel(i_bounce,1), ball.vel(i_bounce,3)];

        
        if abs(b(1:5)-ball_params(1:5)) < tolerances(1:5)
            if sign(b(7)) == sign(ball_params(6))
                return
            end
        end
        s = '';
        names = {'X', 'Y', 'VX', 'VY', 'VZ'};
        for i=1:5
            if abs(b(i)-ball_params(i)) >= tolerances(i)
               if b(i) < ball_params(i)
                   s = [s ' HIGHER ' names{i} ',  '];
               elseif b(i) > ball_params(i)
                   s = [s ' lower ' names{i} ',  '];
               end
            end
        end
        if sign(b(7)) ~= sign(ball_params(6))
            if ball_params(6) < 0
                s = [s ' TOPSPIN'];
            elseif ball_params(6) > 0
                s = [s ' underspin'];
            end
        end
        disp(s);
    end
end
