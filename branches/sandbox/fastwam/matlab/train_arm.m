%function train_arm(train_swings, advice)
%train_arm(train_swings, advice) -- advice should be one of
%{'direct', 'gp'}.

ADVICE_NONE = 0;
ADVICE_DIRECT = 1;
ADVICE_CRKR = 2;
ADVICE_GPUCB = 3;
ADVICE_GPMI = 4;

cd ~/ros/mit-ros-pkg/branches/sandbox/pingpong/data/

% n = 1000;
% X = .2+.6*rand(n,1);
% Y = .3+.6*rand(n,1);
% S = sign(2*rand(n,1)-1);  % positive is underspin
% V = 3-.5*(S>0)+rand(n,1);
% VY = -1+2*rand(n,1);
% VX = -sqrt(V.^2 - VY.^2);
% VZ = -3.5+rand(n,1);
% ball_params = [X Y VX VY VZ S];
% save ball_params.mat ball_params

%load ball_params_underspin.mat
load ball_params_topspin.mat
ball_params = ball_params(51:end,:);

advice_type = ADVICE_NONE;

%for iter=1:100 %size(ball_params,1)
for iter=109:200 %size(ball_params,1)
    
    if iter==1
        system('rm swing_table*');

        load rss14/training_data_baseline_topspin.mat
        swing_table = training_data.swing_table{end};
        save_swing_table(swing_table);
        send_reload_swing_table_msg();

%         swing_table = struct();
%         swing_table.B = [];
%         swing_table.BT = [];
%         swing_table.U = {};
%         swing_table.KP = {};
%         swing_table.KD = {};
%         swing_table.E = [];
%         swing_table.Q = {};

        training_data = struct();
        training_data.hit_plan = [];
        training_data.reward = [];
        training_data.ball = {};
        training_data.pred = {};
        training_data.P_swing = {};
        training_data.T_swing = {};
        training_data.P_plan = {};
        training_data.T_plan = {};
        training_data.swing_table = {};
        training_data.hit_policies = {};
        training_data.advice = [];
        training_data.advice_type = advice_type;
    end

    % send hit policies to brain
    if advice_type > 0
        HP = [];
        if iter > 1
            HP = training_data.hit_policies{iter-1};
            send_hit_policies(HP(:, 1:8), HP(:, 9:17));
        end
    end
    
    iter
    ball_params(iter,:)
    [hp, swing, ball, pred] = get_training_ball(ball_params(iter,:));
%     ball_params(1,:)
%     [hp, swing, ball, pred] = get_training_ball(ball_params(1,:));
    fprintf('*************************************\n');

    %input('Hit enter to begin recording swing:');

    % get the swing data and ball trajectory data
    %[swing, ball, pred] = record_swing_and_ball_trajectories(10);
    Q_swing = swing.Q;
    T_swing = swing.T;

    % load swing plan
    name = '~/.ros/swing_plan';
    P_plan = read_matrix([name '_paddle.txt']);
    T_plan = read_matrix([name '_times.txt']);

    % draw the trajectories
    %draw_swing_and_ball_trajectories(swing, ball, pred);
    figure(4);
    draw_ball_trajectories(ball, pred, 100); %, 5);
    drawnow; pause(.1); drawnow; pause(.1); drawnow; pause(.1);

    % draw planned and recorded swings
    t0 = T_plan(1);
    P_swing = [];
    for i=1:size(Q_swing,1)
        P_swing(i,1:3) = arm_kinematics(Q_swing(i,:), 7, [0,0,.1], 1)';
        P_swing(i,4:6) = arm_kinematics(Q_swing(i,:), 7, [1,0,.1], 1)' - P_swing(i,1:3);
    end
    figure(1); clf, plot(T_plan-t0, P_plan(:,1:3)); hold on, plot(T_swing-t0, P_swing(:,1:3), '--');
    plot([.4 .4], [0,1], 'k-');  % desired hit time
    hold off, legend('x','y','z');
    for i=1:4, drawnow; pause(.1); end

    reward = input('Enter the reward: ');
    if isempty(reward), reward = 0; end
    
    % save training data
    training_data.hit_plan(iter,:) = hp(2:end);
    training_data.reward(iter) = reward;
    training_data.ball{iter} = ball;
    training_data.pred{iter} = pred;
    training_data.P_plan{iter} = P_plan;
    training_data.T_plan{iter} = T_plan;
    training_data.P_swing{iter} = P_swing;
    training_data.T_swing{iter} = T_swing;
    training_data.swing_table{iter} = swing_table;

    % get planned hit params
    [~,ihit_plan] = min(abs(T_plan-t0-.4));
    hit_pos_plan = P_plan(ihit_plan,1:3);
    hit_normal_plan = P_plan(ihit_plan,4:6);
    hit_vel_plan = (P_plan(ihit_plan+2,1:3) - P_plan(ihit_plan-2,1:3)) / (T_plan(ihit_plan+2) - T_plan(ihit_plan-2));

    % get actual hit params
    [~,ihit_swing] = min(abs(T_swing-t0-.4));
    hit_pos_swing = P_swing(ihit_swing,1:3);
    hit_normal_swing = P_swing(ihit_swing,4:6);
    hit_vel_swing = (P_swing(ihit_swing+2,1:3) - P_swing(ihit_swing-2,1:3)) / (T_swing(ihit_swing+2) - T_swing(ihit_swing-2));
    
    % check if swing error is below threshold
    actual_hit = [hit_pos_swing, hit_vel_swing, hit_normal_swing]
    planned_hit = [hit_pos_plan, hit_vel_plan, hit_normal_plan]

    % get advice
    if advice_type > 0
        policy_params = hp(10:end)
        advice = get_advice();
        training_data.advice(iter,:) = advice;
        
        if advice_type == ADVICE_DIRECT
            training_data.hit_policies{iter} = direct_advice(HP, advice);
        elseif advice_type == ADVICE_GPUCB
            training_data.hit_policies{iter} = gp_reward(HP, training_data.hit_plan, training_data.reward, training_data.advice);
        elseif advice_type == ADVICE_CRKR
            training_data.hit_policies{iter} = gp_params_crkr(HP, training_data.hit_plan, training_data.reward, training_data.advice);
        end
    end

    bad_pos = max(abs(actual_hit(1:3) - planned_hit(1:3)) >= [.05 .05 .05]);
    bad_vel = max(abs(actual_hit(4:6) - planned_hit(4:6)) >= [.3 .3 .3]);
    bad_normal = dot(actual_hit(7:9), planned_hit(7:9)) < .9;
    bad_swing = bad_pos || bad_vel || bad_normal;
    bad = [bad_pos, bad_vel, bad_normal]

    if bad_swing && input('Train swing? [y/n]: ','s')=='y'
        b = planned_hit;
        b(5) = 0;  %dbug: set vy=0
        t = .4;
        while 1
            try
                [u,kp,kd,err,q] = train_smooth_swing(b, t);
                break
            catch
                'Error in train_smooth_swing...trying again'
                pause(.1);
            end
        end

        if input('Add trained swing? [y/n]: ','s')=='y'
            swing_table.B(end+1,:) = b;
            swing_table.BT(end+1) = t;
            swing_table.U{end+1} = u;
            swing_table.KP{end+1} = kp;
            swing_table.KD{end+1} = kd;
            swing_table.E(end+1) = err;
            swing_table.Q{end+1} = q;

            % save swing table and tell brain to reload its swing table
            save_swing_table(swing_table);
            send_reload_swing_table_msg();
        end
    end
    
    save training_data.mat training_data swing_table
end








