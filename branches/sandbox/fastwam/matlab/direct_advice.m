function hit_policies = direct_advice(hit_policies, advice)
%hit_policies = direct_advice(hit_policies, advice)

%hit_policies = [];

%while(true)
%    input(':');
    
    %ball x y vel, paddle x y z vel n
    hp = read_matrix('~/.ros/last_swing.txt');
    hp = hp(:,2:end);
    
    index_in_A = 0;
    bkts = 5;
    % tolerances for ball params
    bb_thresh = [0.6/bkts, 0.6/bkts, 1.5/bkts, 2.0/bkts, 1.0/bkts, 1000000, 200, 1000000];

    % look in hit_policies for any previous policies that are close enough
    for i=1:size(hit_policies,1)
        if abs(hit_policies(i,1:8) - hp(1:8)) < bb_thresh
            index_in_A = i;
            break;
        end
    end

    disp('Last swing parameters:');
    disp(hp);

    hp(9:end) = hp(9:end) + advice;

    %update A
    if index_in_A > 0
        hit_policies(index_in_A,:) = hp;
    else
        hit_policies = [hit_policies; hp];
    end
    
%     send_hit_policies(hit_policies(:, 1:8), hit_policies(:, 9:17));
%end