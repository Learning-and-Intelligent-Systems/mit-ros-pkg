function A = new_hit_policy(A)

    input('hit enter to update hit policy:');

    %ball x y vel spin, paddle x y z vel n
    hp = read_matrix('~/.ros/last_swing.txt');
    hp = hp(:,2:end);
    index_in_A = 0;
    
    % look in hit_policies for any previous policies that are close enough
    for i=1:size(A,1)
        temp = A(i,1:8);
        if dot(temp - hp(1:8), temp - hp(1:8)) < 1.0
            hp = A(i,:);
            index_in_A = i;            
            break;
        end
    %     offset = [0.1, 0.1, 0.5, 0.1, 0.1];
    %     diff = true;
    %     
    %     for j=1:5
    %         if abs(temp(j) - new_policy(j)) > offset(i)
    %             diff = 
    %         end
    %     end

    end

    disp('Last swing parameters:');
    disp(hp);

    disp(['Give advice by specifying paddle p x y z, v x y z, or n x y z ' ...
        'along with numerical value, ie. "p x 0.5" indicates paddle position ' ...
        'should be greater in the x direction. Separate multiple pieces ' ...
        'of advice with commas.'])

    advice = input('Enter advice: ', 's');
    
    if isempty(advice)
        A = [A;hp];
        return;
    end
    
    %advice = strsplit(advice, ',');

    %for i=1:size(advice,2)
    %    temp = advice(i);
        property = advice(1:3);
        value = str2double(advice(5:end));
        index = 0;

        if strcmp(property, 'p x')
            index = 9;
        elseif strcmp(property, 'p y')
            index = 10;
        elseif strcmp(property, 'p z')
            index = 11;
        elseif strcmp(property, 'v x')
            index = 12;
        elseif strcmp(property, 'v y')
            index = 13;
        elseif strcmp(property, 'v z')
            index = 14;
        elseif strcmp(property, 'n x')
            index = 15;
        elseif strcmp(property, 'n y')
            index = 16;
        elseif strcmp(property, 'n z')
            index = 17;
        else
            disp('Invalid advice')
        end

        hp(:,index) = hp(:,index) + value;
    %end
    
    %update A
    if index_in_A > 0
        A(index_in_A,:) = hp;
    else
        A = [A; hp];
    end
end