function a = get_advice()
%a = get_advice()

disp(['Give advice by specifying paddle p x y z, v x y z, or n x y z ' ...
    'along with numerical value, ie. "p x 0.5" indicates paddle position ' ...
    'should be greater in the x direction.'])

a = zeros(1,9);

while 1
    advice = input('Enter advice: ', 's');
    %advice = strsplit(advice, ',');

    if ~isempty(advice)
        if length(advice) < 5
            disp('Invalid advice')
            continue
        end

        property = advice(1:3);
        value = str2double(advice(5:end));

        if strcmp(property, 'p x')
            index = 1;
        elseif strcmp(property, 'p y')
            index = 2;
        elseif strcmp(property, 'p z')
            index = 3;
        elseif strcmp(property, 'v x')
            index = 4;
        elseif strcmp(property, 'v y')
            index = 5;
        elseif strcmp(property, 'v z')
            index = 6;
        elseif strcmp(property, 'n x')
            index = 7;
        elseif strcmp(property, 'n y')
            index = 8;
        elseif strcmp(property, 'n z')
            index = 9;
        else
            disp('Invalid advice')
            continue
        end

        a(index) = value;
    end
    break
end
