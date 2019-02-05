function [ cloud_info ] = cloudData( file )
%UNTITLED2 Summary of this function goes here
%  Extracts info from a loaded txt file. Returns nested list of information
%  to be passed into updateList function. 

name_string = file{1};
len = length(name_string);
number_string = name_string(len-6:len-4);
num = str2num(number_string);
cloud_info{1} = num;

numModels = (length(file) + 1)/6;
models = {};
for i = 1:numModels
    modelString = file{6*i-3};
    transString = file{6*i-2};
    rotString = file{6*i-1};
    for n = 1:length(modelString)
        if (modelString(n) == '/');
            start = n+1;
        end
        if (modelString(n) == '.');
            finish = n-1;
        end
    end
    modelName = modelString(start:finish);
    n = 0;
    breaks = {};
    for a = 1:length(transString)
        if (transString(a) == ' ')
            n = n+1;
            breaks{n} = a;
        end
    end
    trans = {};
    trans{1} = str2double(transString(breaks{1}+1:breaks{2}-1));
    trans{2} = str2double(transString(breaks{2}+1:breaks{3}-1));
    trans{3} = str2double(transString(breaks{3}+1:length(transString)));
    
    n = 0;
    breaks2 = {};
    for b = 1:length(rotString)
        if (rotString(b) == ' ')
            n = n+1;
            breaks2{n} = b;
        end
    end
    rot = {};
    rot{1} = str2double(rotString(breaks2{1}+1:breaks2{2}-1));
    rot{2} = str2double(rotString(breaks2{2}+1:breaks2{3}-1));
    rot{3} = str2double(rotString(breaks2{3}+1:breaks2{4}-1));
    rot{4} = str2double(rotString(breaks2{4}+1:length(rotString)));
    pose = {trans{1},trans{2},trans{3},rot{1},rot{2},rot{3},rot{4}};
    models{i} = {modelName,pose};
end

cloud_info{2} = models;

end

