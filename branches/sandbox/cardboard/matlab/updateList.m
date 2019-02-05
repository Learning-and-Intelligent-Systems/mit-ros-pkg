function [] = updateList( list, cloud_info )
%UNTITLED Summary of this function goes here
%   list is loaded list from file, cloud_info is extracted from each text
%   file. contains: {cloud_number,{{object,pose},{object,pose},etc...}}

cloud_number = cloud_info{1};
list{cloud_number} = cloud_info{2};
end

