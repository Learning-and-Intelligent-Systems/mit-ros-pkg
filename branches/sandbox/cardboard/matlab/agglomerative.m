
function [ segmentNodes ] = agglomerative(file,thresh)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here



pcd = load_pcd(file);
numPoints = length(pcd.data);
clusterList = zeros(1:numPoints);
pointsList = randperm(numPoints);


numClusters = 1;
clusterList(1)= 1;

for i = 2:numPoints
    closeList = []
    point = pcd.data(pointsList(i),:);
    for j = 1:i-1
        comPoint = pcd.data(pointsList(j),:);
        if eucDist(point,comPoint) < thresh
            closeList = [closeList clusterList(pointsList(j))];
        end
    end
    closeList = unique(closeList);
    clusterList(i) = min(closeList);
    for k = 2:length(closeList)
        clusterList(clusterlist == closeList(k)) = clusterList(i);
    end
end
            
        
        
            





































while (1 == 1)
    for i = 1:length(workList)
        p = workList{i};
        if ~isIn(justClustered,p) 
            q = nearestIndex(p); %DEFINE THAT <<<<
            r = nearestIndex(q);
            if p == r
                mergeCluster(p,q);
                add(justClustered,p);
                add(justClustered,q);
                add(newWork,p);
            else
                add(newWork,p);
            end    
        end
    end
    justClustered = {};
    if size(newWork) == 1
        break
        segmentNodes = workList;
    end
    workList = newWork;
    newWork = {};
end





























%%

function [bool] = isIn(list,ele)
    bool = 0;
    for i = 1:length(list)
        if list{i} == ele
            bool = 1;
            break
        end
    end
        

%%


function [dist] = eucDist(point1,point2)
dist = ((point1{1} + point2{1})^2 + (point1{2} + point2{2})^2 + (point1{3} + point2{3})^2)^.5;
end

%%
function [] = add( oldList, item )
oldList = [oldList item];
end

%%

function [pointIndex] = closestPointIndex(index, pcd)
d = 0;
ind = 0;
point = pcd.data(index,:);
for  i = 1:length(pcd.data)      
    iPoint = pcd.data(i,:);
    dist = eucDist(point,iPoint);
    if (i ~= index)
        if (i == 1)
            d = dist;
            ind = 1;
        elseif (dist < d)
            d = dist;
            ind = i;
        end
    end    
end
pointIndex = ind;
end
%%

function [nodeNumber] = inWhichNode(index, nodes)
% if nodeNumber == 0, then the index is in no node. 
nodeNumber = 0;
for l = 1:length(nodes)
    for n = 1:length(nodes{l})
        if (nodes{l}{n} == index)
            nodeNumber = l;
        end
    end
end
end
%%

function [newNodes] = deleteElement(nodes, element)
start = nodes(1,1:element-1);
finish = nodes(1,element+1:length(nodes));
newNodes = [start finish];
end
%%

function [] = mergeNodes(nodes, node1Number, node2Number)
nodes{node1Number} = [nodes{node1Number} nodes{node2Number}];
nodes = deleteElement(nodes, node2Number);
end
%%




classdef Cluster < handle
    properties
        elements
    end
    methods
        function [] = Cluster(x)
            elements = [elements x];
        end 
    end
end

function [] = mergeCluster(a,b)
    a.elements = [a.elements b.elements];
end

function [dist] = smallestDist(a,b)
    n = eucDist(pcd.data(a.elements{1},:),pcd.data(b.elements{1},:));
    for i = 1:length(a.elements)
        for j = 1:length(b.elements)
            iPoint = pcd.data(i,:);
            jPoint = pcd.data(j,:);
            d = eucDist(iPoint,jPoint);
        end
    end
end

            
            
            
 function [d] = dist(cluster1, cluster2)
     for i = 1:length(cluster1)
         for j = 1:length(cluster2)
             
            


