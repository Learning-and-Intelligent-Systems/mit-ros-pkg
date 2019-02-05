function [T,Q,dQ,ddQ,U] = load_arm_data(fname)
%[T,Q,dQ,ddQ,U] = load_arm_data(fname)

fname
run(fname);

save tmp.mat times joint_angles joint_torques
data = load('tmp.mat');

T = data.times; %(1:10:end);
Q = data.joint_angles; %(1:10:end,:);
U = data.joint_torques; %(1:10:end,:);
[T,Q,dQ,ddQ,U] = differentiate_arm_data(T,Q,U);

d = size(Q,2);
for i=1:d
    dQ(:,i) = smooth(dQ(:,i));
    dQ(:,i) = smooth(dQ(:,i));
    ddQ(:,i) = smooth(ddQ(:,i));
    ddQ(:,i) = smooth(ddQ(:,i));
    ddQ(:,i) = smooth(ddQ(:,i));
    ddQ(:,i) = smooth(ddQ(:,i));
end

%Q = smooth(Q);

%n = length(data);
%X = reshape(data, [14, n/14])';
%Q = X(:,1:7);
%u = X(:,8:14);