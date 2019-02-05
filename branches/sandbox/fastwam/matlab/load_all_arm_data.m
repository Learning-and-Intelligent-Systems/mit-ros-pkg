function [T,Q,dQ,ddQ,U,L] = load_all_arm_data(fdir)
%[T,Q,dQ,ddQ,U,L] = load_all_arm_data(fdir)


d = dir([fdir '/*.m']);

T = [];
Q = [];
dQ = [];
ddQ = [];
U = [];
L = [];
for i=1:length(d)
    [Ti,Qi,dQi,ddQi,Ui] = load_arm_data(sprintf('%s/swingdata%d', fdir, i-1));
    T = [T, Ti];
    Q = [Q; Qi];
    dQ = [dQ; dQi];
    ddQ = [ddQ; ddQi];
    U = [U; Ui];
    L = [L, repmat(i-1, [1,length(Ti)])];
end
