function save_arm_data(T,Q,dQ,ddQ,U)
%save_arm_data(T,Q,dQ,ddQ,U)

fprintf('Saving T.txt...\n');
write_matrix(T', 'T.txt');

fprintf('Saving Q.txt...\n');
write_matrix(Q, 'Q.txt');

fprintf('Saving dQ.txt...\n');
write_matrix(dQ, 'dQ.txt');

fprintf('Saving ddQ.txt...\n');
write_matrix(ddQ, 'ddQ.txt');

fprintf('Saving U.txt...\n');
write_matrix(U, 'U.txt');
