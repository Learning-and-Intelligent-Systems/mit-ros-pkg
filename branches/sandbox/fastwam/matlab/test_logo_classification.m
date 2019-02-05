% %P = zeros(size(BI,3),4);
% for i=1818:size(BI,3)
%     figure(1);
%     imshow(BI(:,:,i));
%     p = ginput(2)';
%     if size(p)==[2,2]
%         P(i,:) = p(:)';
%     end
% end
% save logo.mat P BI

%load logo.mat

n = size(BI,3);
B = zeros(n,32*32); for i=1:n, b = BI(:,:,i)'; B(i,:) = b(:)'; end
B_avg = mean(B);
%[V,~] = eig(cov(B));
%X = (B-repmat(B_avg,[n,1]))*V;
[V,X] = princomp(B);

P2 = zeros(n,2);
P2(pos,:) = P(pos,3:4) - P(pos,1:2);
P2(pos,:) = P2(pos,:)./repmat(sqrt(sum(P2(pos,:).^2,2)), [1 2]);

d = 8;
L = logical(sum(P.^2,2));

y = 2*L-1;
X2 = [ones(n,1), X(:,1:d)];
X3 = [ones(n,1), [X(2,1:d); X(1:end-1,1:d)], X(1:end,1:d)];
pos = find(y>0);
n_pos = sum(y>0);
neg = find(y<0);
idx = [pos; neg(randperm(length(neg),n_pos))];
idx = idx(randperm(length(idx)));

% % perceptron
% w = perceptron(X3(idx,:), y(idx));
% R_perceptron = mean(sign(X3*w')==y)
% R_pos_perceptron = sum((X3*w'>0).*(y>0)) / sum(y>0)
% R_neg_perceptron = sum((X3*w'<0).*(y<0)) / sum(y<0)
% 
% % SVM (linear)
% S = svmtrain(X2, L, 'kktviolationlevel', .1, 'boxconstraint', .5);
% SL = svmclassify(S,X2);
% R_svm_linear = mean(SL==L)
% R_pos_svm_linear = sum((SL).*(L)) / sum(L)
% R_neg_svm_linear = sum((SL==0).*(L==0)) / sum(L==0)
 
% SVM (rbf)
idx_train = randperm(n,ceil(n/2));
idx_test = setdiff(1:n, idx_train);
L_train = L(idx_train);
L_test = L(idx_test);
%S = svmtrain(X2(idx_train,:), L_train, 'boxconstraint', 5, 'rbf_sigma', 1, 'kernel_function', 'rbf');
%SL = svmclassify(S,X2);
S = svmtrain(X(idx_train,1:d), L_train, 'boxconstraint', 5, 'rbf_sigma', 1, 'kernel_function', 'rbf');
SL = svmclassify(S,X(:,1:d));
% SL_test = svmclassify(S,X3(idx_test,:));
% R_svm_rbf = mean(SL_test==L_test)
% R_pos_svm_rbf = sum((SL_test).*(L_test)) / sum(L_test)
% R_neg_svm_rbf = sum((SL_test==0).*(L_test==0)) / sum(L_test==0)
R_svm_rbf = mean(SL==L)
R_pos_svm_rbf = sum((SL).*(L)) / sum(L)
R_neg_svm_rbf = sum((SL==0).*(L==0)) / sum(L==0)

S = svmtrain(X(:,1:d), L, 'boxconstraint', 5, 'rbf_sigma', 1, 'kernel_function', 'rbf');
SL = svmclassify(S,X(:,1:d));
R_svm_rbf_full = mean(SL==L)
R_pos_svm_rbf_full = sum((SL).*(L)) / sum(L)
R_neg_svm_rbf_full = sum((SL==0).*(L==0)) / sum(L==0)


% regression (position)
d2 = 20;
k = .1;
X4 = [ones(n,1), X(:,1:d2)];
wx4 = (X4(pos,:)'*X4(pos,:) + k*eye(d2+1)) \ (X4(pos,:)'*P(pos,1)); %regress(P(pos,1), X4(pos,:));
wy4 = (X4(pos,:)'*X4(pos,:) + k*eye(d2+1)) \ (X4(pos,:)'*P(pos,2)); %regress(P(pos,2), X4(pos,:));
Ex4 = mean(abs(P(pos,1) - X4(pos,:)*wx4));
Ey4 = mean(abs(P(pos,2) - X4(pos,:)*wy4));
E4 = [Ex4 Ey4]

E5 = zeros(n,2);
for i=1:n
    if ismember(i,neg), continue, end
    b = B_avg + X(i,1:d2)*V(:,1:d2)';
    p4 = [X4(i,:)*wx4, X4(i,:)*wy4];
    p5 = image_find_local_mode(reshape((1-b).*(B_avg>0),[32,32])', p4, 2, 4);
    E5(i,:) = abs(P(i,1:2) - p5);
end
E5 = sum(E5) / length(pos)

%X5 = [ones(n,1), [X(2,1:d2); X(1:end-1,1:d2)], X(1:end,1:d2)];
%wx5 = (X5(pos,:)'*X5(pos,:) + k*eye(2*d2+1)) \ (X5(pos,:)'*P(pos,1)); %regress(P(pos,1), X5(pos,:));
%wy5 = (X5(pos,:)'*X5(pos,:) + k*eye(2*d2+1)) \ (X5(pos,:)'*P(pos,2)); %regress(P(pos,2), X5(pos,:));
%Ex5 = mean(abs(P(pos,1) - X5(pos,:)*wx5));
%Ey5 = mean(abs(P(pos,2) - X5(pos,:)*wy5));
%E5 = [Ex5 Ey5]


% regression (angle)
for iter=1:10
    ux4 = (X4(pos,:)'*X4(pos,:) + k*eye(d2+1)) \ (X4(pos,:)'*P2(pos,1)); %regress(P2(pos,1), X4(pos,:));
    uy4 = (X4(pos,:)'*X4(pos,:) + k*eye(d2+1)) \ (X4(pos,:)'*P2(pos,2)); %regress(P2(pos,2), X4(pos,:));
    P2_r4 = X4(pos,:)*[ux4 uy4]; P2_r4 = P2_r4 ./ repmat(sqrt(sum(P2_r4.^2,2)), [1 2]);
    Eu4 = (180/pi)*mean(acos(abs(sum(P2(pos,:).*P2_r4, 2))));
    
    for i=1:length(pos)
        j = pos(i);
        if dot(P2(j,:), P2_r4(i,:)) < 0
            P2(j,:) = -P2(j,:);
        end
    end
end
Eu4

% for iter=1:10
%     ux5 = (X5(pos,:)'*X5(pos,:) + k*eye(2*d2+1)) \ (X5(pos,:)'*P2(pos,1)); %regress(P2(pos,1), X5(pos,:));
%     uy5 = (X5(pos,:)'*X5(pos,:) + k*eye(2*d2+1)) \ (X5(pos,:)'*P2(pos,2)); %regress(P2(pos,2), X5(pos,:));
%     P2_r5 = X5(pos,:)*[ux5 uy5]; P2_r5 = P2_r5 ./ repmat(sqrt(sum(P2_r5.^2,2)), [1 2]);
%     Eu5 = (180/pi)*mean(acos(abs(sum(P2(pos,:).*P2_r5, 2))));
% 
%     for i=1:length(pos)
%         j = pos(i);
%         if dot(P2(j,:), P2_r5(i,:)) < 0
%             P2(j,:) = -P2(j,:);
%         end
%     end
% end
% Eu5


% for i=1:n
%     if ismember(i,neg), continue, end
%     figure(1);
%     imshow(BI(:,:,i));
%     hold on
%     if L(i)
%         plot_circle(16.5,16.5,11,'g-','LineWidth',5);
%         plot(P(i,1), P(i,2), 'ro');
%         plot([P(i,1) P(i,1)+3*P2(i,1)], [P(i,2) P(i,2)+3*P2(i,2)], 'r-');
%     else
%         plot_circle(16.5,16.5,11,'r-','LineWidth',5);
%     end
%     hold off
%     figure(2);
%     b = B_avg + X(i,1:d2)*V(:,1:d2)';
%     imshow(reshape(b,[32,32])');
%     hold on
% %     if X2(i,:)*w' > 0
%     if SL(i)
%         plot_circle(16.5,16.5,11,'g-','LineWidth',5);
%         plot(P(i,1), P(i,2), 'ro');
%         plot([P(i,1) P(i,1)+3*P2(i,1)], [P(i,2) P(i,2)+3*P2(i,2)], 'r-');
%         p4 = [X4(i,:)*wx4, X4(i,:)*wy4];
%         dp4 = [X4(i,:)*ux4, X4(i,:)*uy4];
%         pp4 = [p4; p4+3*dp4/norm(dp4)];
%         plot(p4(1), p4(2), 'yo');
%         plot(pp4(:,1), pp4(:,2), 'y-');
%         
%         p5 = image_find_local_mode(reshape((1-b).*(B_avg>0),[32,32])', p4, 2, 4);
%         dp5 = dp4;
%         pp5 = [p5; p5+3*dp5/norm(dp5)];
%         plot(p5(1), p5(2), 'co');
%         plot(pp5(:,1), pp5(:,2), 'c-');
%         
% %         p5 = [X5(i,:)*wx5, X5(i,:)*wy5];
% %         dp5 = [X5(i,:)*ux5, X5(i,:)*uy5];
% %         pp5 = [p5; p5+3*dp5/norm(dp5)];
% %         plot(p5(1), p5(2), 'co');
% %         plot(pp5(:,1), pp5(:,2), 'c-');
%     else
%         plot_circle(16.5,16.5,11,'r-','LineWidth',5);
%     end
%     hold off
%     input(':');
% end


write_matrix(B_avg, 'logo_b_avg.txt');
write_matrix(V(:,1:20)', 'logo_principal_components.txt');
write_matrix([wx4 wy4 ux4 uy4]', 'logo_regression_coeffs.txt');
write_matrix(S.SupportVectors, 'logo_svm_vectors.txt');
write_matrix(S.Alpha', 'logo_svm_alpha.txt');
write_matrix([S.Bias, S.ScaleData.shift, S.ScaleData.scaleFactor], 'logo_svm_params.txt');


