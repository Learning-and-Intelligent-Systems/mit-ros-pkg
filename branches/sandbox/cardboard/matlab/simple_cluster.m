function L = simple_cluster(X,d)
%L = simple_cluster(X,d) -- returns a set of labels for data point X(i,:),
%clustered with a simple distance cutoff, d.


n = size(X,1);
L = zeros(1,n);

L(1) = 1;
cnt = 1;
for i=2:n
    DX = X(1:i-1,:) - repmat(X(i,:),[i-1 1]);
    D = sqrt(sum(DX.*DX,2));
    [dmin imin] = min(D);
    if dmin < d
        L(i) = L(imin);
        for j=1:i-1
            if D(j) < d && L(j) ~= L(i)
                L(L==L(j)) = L(i);
            end
        end
    else
        cnt = cnt+1;
        L(i) = cnt;
    end
end

U = unique(L);
for i=1:length(U)
    L(L==U(i)) = i;
end

