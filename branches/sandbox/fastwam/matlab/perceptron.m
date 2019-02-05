function w = perceptron(X,y)
%w = perceptron(X,y)

n = size(X,1);
d = size(X,2);

w = zeros(1,d);

for i=1:n
    x = X(i,:)/norm(X(i,:));
    if sign(x*w') ~= y(i)
        w = w + y(i)*x;
    end
end
