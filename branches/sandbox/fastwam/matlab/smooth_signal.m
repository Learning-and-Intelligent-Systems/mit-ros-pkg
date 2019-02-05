function y = smooth_signal(x, alpha)
%y = smooth_signal(x, alpha) -- smooth with exponential moving average

y = x;
for i=2:size(y,1)
    y(i,:) = alpha*y(i,:) + (1-alpha)*y(i-1,:);
end
