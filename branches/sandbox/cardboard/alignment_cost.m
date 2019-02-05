function [f g] = alignment_cost(x,C,D1,D2,D3) %,XI,YI,ZI)
    xi = round(x);
    xi(1) = 1 + mod(xi(1)-1,size(C,1));
    i = sub2ind(size(C),xi(1),xi(2),xi(3));
    f = C(i);
    g = [D1(i); D2(i); D3(i)];
    %f = interp3(XI,YI,ZI,C,x(1),x(2),x(3));
    %g = zeros(3,1);
    %g(1) = interp3(XI,YI,ZI,D1,x(1),x(2),x(3));
    %g(2) = interp3(XI,YI,ZI,D1,x(1),x(2),x(3));
    %g(3) = interp3(XI,YI,ZI,D1,x(1),x(2),x(3));
end
