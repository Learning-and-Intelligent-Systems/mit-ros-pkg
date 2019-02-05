
% Given costs of the form:
%
%   g(p,dp) = (p-p0)'*C*(p-p0) + (dp-dp0)'*D*(dp-dp0)
%
% derive a quadratic Taylor approximation in terms of x=[q,dq]'
%
%   g(x0) + (d/dx)g(x0)*(x-x0) + (1/2)*(x-x0)'*(d2/ddx)g(x0)*(x-x0)

syms q1 q2 q3 q4 q5 q6 q7 real           % joint angles
syms dq1 dq2 dq3 dq4 dq5 dq6 dq7 real    % joint velocities
syms c1 c2 c3 c4 c5 c6 real              % paddle state costs
syms d1 d2 d3 d4 d5 d6 real              % paddle velocity costs
syms p01 p02 p03 p04 p05 p06 real        % goal paddle state
syms dp01 dp02 dp03 dp04 dp05 dp06 real  % goal paddle velocities

q = [q1 q2 q3 q4 q5 q6 q7]';
dq = [dq1 dq2 dq3 dq4 dq5 dq6 dq7]';
x = [q;dq];
C = diag([c1 c2 c3 c4 c5 c6]);
D = diag([d1 d2 d3 d4 d5 d6]);
p0 = [p01 p02 p03 p04 p05 p06]';
dp0 = [dp01 dp02 dp03 dp04 dp05 dp06]';

p = arm_kinematics(q, 7, [0,0,.1], 1);
n = arm_kinematics(q, 7, [1,0,.1], 1) - p;
p = [p;n];
dp = paddle_jacobian(q)*dq;

g = (p-p0)'*C*(p-p0) + (dp-dp0)'*D*(dp-dp0);
J = jacobian(g,x);
H = hessian(g,x);

f = fopen('J.txt', 'w');
for i=1:7
    fprintf(f, 'sq%d = sin(q(%d));\n', i, i);
    fprintf(f, 'cq%d = cos(q(%d));\n', i, i);
end
fprintf(f, '\nJ = zeros(1,14);\n\n');
for i=1:14
    s = char(J(i));
    for j=1:7
        s = strrep(s, sprintf('cos(q%d)',j), sprintf('cq%d',j));
        s = strrep(s, sprintf('sin(q%d)',j), sprintf('sq%d',j));
    end
    fprintf(f, 'J(%d) = ', i);
    fprintf_formula(f,s);
    fprintf(f, ';\n\n');
    %fprintf(f, 'J(%d) = %s;\n\n', i, s);
end
fclose(f);

f = fopen('H.txt', 'w');
for i=1:7
    fprintf(f, 'sq%d = sin(q(%d));\n', i, i);
    fprintf(f, 'cq%d = cos(q(%d));\n', i, i);
end
fprintf(f, '\nH = zeros(14,14);\n\n');
for i=1:14
    for j=1:i
        s = char(H(i,j));
        for k=1:7
            s = strrep(s, sprintf('cos(q%d)',k), sprintf('cq%d',k));
            s = strrep(s, sprintf('sin(q%d)',k), sprintf('sq%d',k));
        end
        fprintf(f, 'H(%d,%d) = ', i, j);
        fprintf_formula(f,s);
        fprintf(f, ';\n\n');
        %fprintf(f, 'H(%d,%d) = %s;\n\n', i, j, s);
    end
end
fprintf(f, 'for i=1:14, for j=i+1:14, H(i,j) = H(j,i); end, end\n\n');
fclose(f);
