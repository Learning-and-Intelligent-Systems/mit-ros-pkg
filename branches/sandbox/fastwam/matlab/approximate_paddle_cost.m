function [g,J,H,A] = approximate_paddle_cost(q0, dq0, p_des, dp_des, c, d)
%[g,J,H,A] = approximate_paddle_cost(q0, dq0, p_des, dp_des, c, d)
%
% Given costs of the form:
%
%   g(p,dp) = (p-p_des)'*C*(p-p_des) + (dp-dp_des)'*D*(dp-dp_des)
%
% derive a quadratic Taylor approximation in terms of x=[q,dq]'
%
%   g(x0) + (d/dx)g(x0)*(x-x0) + (1/2)*(x-x0)'*(d2/ddx)g(x0)*(x-x0)
%
% Returns g = g(x0), J = (d/dx)g(x0), and H = (d2/ddx)g(x0), and A such that
%
%   [1,x]'*A*[1,x] = g(x0) + J*(x-x0) + (1/2)*(x-x0)'*H*(x-x0)

p0 = arm_kinematics(q0, 7, [0,0,.1], 1);
n0 = arm_kinematics(q0, 7, [1,0,.1], 1) - p0;
p0 = [p0;n0];
dp0 = paddle_jacobian(q0)*dq0;

g = (p0-p_des)'*diag(c)*(p0-p_des) + (dp0-dp_des)'*diag(d)*(dp0-dp_des);
J = approximate_paddle_cost_gradient(q0, dq0, p_des, dp_des, c, d);
H = approximate_paddle_cost_hessian(q0, dq0, p_des, dp_des, c, d);

x0 = [q0;dq0];
A = [g - J*x0 + x0'*H*x0/2,  (J - x0'*H)/2;   (J' - H*x0)/2,  H/2];
