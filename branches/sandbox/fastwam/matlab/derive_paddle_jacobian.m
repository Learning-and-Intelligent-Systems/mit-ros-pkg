syms q1 q2 q3 q4 q5 q6 q7 real
q = [q1 q2 q3 q4 q5 q6 q7];
p = arm_kinematics(q,7,[0;0;.1],1);
n = arm_kinematics(q,7,[1;0;.1],1) - p;
J = jacobian([p;n],q);

s = ccode(J);
for k=1:7
    s = strrep(s, sprintf('cos(q%d)',k), sprintf('c%d',k));
    s = strrep(s, sprintf('sin(q%d)',k), sprintf('s%d',k));
end
s

% for i=1:6
%     for j=1:7
%         s = char(J(i,j));
%         for k=1:7
%             s = strrep(s, sprintf('cos(q%d)',k), sprintf('c%d',k));
%             s = strrep(s, sprintf('sin(q%d)',k), sprintf('s%d',k));
%         end
%         fprintf('J[%d][%d] = %s;\n', i-1, j-1, s);
%     end
% end
