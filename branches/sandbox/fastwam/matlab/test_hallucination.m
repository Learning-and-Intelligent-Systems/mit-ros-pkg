
% b=[0,.5,.2];
% 
% n=100;
% TABLE_WIDTH = 1.525;
% [Q,U,T] = hallucination_swing([(TABLE_WIDTH - b(2))*ones(n,1), .01*[89:-1:-10]', b(3)*ones(n,1)]);
% 
% P = zeros(size(Q,1),3); for i=1:size(Q,1), P(i,:) = arm_kinematics(Q(i,:), 7, [0,0,.1], 1); end
% figure(1); for i=1:10:size(Q,1), [a1,a2] = view(); draw_table, hold on, draw_arm(Q(i,:)); view(a1,a2); drawnow; end
% %hold on, [a1,a2] = view(); draw_table, hold on, draw_arm(Q(350,:)); view(a1,a2); hold off;
% hold on, [a1,a2] = view(); draw_table, hold on, draw_arm(Q(250,:)); view(a1,a2); hold off;
% hold on, plot3(P(:,1), P(:,2), P(:,3), 'r-'); hold off;
% %hold on, for i=150:50:350, plot3(P(i,1), P(i,2), P(i,3), 'r.'); plot3(b(1)-(.7-.002*i), b(2), b(3)-.2*(.7-.002*i), 'bo'); end, hold off;
% hold on, for i=50:50:250, plot3(P(i,1), P(i,2), P(i,3), 'r.'); plot3(b(1)-(.25-(.002*i)^2), b(2), b(3)-.2*(.25-(.002*i)^2), 'bo'); end, hold off;
% figure(2); plot(U(1:end-1,:)); legend('1','2','3','4','5','6','7');
% %    figure(3); for j=1:4, subplot(4,1,j); plot(dt*[0:NT], Q(:,j),'-'); hold on, plot(Ti-Ti(1),Qi(:,j),'--'); hold off; end
% figure(4); plot(P);
% dP = []; for i=1:3, dP(:,i) = smooth(smooth((P(2:end,i)-P(1:end-1,i))./(T(2:end)-T(1:end-1)))); end
% figure(5); plot(dP);


n=100;
TABLE_WIDTH = 1.525;

Q = {}; U = {}; T = {}; B = {};
for i=1:100
    i
    b=[0,.5,.2] + [0, .5*rand(), .2*rand()]
    B{i} = b;
    [q,u,t] = hallucination_swing([(TABLE_WIDTH - b(2))*ones(n,1), .01*[89:-1:-10]', b(3)*ones(n,1)]);
    swing_err = compute_swing_error(q,t,b,1)
    Q{i} = q;
    U{i} = u;
    T{i} = t;
    if mod(i,10)==0
        save swings3.mat Q U T B
    end
end

