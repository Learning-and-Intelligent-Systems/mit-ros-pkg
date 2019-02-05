% simulate ball trajectories with random noise 

b0 = [0;0;0; -2;0;2];
dt = .01;
NT = 10;
NB = 10;

for NT=1:20

paddle_vx = 1.5;

B = [];
B_final = [];
for i=1:NB
    B(1,:,i) = (b0 + [0;0;0; -.05*i; 0; 0])';
    for j=2:NT
        B(j,:,i) = ball_dynamics(B(j-1,:,i)', dt)';
    end
    B_final(i,:) = B(end,:,i);
end
B_final

figure(1); clf, hold on
for i=1:NB
    plot(B(:,1,i), B(:,3,i), '-');
end
plot(B_final(:,1), B_final(:,3), 'o');

B_hit = [];
x0 = B_final(end,1);
for i=1:NB
    dx = B_final(i,1) - x0;
    vx = paddle_vx - B_final(i,4);
    t = dx/vx;
    B_hit(i,:) = ball_dynamics(B_final(i,:)', t)';
    %plot([B_hit(i,1) B_final(i,1)], [B_hit(i,3) B_final(i,3)], 'b-');
end
plot(B_hit(:,1), B_hit(:,3), 'ro');
hold off
axis equal
axis([-.5 0 0 .3]);

pause(.5);

end