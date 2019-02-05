function b2 = ball_dynamics(b,dt)
%b2 = ball_dynamics(b,dt) -- b = [x,y,z,vx,vy,vz]

r = .02;                 % ball radius
g = 9.8;                % gravity
%ball_coeff_rest = .88;   % coefficient of restitution
p = 1.204;               % air density at 20 celcius
C = .1;              % drag coeff. for smooth sphere
m = .0027;               % ball mass
A = pi*r^2;
Cd = .5*C*p*A/m;
Cm = .0004;


x = b(1:3);
v = b(4:6);
s = norm(v);

if length(b) >= 9  % ball state includes spin
    w = b(7:9);
    dvdt = -Cd*s*v + Cm*s*cross(w,v) - [0 0 g]';
    b2 = [x+dt*v;  v+dt*dvdt;  w];
else
    dvdt = -Cd*s*v - [0 0 g]';
    b2 = [x+dt*v;  v+dt*dvdt];
end
