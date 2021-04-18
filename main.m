clear
clc

Wl = -6;
Wr = 6;
Y_max = 40;
Y_min = 0;

figure('Units','characters','Position',[30 30 120 60]);

hold on
plot([Wr, Wr], [Y_min, Y_max], 'b')
plot([Wl, Wl], [Y_min, Y_max], 'b')
xlim([Wl-1, Wr+1])
axis equal
x0 = -3;
y0 = 0;
v0 = 0;
fai0 = pi/2;
s0 = [fai0;v0; x0; y0];

xc = 3;
yc = 4;
vc = 0.4;
sc = [pi/2,vc, xc, yc]';

xc2 = 0;
yc2 = 6;
vc2 = 0.8;
sc2 = [pi/2,vc2, xc2, yc2]';

xc3 = -3;
yc3 = 4;
vc3 = 0.6;
sc3 = [pi/2, vc3, xc3, yc3]';

r = 1;
u = [0.1; 1];
%%
dt = 0.3;
T = 35;

s = s0;
log_s = s0;
log_t = 0;
h = animatedline;
p = rectangle('Position',[s(3)-1 s(4)-1 2 2],'Curvature',[1 1]);
obstacle = rectangle('Position',[xc-1 yc-1 2 2],'Curvature',[1 1], 'EdgeColor','red');
obstacle2 = rectangle('Position',[xc2-1 yc2-1 2 2],'Curvature',[1 1], 'EdgeColor','red');
obstacle3 = rectangle('Position',[xc3-1 yc3-1 2 2],'Curvature',[1 1], 'EdgeColor','red');
% car = rectangle('Position',[s(3)-0.5 s(4)-0.5 1 1], 'EdgeColor','r');
for current = 0:dt:T
    angle = s(1);
    speed = s(2);
    ua = pi/2-angle;
    uv = 2-speed;
    u_ref = [ua, uv];
    
    s(3) = s(3) + 0.05 * rand;
    u = solve_cbf(u_ref, s, [sc, sc2, sc3], Wl, Wr, r*1.2);
%     u = [pi/2-angle, 1];
    [t, s] = ode45(@(t,s) Dyn_car(t,s,u), [0, dt], s);
    s = s(end, :)';
    log_s = [log_s, s];
    
    addpoints(h,s(3),s(4));
    p.Position = [s(3)-1 s(4)-1 2 2]; 
    rectangle('Position',[s(3)-1 s(4)-1 2 2],'Curvature',[1 1]);
    
    
    [t, sc] = ode45(@(t,sc) Dyn_car(t,sc,[0,0]), [0, dt], sc);
    sc = sc(end, :)';
    
    [t, sc2] = ode45(@(t,sc2) Dyn_car(t,sc2,[0,0]), [0, dt], sc2);
    sc2 = sc2(end, :)';
    
    [t, sc3] = ode45(@(t,sc3) Dyn_car(t,sc3,[0,0]), [0, dt], sc3);
    sc3 = sc3(end, :)';
    

    obstacle.Position = [sc(3)-1 sc(4)-1 2 2];
    obstacle2.Position = [sc2(3)-1 sc2(4)-1 2 2];
    obstacle3.Position = [sc3(3)-1 sc3(4)-1 2 2];
%     rectangle('Position',[sc(3)-1 sc(4)-1 2 2],'Curvature',[1 1], 'EdgeColor','red');
%     rectangle('Position',[sc2(3)-1 sc2(4)-1 2 2],'Curvature',[1 1], 'EdgeColor','red');
%     rectangle('Position',[sc3(3)-1 sc3(4)-1 2 2],'Curvature',[1 1], 'EdgeColor','red');
%     sc3
%     car.Position = [s(2)-0.5 s(3)-0.5 1 1   ]; 
    drawnow 
%     pause(dt);
end
log_t = [0, (0:dt:T)+dt];
% plot(log_s(2, :), log_s(3, :), 'r')
ylim([0, Y_max])