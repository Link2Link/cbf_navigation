clear
clc
close all
init

cur_road = 12;
c1 = [0; cur_road];
c2 = [2*sin(pi/3)*cur_road; cur_road*(1-2*cos(pi/3))];
curvature_center{1} = c1;
curvature_center{2} = c2;
r1 = cur_road-3;
r2 = cur_road+3;
f_handle = figure('Units','characters','Position',[30 10 300 80]);
hold on
axis equal

draw_map(r1, r2, c1, [-pi, -pi/6 + 1e-2], f_handle)
draw_map(r1, r2, c2, [pi-pi/6 + 1e-2, -2*pi], f_handle)

sc = generate_cars([7,10,8], c2, r1, r2);


s = [-pi/2; 8; -10; 10];


h_vehicle = plot(0,0, 'r');
for i=1:length(sc)
    h_sc{i} = plot(0,0, 'g');
end
h = animatedline;
h.LineStyle = '--';
h.Color = 'r';
    

dt = 5e-2;
idx = 1;
for current = 0:dt:10
   phi = s(1);
   v = s(2);
   x = s(3);
   y = s(4);
   target = 10;
   ua_ref = (target - v) * 5;

   if judge_center([x;y], c1, c2) == 2
       idx = 2;
   end
   r = norm([x;y] - curvature_center{idx});
   curventer = 1/r;
   
   K = 25;
   if idx ==1
       uw_ref = curventer * K;
   else
       uw_ref = -curventer * K;
   end
   
   u_ref = [uw_ref, ua_ref];
   if idx == 1
       u = solve_cbf(u_ref, s, [], r1, r2, curvature_center{idx}, 1.5);
   else
       u = solve_cbf(u_ref, s, sc, r1, r2, curvature_center{idx}, 1.5);
   end
       [t, s] = ode45(@(t,s) Dyn_car(t,s,u), [0, dt], s);
   s = s(end, :)';
   p = s(3:4, :);
   phi = s(1);
   draw_car(p, phi, h_vehicle)
   addpoints(h,s(3),s(4));
   
   sc = update_cars(sc, dt, c2);
   
   for i=1:length(sc)
       draw_car(sc{i}(3:4), sc{i}(1), h_sc{i}); 
   end
end