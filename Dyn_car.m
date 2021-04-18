function ds = Dyn_car(t, s, u)
%DYN_CAR dynamic model for car

r = 1;

fai = s(1);
v = s(2);
x = s(3);
y = s(4);

dfai = u(1);
dv = u(2);
dx = r*cos(fai)*v;
dy = r*sin(fai)*v;

ds = [dfai;dv; dx; dy];
end

