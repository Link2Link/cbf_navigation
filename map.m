clear
clc
u_ref = [0, 1];
s = [0,1,1]';
sc = [0, 0, 5]';
L = -5;
R = 5;
r = 1;

x = -5:0.5:5;
y = 4:0.1:6;
[X,Y] = meshgrid(x,y);
U = zeros(size(X));
for i = 1:length(X(:))
    x = X(i);
    y = Y(i);
    s = [0,x,y]';
    u = solve_cbf(u_ref, s, sc, L, R, r);
    U(i) = u(1);
    i/length(X(:))
end
