function uout = solve_cbf(u_ref, s, sc, L, R, r)
%SOLVE_CBF solve the optimization problem for cbf constriant

phi = s(1);
v = s(2);
x = s(3);
y = s(4);


phic = sc(1, :);
vc = sc(2, :);
xc = sc(3, :);
yc = sc(4, :);
num = size(sc,2);

A = [0 1; 0 0];
B = [0;1];
p = -[1, 3];
K = place(A,B,p);

a0 = K(1);
a1 = K(2);

cvx_begin quiet
    variable u_a
    variable u_p
    variable p(3,1)
    minimize( norm( u_p - u_ref(1)) + 5*norm( u_a - u_ref(2))- [0.1,0.1,0.1]*p)
    subject to
        cos(phi)*u_a-sin(phi)*v*u_p + a0*(x-L-r) + a1*v*cos(phi) >=p(1);
        -cos(phi)*u_a + sin(phi)*v*u_p + a0*(-x+R-r) - a1*v*cos(phi) >=p(2);

        for k =1:num
            ddh = (v*cos(phi)-vc(k)*cos(phic(k)))^2 + (x - xc(k))*(u_a*cos(phi)-v*sin(phi)*u_p) ...
                + (v*sin(phi)-vc(k)*sin(phic(k)))^2 + (y - yc(k))*(u_a*sin(phi)+v*cos(phi)*u_p);
            dh = (x-xc(k))*(v*cos(phi)-vc(k)*cos(phic(k))) + (y-yc(k))*(v*sin(phi)-vc(k)*sin(phic(k)));
            h = 1/2*(x-xc(k))^2 + 1/2*(y-yc(k))^2 -2*r^2;
            ddh + a1*dh + a0*h >=p(3);
            
%             ((x-xc(k))*cos(phi)+(y-yc(k))*sin(phi))*u_a + v^2 + v*(-(x-xc(k))*sin(phi)+(y-yc(k))*cos(phi))*u_p...
%             +a0*(0.5*(x-xc(k))^2+0.5*(y-yc(k))^2-2*r^2) + ...
%             a1*((x-xc(k))*cos(phi)+(y-yc(k))*sin(phi))*v >=p(3);
        end
        u_p <= pi/6;
        u_p >= -pi/6;
        p >= 0;
cvx_end


uout = [u_p, u_a];

end

