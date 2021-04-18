function uout = solve_cbf(u_ref, s, sc, r1, r2, curvature_center, r_safe)
%SOLVE_CBF solve the optimization problem for cbf constriant

phi = s(1);
v = s(2);
x = s(3);
y = s(4);
c_x = curvature_center(1);
c_y = curvature_center(2);
r1 = r1 + 1;
r2 = r2 - 1;

num = length(sc);

A = [0 1; 0 0];
B = [0;1];
p = -[1, 5];
K = place(A,B,p);

a0 = K(1);
a1 = K(2);

cvx_begin quiet
    variable u_a
    variable u_w
    variable p
    minimize( norm( u_w - u_ref(1)) + 5*norm( u_a - u_ref(2)))
    subject to
        u_w >= -pi;
        u_w <= pi; 
        h = 0.5*(x-c_x)^2 + 0.5*(y-c_y)^2;
        dh = (x - c_x)*v*cos(phi) + (y - c_y)*v*sin(phi);
        ddh = v^2 + (x - c_x)*(u_a*cos(phi) - v*sin(phi)*u_w) + (y - c_y)*(u_a*sin(phi) + v*cos(phi)*u_w);

        ddh + a1*dh + a0*(h - 0.5*r1^2) >=0;
        -ddh - a1*dh + a0*(0.5*r2^2 - h) >=0;
        for k = 1:num
            xc = sc{k}(3);
            yc = sc{k}(4);
            vc = -sc{k}(2);
            phic = sc{k}(1);
            ddh = (v*cos(phi)-vc*cos(phic))^2 + (x - xc)*(u_a*cos(phi)-v*sin(phi)*u_w) ...
            + (v*sin(phi)-vc*sin(phic))^2 + (y - yc)*(u_a*sin(phi)+v*cos(phi)*u_w);
            dh = (x-xc)*(v*cos(phi)-vc*cos(phic)) + (y-yc)*(v*sin(phi)-vc*sin(phic));
            h = 1/2*(x-xc)^2 + 1/2*(y-yc)^2 -2*r_safe^2;
            ddh + a1*dh + a0*h >= 0;
        end
cvx_end


uout = [u_w, u_a];

end

