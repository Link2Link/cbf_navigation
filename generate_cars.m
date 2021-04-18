function sc = generate_cars(speeds, c, r1, r2)
num = length(speeds);
theta = rand(num, 1)*2*pi - pi;
r = rand(num, 1)*(r2-r1) + r1;
x = r.*cos(theta);
y = r.*sin(theta);
x = x + c(1);
y = y + c(2);
for i = 1:num
    sc{i} = [theta(i) + pi/2, speeds(i), x(i), y(i)]';
end
end

