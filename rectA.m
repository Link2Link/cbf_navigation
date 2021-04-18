function h=rectA(x0,y0,w,h,A);
% 画倾斜的的矩形
% x0是旋转前矩形左下角横坐标
% y0是旋转前矩形左下角纵坐标
% w是宽度
% h是高度
% A是旋转角度


X=[x0,x0+w,x0+w,x0,x0];
Y=[y0,y0,y0+h,y0+h,y0];
Z=X+Y*i;
Z=[Z-[X(1)+Y(1)*i]]*exp(i*A)+[X(1)+Y(1)*i];
h=plot(Z);