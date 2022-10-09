function plot_solution(r_x,r_y,r_theta,r_s,v_s,v_n)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
r_lx = r_x - sin(r_theta);
r_ly = r_y + cos(r_theta);
r_rx = r_x + sin(r_theta);
r_ry = r_y - cos(r_theta);

v_theta = interp1(r_s,r_theta,v_s);
v_x = interp1(r_s,r_x,v_s) - v_n.*sin(v_theta);
v_y = interp1(r_s,r_y,v_s) + v_n.*cos(v_theta);

figure;
hold on;
plot(r_x,r_y, 'b');
plot(r_lx,r_ly, 'r');
plot(r_rx,r_ry, 'r');
plot(v_x,v_y,'g');
axis equal;

end