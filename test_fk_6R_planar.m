clc;
clear;
joint_angles=zeros(1,6);
joint_angles=[-pi/2,-pi/2,pi/4,pi/4,-pi/4,pi/6];
T = fk_6R_planar(joint_angles);

plot_data = zeros(3,6);
for i=1:6
    plot_data(:,i) = T{i}(1:3,4);
    ref_axis(T{i}, 0.5, 1.5);
end
scatter3(plot_data(1,:), plot_data(2,:), plot_data(3,:)); 
line(plot_data(1,:), plot_data(2,:), plot_data(3,:));
axis([-4 4 -4 4 -4 4]);
ref_axis(eye(4), 0.5, 1.5);
