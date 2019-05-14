clc;
clear;
joint_angles=zeros(1,6);
joint_angles=[pi/2 pi/2 pi/2 pi/2 0 0];
T = fk_UR10(joint_angles);


plot_data = zeros(3,6);
for i=1:6
    plot_data(:,i) = T{i}(1:3,4);
    ref_axis(T{i}, 0.1, 1.5);
end

%Construct World Environment
ref_axis(eye(4), 0.2, 3);
scatter3(plot_data(1,:), plot_data(2,:), plot_data(3,:)); 
line(plot_data(1,:), plot_data(2,:), plot_data(3,:), 'LineWidth',10);
a=1.2;
axis([-a a -a a -a a]);
[x y] = meshgrid(-a:a/2:a); % Generate x and y data
z = zeros(size(x, 1))-0.1; % Generate z data
surf(x, y, z, 'FaceColor',[0.82 0.82 0.82]) % Plot the surface
