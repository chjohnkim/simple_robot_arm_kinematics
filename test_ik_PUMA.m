clc;
clear;

%Input: end-effector pose
alpha = 0;
beta = 0;
gamma = 0;
x_pos = 0.4;
y_pos = 0.4;
z_pos = 0.4;

R = eulerZYXtoSO3(alpha, beta, gamma);
X=eye(4);
X(1:3, 1:3) = R;
X(1,4) = x_pos;
X(2,4) = y_pos;
X(3,4) = z_pos;

theta = ik_PUMA(X)
joint_angles=[theta(1,:)];
T = fk_PUMA(joint_angles);


plot_data = zeros(3,6);
for i=1:6
    plot_data(:,i) = T{i}(1:3,4);
end
ref_axis(T{1}, 0.1, 1.5);
ref_axis(T{6}, 0.1, 1.5);

%Construct World Environment
ref_axis(eye(4), 0.2, 3);
scatter3(plot_data(1,:), plot_data(2,:), plot_data(3,:)); 
line(plot_data(1,:), plot_data(2,:), plot_data(3,:), 'LineWidth',5);
a=1.2;
axis([-a a -a a -a a]);
[x y] = meshgrid(-a:0.4:a); % Generate x and y data
z = zeros(size(x, 1))-0.1; % Generate z data
surf(x, y, z, 'FaceColor',[0.82 0.82 0.82]) % Plot the surface
