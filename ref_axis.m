%Description: Given transformation, plots 3-D reference axis
%Input: Homogenious transformation matrix
%Output: Reference axis 
function ref_axis(T, scale, line_width)

origin = [0 0 0 1];

x = [T*origin' T*[scale 0 0 1]'];
y = [T*origin' T*[0 scale 0 1]'];
z = [T*origin' T*[0 0 scale 1]'];

hold on;
plot3(x(1,:),x(2,:),x(3,:),'r', 'LineWidth', line_width);
plot3(y(1,:),y(2,:),y(3,:),'g', 'LineWidth', line_width);
plot3(z(1,:),z(2,:),z(3,:),'b', 'LineWidth', line_width);

end
