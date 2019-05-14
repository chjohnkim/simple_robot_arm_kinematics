%Description: Analytical solution of inverse kinematics for PUMA-type 6R
%robot arm. 
%Input: End-effector frame X in SE(3)
%Output: Joint angles
function joint_angles = ik_PUMA(X)

x = X(1,4);
y = X(2,4);
z = X(3,4);

% Each row is a solution
theta = zeros(2,6);

%Physical parameters of robot where W1=W1, L1=L1, L2=L2 in fk
W1 = 0.109; 
L1 = 0.425;
L2 = 0.392; 
W2 = 0.0;
H1 = 0.0;
H2 = 0.0;

%Joint positions of theta_1, theta_2, and theta_3
psi = atan2(y,x);
r=sqrt(x^2+y^2);
alpha = atan2(W1, sqrt((r)^2-(W1)^2));
theta(:,1)= psi-alpha;

D = (x^2+y^2+z^2-W1^2-L1^2-L2^2)/(2*L1*L2);
theta(1,3) = atan2(sqrt(1-D^2), D);
theta(2,3) = atan2(-sqrt(1-D^2), D);

theta(1,2) = atan2(z, sqrt(x^2+y^2-W1^2))-atan2(L2*sin(theta(1,3)), L1+L2*cos(theta(1,3)));
theta(2,2) = atan2(z, sqrt(x^2+y^2-W1^2))-atan2(L2*sin(theta(2,3)), L1+L2*cos(theta(2,3)));

joint_angles = theta;

%Joint positions of theta_4, theta_5, and theta_6
%Home configuration of end-effector when all joint angles are 0
M = [1 0 0 -W1; 0 1 0 L1+L2; 0 0 1 0; 0 0 0 1];

%Joint screws
joint_screws=cell(1,3);
joint_screws{1} = [0 0 1 0 0 0]';
joint_screws{2} = [1 0 0 0 0 0]';
joint_screws{3} = [1 0 0 0 0 -L1]';

exponentials=cell(1,3);
for i=1:3
    exponentials{i} = expm(-skew_symmetric(joint_screws{i})*joint_angles(1,i));
end

R = exponentials{3}*exponentials{2}*exponentials{1}*X*inv(M)

theta(:,4) = atan2(R(2,1), R(1,1));
theta(:,5) = atan2(-R(3,1), sqrt(R(1,1)^2+R(2,1)^2));
theta(:,6) = atan2(R(3,2), R(3,3));

joint_angles = theta;

end
