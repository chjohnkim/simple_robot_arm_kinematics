%Description: PUMA-type 6R robot arm forward kinematics
%Input: Vector of joint angles where first element is joint angle of base
%Output: Pose of all the links

function T = fk_PUMA(joint_angles)

%Physical parameters of robot
L1 = 0.425; 
L2 = 0.392;
W1 = 0.109;
W2 = 0.0;
H1 = 0.0;
H2 = 0.0;

%Home configuration of joints when all joint angles are 0
joint_home=cell(1,7);
joint_home{1} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
joint_home{2} = [1 0 0 -W1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
joint_home{3} = [1 0 0 -W1; 0 1 0 L1; 0 0 1 0; 0 0 0 1];
joint_home{4} = [1 0 0 -W1; 0 1 0 L1+L2; 0 0 1 0; 0 0 0 1];
joint_home{5} = [1 0 0 -W1; 0 1 0 L1+L2; 0 0 1 0; 0 0 0 1];
joint_home{6} = [1 0 0 -W1; 0 1 0 L1+L2; 0 0 1 0; 0 0 0 1];

%Joint screws
joint_screws=cell(1,6);
joint_screws{1} = [0 0 1 0 0 0]';
joint_screws{2} = [1 0 0 0 0 0]';
joint_screws{3} = [1 0 0 0 0 -L1]';
joint_screws{4} = [0 0 1 L1+L2 W1 0]';
joint_screws{5} = [0 1 0 0 0 -W1]';
joint_screws{6} = [1 0 0 0 0 -L1-L2]';

exponentials=cell(1,6);
for i=1:6
    exponentials{i} = expm(skew_symmetric(joint_screws{i})*joint_angles(i));
end

%Final transforms of each link w.r.t. world
T=cell(1,6);
for i=1:6
    T{i} = exponentials{1};
end
for i=2:6
    for j=2:i
        T{i}=T{i}*exponentials{j};
    end
    T{i}=T{i}*joint_home{i};
end

end