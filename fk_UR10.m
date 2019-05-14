%Description: Universal Robots' UR10 6R robot arm forward kinematics
%Input: Vector of joint angles where first element is joint angle of base
%Output: Pose of all the links

function T = fk_UR10(joint_angles)

%Physical parameters of robot
L1 = 0.425; 
L2 = 0.392;
W1 = 0.109;
W2 = 0.082;
H1 = 0.089;
H2 = 0.095;

%Home configuration of joints when all joint angles are 0
joint_home=cell(1,7);
joint_home{1} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
joint_home{2} = [-1 0 0 0; 0 0 1 W1; 0 1 0 H1; 0 0 0 1];
joint_home{3} = [-1 0 0 L1; 0 0 1 W1; 0 1 0  H1; 0 0 0 1];
joint_home{4} = [-1 0 0 L1+L2; 0 0 1 W1; 0 1 0 H1; 0 0 0 1];
joint_home{5} = [-1 0 0 L1+L2; 0 1 0 W1; 0 0 -1 H1-H2; 0 0 0 1];
joint_home{6} = [-1 0 0 L1+L2; 0 0 1 W1+W2; 0 1 0 H1-H2; 0 0 0 1]; 

%Joint screws
joint_screws=cell(1,6);
joint_screws{1} = [0 0 1 0 0 0]';
joint_screws{2} = [0 1 0 -H1 0 0]';
joint_screws{3} = [0 1 0 -H1 0 L1]';
joint_screws{4} = [0 1 0 -H1 0 L1+L2]';
joint_screws{5} = [0 0 -1 -W1 L1+L2 0]';
joint_screws{6} = [0 1 0 H2-H1 0 L1+L2]';

exponentials=cell(1,6);
for i=1:6
    exponentials{i} = expm(skew_symmetric(joint_screws{i})*joint_angles(i));
end

%Final transforms of each link w.r.t. world
T = cell(1,6);
T{1}= exponentials{1}*joint_home{1};
T{2}= exponentials{1}*exponentials{2}*joint_home{2};
T{3}= exponentials{1}*exponentials{2}*exponentials{3}*joint_home{3};
T{4}= exponentials{1}*exponentials{2}*exponentials{3}*exponentials{4}*joint_home{4};
T{5}= exponentials{1}*exponentials{2}*exponentials{3}*exponentials{4}*exponentials{5}*joint_home{5};
T{6}= exponentials{1}*exponentials{2}*exponentials{3}*exponentials{4}*exponentials{5}*exponentials{6}*joint_home{6};

end