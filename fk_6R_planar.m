%Description: Given angles of joint arms, find pose of 3R-Robot Arm
%Input: Vector of joint angles where first element is joint angle of base
%Output: Pose of all the links

function T = fk_6R_planar(joint_angles)

%Physical parameters of robot (i.e. length of each link, where L1 is first
%link from base)
L = [1 1 1 1 1 1]

%Home configuration of joints when all joint angles are 0
joint_home=cell(1,6);
for i = 1:6
    joint_home{i} = eye(4);
    joint_home{i}(1,4)= sum(L(1:i-1));
end

%Intermediate transforms 
joint_screws=cell(1,6);
exponentials=cell(1,6);
for i=1:6
    joint_screws{i} = zeros(4);
    joint_screws{i}(2,1) = 1;
    joint_screws{i}(1,2) = -1;
    joint_screws{i}(2,4) = -sum(L(1:i-1));
    exponentials{i} = expm(joint_screws{i}*joint_angles(i));
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