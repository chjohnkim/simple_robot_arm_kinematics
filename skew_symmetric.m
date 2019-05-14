%Description: Turns screw vector into skew_symmetric
%Input: screw vector (R^6)
%Output: skew symmetric (R^4x4)
function S = skew_symmetric(screw)
S = zeros(4);
S(3,2) = screw(1);
S(2,3) = -screw(1);
S(3,1) = -screw(2);
S(1,3) = screw(2);
S(2,1) = screw(3);
S(1,2) = -screw(3);
S(1:3, 4) = screw(4:6);
end
