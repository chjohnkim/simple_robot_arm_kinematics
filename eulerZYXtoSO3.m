function R = eulerZYXtoSO3(alpha, beta, gamma)
R = zeros(3);
R(1,1) = cos(alpha)*cos(beta);
R(2,1) = sin(alpha)*cos(beta);
R(3,1) = -sin(beta);
R(1,2) = cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma);
R(2,2) = sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma);
R(3,2) = cos(beta)*sin(gamma);
R(1,3) = cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
R(2,3) = sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
R(3,3) = cos(beta)*cos(gamma);
end
