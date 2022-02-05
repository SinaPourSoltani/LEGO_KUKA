function T = DH2trans(DH_row)

theta = DH_row(4);
a = DH_row(2);
d = DH_row(3);
alpha = DH_row(1);

T = eye(4);
T(1,1) = cos(theta);
T(1,2) = -sin(theta) * cos(alpha);
T(1,3) = sin(theta) * sin(alpha);
T(1,4) = a * cos(theta);
T(2,1) = sin(theta);
T(2,2) = cos(theta) * cos(alpha);
T(2,3) = -cos(theta) * sin(alpha);
T(2,4) = a * sin(theta);
T(3,2) = sin(alpha);
T(3,3) = cos(alpha);
T(3,4) = d;

end
