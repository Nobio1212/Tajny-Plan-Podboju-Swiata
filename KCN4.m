u  = [0 0 0]';
w  = [0 0 1]';
p0 = [0 4 0 1]';

u1  = [0 4 0]';
w1  = [0 0 1]';
p01 = [0 12 0 1]';

s  = [w; cross(-w, u)];
s1 = [w1; cross(-w1, u1)];
hat = @(s)[  0   -s(3)  s(2)  s(4)
           s(3)   0     -s(1) s(5)
           -s(2) s(1)     0   s(6)
             0    0       0    0];
         
syms theta1 theta2 real;

A1 = expm(hat(s) * theta1);
A2 = expm(hat(s1) * theta2);

p2 = A1*A2*p01;
J = simplify([hat(s)*p2 A1*hat(s1)/A1*p2])
%simplify(J)