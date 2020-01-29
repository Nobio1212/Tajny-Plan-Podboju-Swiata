clear all
u  = [0 0 0]';
w  = [0 0 1]';
p0 = [5 4 0 1]';

u1  = [5 4 0]';
w1  = [0 0 1]';
p01 = [10 12 0 1]';

k=1;
theta=0:0.1:2*pi;
theta1=linspace(0,2*pi,length(theta));
s  = [w; cross(-w, u)];
s1 = [w1; cross(-w1, u1)];
hat = @(s)[  0   -s(3)  s(2)  s(4)
           s(3)   0     -s(1) s(5)
           -s(2) s(1)     0   s(6)
             0    0       0    0];
while(true)
for i = 1:length(theta)
    p  = expm(hat(s)*theta(i))*p0;
    p1 = expm(hat(s)*theta(i))*expm(hat(s1)*theta1(i))*p01;
    pp1(k)=p(1);
    pp2(k)=p(2);
    pk1(k)=p1(1);
    pk2(k)=p1(2);
    dtheta=kin(10,theta);
    dp = hat(s)*dtheta(1)*p
    dp1 = [hat(s) *p1,  expm(hat(s)*theta(i))*hat(s1)*expm(-hat(s1)*theta1(i))*p1]*dtheta(2)
    
    
    plot([0 p(1) p1(1)], [0 p(2) p1(2)])
    hold on
    plot(p1(1),p1(2),'ro')
    plot(pp1,pp2,'r.',pk1,pk2,'g.');
    hold off
    axis([-20 20 -20 20]);
    grid on
    pause(0.1);
    k=k+1;
end
end
    