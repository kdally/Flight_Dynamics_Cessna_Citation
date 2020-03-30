function y = myspace(x,elevatordeflection,ts,pert0,alpha0,theta0,rate0)
C2s= [x(1)/x(18) x(2) x(3) x(4)*x(17)/x(18);
    x(5)/x(18) x(6) -x(7) (x(8)+2*x(16))*x(17)/x(18);
    0. 0. 0. x(17)/x(18);
    x(9)/x(18) x(15) 0. x(10)*x(17)/x(18)];
C1s= [-2*x(16)*x(17)/x(18)^2 0 0 0;
    0 (x(11)-2*x(16))*x(17)/x(18) 0 0;
    0 0 -x(17)/x(18) 0;
    0 x(12)*x(17)/x(18) 0 -2*x(16)*x(19)*(x(17)/x(18))^2];
C3s= [x(13);x(14);0;x(20)];
As=-C1s^-1*C2s;
Bs=-C1s^-1*C3s;
Cs = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
Ds = [0 0 0 0]';

syss = ss(As,Bs,Cs,Ds);
u=elevatordeflection;
z = lsim(syss,u,ts,[pert0 alpha0 theta0 rate0]');
y = z;

end