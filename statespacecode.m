%symmetric motion 
load('variables.mat')
%%
C1s= [CXu/V0 CXa CZ0 CXq*c/V0;
    CZu/V0 CZa -CX0 (CZq+2*muc)*c/V0;
    0 0 0 c/V0;
    Cmu/V0 Cma*c/V0 0 Cmq*c/V0];
C2s= [-2*muc*c/V0^2 0 0 0;
    0 (CZa-2*muc)*c/V0 0 0;
    0 0 -c/V0 c/V0;
    0 Cma*c/V0 0 -2*muc*KY2*(c/V0)^2];
C3s= [-CXde;-CZde;0;-Cmde];
As=-C1s^-1*C2s;
Bs=-C1s^-1*C3s;
% C1a = [((CYbdot-2*mub)*(b/V0)) 0 0 0;
%     0 -(b/(2*V0)) 0 0;
%     0 0 (-2*mub*KX2*((b^2)/(V0^2))) (4*mub*KXZ*((b^2)/(V0^2)));
%     (Cnbdot*(b/V0)) 0 (2*mub*KXZ*((b^2)/(V0^2))) (-2*mub*KZ2*((b^2)/(V0^2)))];
% C2a = [CYb CL (CYp*b/(2*V0)) ((CYr -4*mub)*b/(2*V0));
%     0 0 (b/(2*V0)) 0;
%     Clb 0 (Clp*b/(2*V0)) (Clr*b/(2*V0));
%     Cnb 0 (Cnp*b/(2*V0)) (Cnr*b/(2*V0))];
% C3a = [CYda CYdr;
%     0 0;
%     Clda Cldr;
%     Cnda Cndr];
% Aa = -(C1a^-1)*C2a;
% Ba = -(C1a^-1)*C3a;
%asymmetric
% Ca = [1 0 0 0;
%     0 1 0 0;
%     0 0 1 0;
%     0 0 0 1];
% Da = [0 0 0 0;
%     0 0 0 0]';
% sysa = ss(Aa,Ba,Ca,Da);
% ta = 0:0.001:20;
% ua = zeros(length(ta),2);
% ua(1000,2) = 0.01;
% ya = lsim(sysa,ua,ta);
% plot(ta,ya)
%symmetric
Cs = [1 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0];
Ds = [0 0 0 0]';
syss = ss(As,Bs,Cs,Ds);
ts = 0:0.001:10;
ys = step(syss,ts);
plot(ts,ys)