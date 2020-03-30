timebegin=3237;
run('Cit_par_reference');
%%
% C2s= [CXu/V0 CXa CZ0 CXq*c/V0;
%     CZu/V0 CZa -CZ0 (CZq+2*muc)*c/V0;
%     0. 0. 0. c/V0;
%     Cmu/V0 Cma 0. Cmq*c/V0];
% C1s= [-2*muc*c/V0^2 0 0 0;
%     0 (CZadot-2*muc)*c/V0 0 0;
%     0 0 -c/V0 0;
%     0 Cmadot*c/V0 0 -2*muc*KY2*(c/V0)^2];
% C3s= [0;0;0;0];
% As=-(C1s^-1*C2s);
% Bs=-C1s^-1*C3s;
% Cs = [1 0 0 0;
%     0 1 0 0;
%     0 0 1 0;
%     0 0 0 1];
% Ds = [0 0 0 0]';
% % syss = ss(As,Bs,Cs,Ds);
% eigs(As)
% pzmap(syss)

%% new method

x = [CXu;CXa;CZ0;CXq;CZu;CZa;CX0;CZq;Cmu;Cmq;CZadot;Cmadot;CXde;CZde;Cma;muc;c;V0;KY2;Cmde];
x=x';

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
eigs(As)

%% analytical with simplified solutions

%short period
Ashort= (-2*(c/V0)^3*(CZadot-2*muc)*KY2*muc);
Bshort=-(c/V0)^2*(Cmadot*(CZq+2*muc)-Cmq*(CZadot-2*muc)+2*CZa*KY2*muc);
Cshort=-(c/V0)*(Cma*(CZq+2*muc)-Cmq*CZa);
%eigenV0alues of short period
lambda1=roots([Ashort Bshort Cshort])

%phugoid

%very simplified solution

% Aphu = -4*((c^3)/(V0^4))*muc^2;
% Bphu = 2*muc*CXu*(c^2)/(V0^3);
% Cphu = -CZu*CZ0*(c)/(V0^2);
%lambda2=roots([Aphu Bphu Cphu]);

%less simplified solution

Aphu=2*muc*(CZa*Cmq-2*muc*Cma);
Bphu=2*muc*(CXu*Cma-Cmu*CXa)+Cmq*(CZu*CXa-CXu*CZa);
Cphu=CZ0*(Cmu*CZa-CZu*Cma);


lambda2=roots([Aphu Bphu Cphu]);
lambda2=lambda2*V0/c