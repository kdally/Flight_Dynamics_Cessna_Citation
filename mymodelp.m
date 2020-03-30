
%% PREPARE NUMERICAL MODEL
function y = mymodelp(x,pert0,alpha0,theta0,rate0,timeused,elevatordeflection,theta00)

C2s= [x(1)/x(18) x(2) x(3) x(4)*x(17)/x(18);
    x(5)/x(18) x(6) -x(7) (x(8)+2*x(16))*x(17)/x(18);
    0. 0. 0. x(17)/x(18);
    x(9)/x(18) x(15) 0. x(10)*x(17)/x(18)];
C1s= [-2*x(16)*x(17)/x(18)^2 0 0 0;
    0 (x(11)-2*x(16))*x(17)/x(18) 0 0;
    0 0 -x(17)/x(18) 0;
    0 x(12)*x(17)/x(18) 0 -2*x(16)*x(19)*(x(17)/x(18))^2];
C3s= [x(13);x(14);0;x(20)];
As=-(C1s^-1*C2s);
Bs=-(C1s^-1*C3s);
Cs = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
Ds = [0 0 0 0]';
syss = ss(As,Bs,Cs,Ds);
ts = timeused; %0:0.1:20;
u=elevatordeflection;
z = lsim(syss,u,ts,[pert0 alpha0 theta0 rate0]');
y=cat(2,z(:,1)/pert0,z(:,3)/theta00,z(:,4)/rate0);
%y = z;
%eigs(As)
%y = z(:,2:3);

end
% 
% function y = mymodel(x,Cma,muc,c,V0,KY2,Cmde,alpha0,theta0,timeused)
% C1s= [x(1)/V0 x(2) x(3) x(4)*c/V0;
%     x(5)/V0 x(6) -x(7) (x(8)+2*muc)*c/V0;
%     0. 0. 0. c/V0;
%     x(9)/V0 Cma 0. x(10)*c/V0];
% C2s= [-2*muc*c/V0^2 0 0 0;
%     0 (x(11)-2*muc)*c/V0 0 0;
%     0 0 -c/V0 0;
%     0 x(12)*c/V0 0 -2*muc*KY2*(c/V0)^2];
% C3s= [x(13);x(14);0;Cmde];
% As=-C1s^-1*C2s;
% Bs=-C1s^-1*C3s;
% Cs = [1 0 0 0;
%     0 1 0 0;
%     0 0 1 0;
%     0 0 0 1];
% Ds = [0 0 0 0]';
% syss = ss(As,Bs,Cs,Ds);
% ts = timeused-timeused(1); %0:0.1:20;
% u=ones(size(ts));
% z = lsim(syss,u,ts,[0 alpha0 theta0 0]')*180/pi;
% y = z(:,1)';
% end
