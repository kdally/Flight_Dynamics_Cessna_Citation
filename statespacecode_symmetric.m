%symmetric motion 
% load('variables.mat')
C2s= [CXu/V0 CXa CZ0 CXq*c/V0;
    CZu/V0 CZa -CX0 (CZq+2*muc)*c/V0;
    0 0 0 c/V0;
    Cmu/V0 Cma 0 Cmq*c/V0];
C1s= [-2*muc*c/V0^2 0 0 0;
    0 (CZadot-2*muc)*c/V0 0 0;
    0 0 -c/V0 0;
    0 Cmadot*c/V0 0 -2*muc*KY2*(c/V0)^2];
C3s= [CXde;CZde;0;Cmde];
As=-(C1s^-1)*C2s;
Bs=-(C1s^-1)*C3s;
Cs = [0 0 0 0;
    0 1 0 0;
    0 0 0 0;
    0 0 0 0];
Ds = [0 0 0 0]';
syss = ss(As,Bs,Cs,Ds);
ts = 0:0.01:20;
u=ones(size(ts));
ys = lsim(syss,zeros(size(ts)),ts,[0 alphaused(1)*pi/180 2 0]')*180/pi;

index1= find(timetrue==2650);
index2= find(timetrue==2650+20);

 plot(ts,ys(:,2))
 hold on
 plot(timetrue(index1:index2)-timetrue(index1),alphaused)
 hold off