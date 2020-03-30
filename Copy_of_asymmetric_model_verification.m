clear
timebegin = 2883.6; %3059 aperiodic roll          %2883.6 dutch roll
Cit_par_original
% Cit_par_fitted_new

clf
warning('off','Control:analysis:LsimStartTime')
%simulation
C1a = [((CYbdot-2*mub)*(b/V0)) 0 0 0;
    0 -(b/(2*V0)) 0 0;
    0 0 (-2*mub*KX2*((b^2)/(V0^2))) (2*mub*KXZ*((b^2)/(V0^2)));
    (Cnbdot*(b/V0)) 0 (2*mub*KXZ*((b^2)/(V0^2))) (-2*mub*KZ2*((b^2)/(V0^2)))];
C2a = [CYb CL (CYp*b/(2*V0)) ((CYr -4*mub)*b/(2*V0));
    0 0 (b/(2*V0)) 0;
    Clb 0 (Clp*b/(2*V0)) (Clr*b/(2*V0));
    Cnb 0 (Cnp*b/(2*V0)) (Cnr*b/(2*V0))];
C3a = [CYda CYdr;
    0 0;
    Clda Cldr;
    Cnda Cndr];
Aa = -(C1a^-1)*C2a;
Ba = -(C1a^-1)*C3a;
Ca = [180/pi 0 0 0;
    0 180/pi 0 0;
    0 0 180/pi 0;
    0 0 0 180/pi];
Da = [0 0 0 0;
    0 0 0 0]';
sys = ss(Aa,Ba,Ca,Da);

aperiodicperiod = 11; %[s]
dutchperiod = 15.6; %[s]
%flight data has 0.1 [s] step in time
t = 0:0.1:aperiodicperiod;
u = zeros(length(t),2);
u(2:11,1) = 0.03;

%plot
% 
subplot(2,2,1)
hold on
title('Aileron input')
plot(t,u(:,1)*180/pi,'b','DisplayName','\delta_a','LineWidth',1.5)
axis([ t(1) t(end) min([u(:,1)])*180/pi-0.1 max([u(:,1)])*180/pi+0.1 ]);
ylabel('[deg]')
legend();

% subplot(2,1,2)
subplot(2,2,3)

x0 = [0.0;0;0;0];
y = lsim(sys,u,t,x0);
hold on
plot(t,y(:,1),'DisplayName','\beta','LineWidth',1.5);
plot(t,y(:,2),'DisplayName','\phi','LineWidth',1.5);
plot(t,y(:,3),'DisplayName','p','LineWidth',1.5);
plot(t,y(:,4),'DisplayName','r','LineWidth',1.5);
ylabel('[deg] / [deg/s]')
xlabel('Time [s]')
axis([ t(1) t(end) min([y(:,1);y(:,2);y(:,3);y(:,4)])-0.01 max([y(:,1);y(:,2);y(:,3);y(:,4)])+0.01 ]);
legend();
box on
set(findall(gcf,'-property','FontSize'),'FontSize',13)


u = zeros(length(t),2);
u(2:11,2) = 0.03;
subplot(2,2,2)
hold on
title('Rudder input')
plot(t,u(:,2)*180/pi,'b','DisplayName','\delta_r','LineWidth',1.5)
axis([ t(1) t(end) min([u(:,2)])*180/pi-0.1 max([u(:,2)])*180/pi+0.1 ]);
ylabel('[deg]')
legend();

% subplot(2,1,2)
subplot(2,2,4)

x0 = [0.0;0;0;0];
y = lsim(sys,u,t,x0);
hold on
plot(t,y(:,1),'DisplayName','\beta','LineWidth',1.5);
plot(t,y(:,2),'DisplayName','\phi','LineWidth',1.5);
plot(t,y(:,3),'DisplayName','p','LineWidth',1.5);
plot(t,y(:,4),'DisplayName','r','LineWidth',1.5);
ylabel('[deg] / [deg/s]')
xlabel('Time [s]')
axis([ t(1) t(end) min([y(:,1);y(:,2);y(:,3);y(:,4)])-0.01 max([y(:,1);y(:,2);y(:,3);y(:,4)])+0.01 ]);
legend();
box on
set(findall(gcf,'-property','FontSize'),'FontSize',13)





% subplot(2,2,2)
% title('Initial value \phi')
% x0 = [0;0.001;0;0];
% y = lsim(sys,u,t,x0);
% hold on
% plot(t,y(:,1),'DisplayName','\beta','LineWidth',1.5);
% plot(t,y(:,2),'DisplayName','\phi','LineWidth',1.5);
% plot(t,y(:,3),'DisplayName','p','LineWidth',1.5);
% plot(t,y(:,4),'DisplayName','r','LineWidth',1.5);
% ylabel('[deg] / [deg/s]')
% xlabel('Time [s]')
% axis([ t(1) t(end) min([y(:,1);y(:,2);y(:,3);y(:,4)])-0.01 max([y(:,1);y(:,2);y(:,3);y(:,4)])+0.01 ]);
% legend();
% box on
% set(findall(gcf,'-property','FontSize'),'FontSize',13)
% 
% subplot(2,2,3)
% title('Initial value p')
% x0 = [0;0;0.05;0];
% y = lsim(sys,u,t,x0);
% hold on
% plot(t,y(:,1),'DisplayName','\beta','LineWidth',1.5);
% plot(t,y(:,2),'DisplayName','\phi','LineWidth',1.5);
% plot(t,y(:,3),'DisplayName','p','LineWidth',1.5);
% plot(t,y(:,4),'DisplayName','r','LineWidth',1.5);
% ylabel('[deg] / [deg/s]')
% xlabel('Time [s]')
% axis([ t(1) t(end) min([y(:,1);y(:,2);y(:,3);y(:,4)])-0.01 max([y(:,1);y(:,2);y(:,3);y(:,4)])+0.01 ]);
% legend();
% box on
% set(findall(gcf,'-property','FontSize'),'FontSize',13)
% 
% subplot(2,2,4)
% title('Initial value r')
% x0 = [0;0;0;0.05];
% y = lsim(sys,u,t,x0);
% hold on
% plot(t,y(:,1),'DisplayName','\beta','LineWidth',1.5);
% plot(t,y(:,2),'DisplayName','\phi','LineWidth',1.5);
% plot(t,y(:,3),'DisplayName','p','LineWidth',1.5);
% plot(t,y(:,4),'DisplayName','r','LineWidth',1.5);
% ylabel('[deg] / [deg/s]')
% xlabel('Time [s]')
% axis([ t(1) t(end) min([y(:,1);y(:,2);y(:,3);y(:,4)])-0.01 max([y(:,1);y(:,2);y(:,3);y(:,4)])+0.01 ]);
% legend();
% box on
% set(findall(gcf,'-property','FontSize'),'FontSize',13)
