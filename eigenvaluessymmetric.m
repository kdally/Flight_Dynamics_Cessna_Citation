%% PREPARE DATA TO FIT PHUGOID
V0     = 103.5474;            
m      =5727.6;
hp0 =1.9132e+03;
Cma    = -0.43;           
Cmde   = -1.553;  
S      = 24.20;	                 
lh     = 5.5;     
c      = 2.022;	  
muc    = 102.7;
KY2    = 0.98;
CX0    = 0;
CXu    = -0.2199;
CXa    =  0.4653;
CXadot = 0.;     
CXq    = 0.;
CXde   = 0;
CZ0    = -1.136; 
CZu    =  -2.272;
CZa    = -5.16;
CZadot = -1.43;
CZq    = -3.86;
CZde   =  -0.6328;
Cmu    = 0.;
Cmadot = -3.7;
Cmq    = -7.04;
%%
S      = 30.00;	          % wing area [m^2]
Sh     = 0.2*S;           % stabiliser area [m^2]
Sh_S   = Sh/S;	          % [ ]
lh     = 0.71*5.968;      % tail length [m]
c      = 2.0569;	  % mean aerodynamic cord [m]
lh_c   = lh/c;	          % [ ]
b      = 15.911;	  % wing span [m]
bh     = 5.791;	          % stabilser span [m]
A      = b^2/S;           % wing aspect ratio [ ]
Ah     = bh^2/Sh;         % stabilser aspect ratio [ ]
Vh_V   = 1;		  % [ ]
ih     = -2*pi/180;       % stabiliser angle of incidence [rad]

% Constant values concerning atmosphere and gravity

rho0   = 1.2250;          % air density at sea level [kg/m^3] 
lambda = -0.0065;         % temperature gradient in ISA [K/m]
Temp0  = 288.15;          % temperature at sea level in ISA [K]
R      = 287.05;          % specific gas constant [m^2/sec^2K]
g      = 9.81;            % [m/sec^2] (gravity constant)

rho    = rho0*((1+(lambda*hp0/Temp0)))^(-((g/(lambda*R))+1));   % [kg/m^3]  (air density)
W      = m*g;				                        % [N]       (aircraft weight)

% Constant values concerning aircraft inertia

muc    = m/(rho*S*c);
mub    = m/(rho*S*b);
KX2    = 0.019;
KZ2    = 0.042;
KXZ    = 0.002;
KY2    = 1.25*1.114;

% Aerodynamic constants

Cmac   = 0;                     % Moment coefficient about the aerodynamic centre [ ]
CNwa   = CLa;   		        % Wing normal force slope [ ]
CNha   = 2*pi*Ah/(Ah+2);        % Stabiliser normal force slope [ ]
depsda = 4/(A+2);               % Downwash gradient [ ]

% Lift and drag coefficient

CL = 2*W/(rho*V0^2*S);               % Lift coefficient [ ]
CD = CD0 + (CLa*alpha0)^2/(pi*A*e);  % Drag coefficient [ ]

% Stabiblity derivatives

CX0    = W*sin(th0)/(0.5*rho*V0^2*S);
CXu    = -0.02792;
CXa    = -0.47966;
CXadot = +0.08330;
CXq    = -0.28170;
CXde   = -0.03728;

CZ0    = -W*cos(th0)/(0.5*rho*V0^2*S);
CZu    = -0.37616;
CZa    = -5.74340;
CZadot = -0.00350;
CZq    = -5.66290;
CZde   = -0.69612;

Cmu    = +0.06990;
Cmadot = +0.17800;
Cmq    = -8.79415;
%%
x = [CXu;CXa;CZ0;CXq;CZu;CZa;CX0;CZq;Cmu;Cmq;CZadot;Cmadot;CXde;CZde;Cma;muc;c;V0;KY2;Cmde];
x=x';
%%
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
%%
pzmap(syss);

%% Initial value
subplot(2,2,1)
title('Initial value $\hat{u}$','Interpreter','latex')
t=0:0.1:15;
y=myspace(x,zeros(size(t)),t,0.1*V0,0,0,0);
hold on
plot(t,y(:,1)/V0,'DisplayName','$\hat{u}$','LineWidth',1.5);
plot(t,y(:,2),'DisplayName','$\alpha$','LineWidth',1.5);
plot(t,y(:,3),'DisplayName','$\theta$','LineWidth',1.5);
plot(t,y(:,4),'DisplayName','q','LineWidth',1.5);
ylabel('[-] / [rad] / [rad/s]')
xlabel('Time [s]')
axis([ t(1) t(end) min([y(:,1)/V0;y(:,2);y(:,3);y(:,4)])-0.01 max([y(:,1)/V0;y(:,2);y(:,3);y(:,4)])+0.01 ]);
legend('Interpreter','latex');
box on
set(findall(gcf,'-property','FontSize'),'FontSize',13)

subplot(2,2,2)
title('Initial value $\alpha$','Interpreter','latex')
t=0:0.1:15;
y=myspace(x,zeros(size(t)),t,0,0.05,0,0);
hold on
plot(t,y(:,1)/V0,'DisplayName','$\hat{u}$','LineWidth',1.5);
plot(t,y(:,2),'DisplayName','$\alpha$','LineWidth',1.5);
plot(t,y(:,3),'DisplayName','$\theta$','LineWidth',1.5);
plot(t,y(:,4),'DisplayName','q','LineWidth',1.5);
ylabel('[-] / [rad] / [rad/s]')
xlabel('Time [s]')
axis([ t(1) t(end) min([y(:,1)/V0;y(:,2);y(:,3);y(:,4)])-0.01 max([y(:,1)/V0;y(:,2);y(:,3);y(:,4)])+0.01 ]);
legend('Interpreter','latex');
box on
set(findall(gcf,'-property','FontSize'),'FontSize',13)

subplot(2,2,3)
title('Initial value $\theta$','Interpreter','latex')
t=0:0.1:150;
y=myspace(x,zeros(size(t)),t,0,0,0.05,0);
hold on
plot(t,y(:,1)/V0,'DisplayName','$\hat{u}$','LineWidth',1.5);
plot(t,y(:,2),'DisplayName','$\alpha$','LineWidth',1.5);
plot(t,y(:,3),'DisplayName','$\theta$','LineWidth',1.5);
plot(t,y(:,4),'DisplayName','q','LineWidth',1.5);
ylabel('[-] / [rad] / [rad/s]')
xlabel('Time [s]')
axis([ t(1) t(end) min([y(:,1)/V0;y(:,2);y(:,3);y(:,4)])-0.01 max([y(:,1)/V0;y(:,2);y(:,3);y(:,4)])+0.01 ]);
legend('Interpreter','latex');
box on
set(findall(gcf,'-property','FontSize'),'FontSize',13)

subplot(2,2,4)
title('Initial value q','Interpreter','latex')
t=0:0.1:15;
y=myspace(x,zeros(size(t)),t,0,0,0,0.05);
hold on
plot(t,y(:,1)/V0,'DisplayName','$\hat{u}$','LineWidth',1.5);
plot(t,y(:,2),'DisplayName','$\alpha$','LineWidth',1.5);
plot(t,y(:,3),'DisplayName','$\theta$','LineWidth',1.5);
plot(t,y(:,4),'DisplayName','q','LineWidth',1.5);
ylabel('[-] / [rad] / [rad/s]')
xlabel('Time [s]')
axis([ t(1) t(end) min([y(:,1)/V0;y(:,2);y(:,3);y(:,4)])-0.01 max([y(:,1)/V0;y(:,2);y(:,3);y(:,4)])+0.01 ]);
legend('Interpreter','latex');
box on
set(findall(gcf,'-property','FontSize'),'FontSize',13)


%% Initial input PROBLEM
dt=0.1;
tstart=5;
t1=0:dt:tstart;
tend=15;
t2=tstart+dt:dt:tend;
tendsimulation=100;
t3= tend+dt:dt:tendsimulation;
t4=0:dt:tendsimulation;
de=-0.01;
u= cat(2,zeros(size(t1)),de*ones(size(t2)),zeros(size(t3)));
%%
t=t4;
y=myspace(x,u,t,0,0,0,0);
subplot(2,1,1)
hold on
plot(t,y(:,1)/V0,'DisplayName','$\hat{u}$','LineWidth',1.5);
plot(t,y(:,2),'DisplayName','$\alpha$','LineWidth',1.5);
plot(t,y(:,3),'DisplayName','$\theta$','LineWidth',1.5);
plot(t,y(:,4),'DisplayName','q','LineWidth',1.5);
ylabel('[-] / [rad] / [rad/s]')
xlabel('Time [s]')
axis([ t(1) t(end) min([y(:,1)/V0;y(:,2);y(:,3);y(:,4)])-0.01 max([y(:,1)/V0;y(:,2);y(:,3);y(:,4)])+0.01 ]);
legend('Interpreter','latex');
box on
set(findall(gcf,'-property','FontSize'),'FontSize',14)

subplot(2,1,2)
plot(t4,u,'b','LineWidth',1.5)
ylabel('rad');
lgd=legend({'$\delta_e$'},'Interpreter','latex');
axis([ t(1) t(end) min([u])-0.005 max([u])+0.005 ]);
box on
set(findall(gcf,'-property','FontSize'),'FontSize',14)
%%
subplot(4,1,1)
plot(t4,yu(:,1), 'r');
grid on
grid minor
ylabel('m/s');
%title(['pertvelocity']);
hold off
lgd=legend({'u'});

subplot(5,1,2)
ylabel('rad');
%title('alpha');
plot(t4,yu(:,2)','r')
grid on
grid minor
lgd=legend({'$\alpha$ '},'Interpreter','latex');
lgd.FontSize=14;

subplot(5,1,3)
plot(t4,yu(:,3)','r')
grid on
grid minor
ylabel('rad');
lgd=legend({'$\theta$ '},'Interpreter','latex');
lgd.FontSize=14;

subplot(5,1,4)
ylabel('rad/s');
plot(t4,yu(:,4)','r')
grid on
grid minor
lgd=legend({'q '},'Interpreter','latex');
lgd.FontSize=14;

subplot(5,1,5)
plot(t4,u,'g')
grid on
grid minor
ylabel('rad');
lgd=legend({'$\delta_e$'},'Interpreter','latex');
lgd.FontSize=14;
