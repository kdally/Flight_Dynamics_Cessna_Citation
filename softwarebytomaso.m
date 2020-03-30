%% PREPARE DATA TO FIT PHUGOID
timetrue=[flightdata.time.data];
windowSize = 20; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;

manouverstart=2651;%3237;2659
simulationdelay=14.2;
manouverlenght=2806.7-manouverstart-simulationdelay;

index1= find(timetrue==manouverstart+simulationdelay);
index2= find(timetrue==manouverstart+manouverlenght);
index3= find(timetrue==manouverstart);

% Stationary flight condition

hp0    = flightdata.Dadc1_alt.data(index1)*0.305;      	  % pressure altitude in the stationary flight condition [m]
V0     = flightdata.Dadc1_tas.data(index1)*0.514444;            % true airspeed in the stationary flight condition [m/sec]

alphatrue=[flightdata.vane_AOA.data]*pi/180;            %*pi/180
alphatrue=filter(b,a,alphatrue);
alpha0 = alphatrue(index1);
alphatrue=alphatrue-alpha0;

velocitytrue=[flightdata.Dadc1_tas.data*0.514444];
velocitytrue=filter(b,a,velocitytrue);

thetatrue=[flightdata.Ahrs1_Pitch.data]*pi/180; %*pi/180
thetatrue=filter(b,a,thetatrue);
th0=thetatrue(index1);
thetatrue=thetatrue-th0;

pitchratetrue=[flightdata.Ahrs1_bPitchRate.data]*pi/180;             %*pi/180
pitchratetrue=filter(b,a,pitchratetrue);
pertvelocitytrue=velocitytrue-V0;

rate0  = pitchratetrue(index1);
pert0  = pertvelocitytrue(index1);

timeused=timetrue(index1:index2)-timetrue(index1);
alphaused=alphatrue(index1:index2);
velocityused=velocitytrue(index1:index2);
thetaused=thetatrue(index1:index2);
elevatordeflection=[flightdata.delta_e.data(index1:index2)]*pi/180;%*pi/180
%elevatordeflection= filter(b,a,elevatordeflection);
elevator0=elevatordeflection(1);
elevatordeflection=elevatordeflection;

pertvelocityused=pertvelocitytrue(index1:index2);
pitchrateused=pitchratetrue(index1:index2);  

alpha00=alpha0;
th00=th0;
alpha0=0;
th0=0;

%% DATA OF THE AIRCRAFT
% Aircraft mass
m      =[weight_calculator(timetrue(index1),2)];         	  % mass [kg]

% % aerodynamic properties
e      = 0.85415;            % Oswald factor [ ]
CD0    = 0.021888;            % Zero lift drag coefficient [ ]
CLa    = 5.084 ;%4.4358;            % Slope of CL-alpha curve [ ]

% Longitudinal stability
Cma    = -0.76;            % longitudinal stabilty [ ]
Cmde   = -1.74;  %-1.1642          % elevator effectiveness [ ]
% Aircraft geometry

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
KY2    = 1.25*1.114;

% Aerodynamic constants

Cmac   = 0;                     % Moment coefficient about the aerodynamic centre [ ]
CNwa   = CLa;   		        % Wing normal force slope [ ]
CNha   = 2*pi*Ah/(Ah+2);        % Stabiliser normal force slope [ ]
depsda = 4/(A+2);               % Downwash gradient [ ]

% Lift and drag coefficient

CL = 2*W/(rho*V0^2*S);               % Lift coefficient [ ]
CD = CD0 + (CLa*alpha00)^2/(pi*A*e);  % Drag coefficient [ ] %QUI *pi/180
% Stabiblity derivatives
CX0    = W*sin(th00)/(0.5*rho*V0^2*S);
CXu    = -0.02;
CXa    = -0.47966;
CXadot = +0.08330;
CXq    = -0.28170;
CXde   = -0.03728;

CZ0    = -W*cos(th00)/(0.5*rho*V0^2*S);
CZu    = -0.0373;
CZa    = -5.74340;
CZadot = -0.00350;
CZq    = -5.66290;
CZde   = -0.69612;

Cmu    = +0.06990;
Cmadot = +0.17800;
Cmq    = -8.79415;
% CX0    = 0;
% CXu    = -2*CD;%-0.02799;
% CXa    = -0.4797; %(-0.4796) 0.4653
% CXadot = 0.08330;      %(0.08330;) 0
% CXq    = -0.2817;
% CXde   = 0;%-0.0373;%-0.0373;
% 
% CZ0    = -CL;%-1.136; % -0.7919
% CZu    = -2*CL;%-0.37616; % -2.272
% CZa    = -5.7434;
% CZadot = 0 ;%-0.003500; %-0.00350 0
% CZq    = 0; %-5.66290 0
% CZde   =  -0.6961; %-0.6328
% 
% Cmu    = 0.0699;
% Cmadot = 0.17800;%-0.7;
% Cmq    = -8.79415; %-7.04;



%% solve model 
x0 = [CXu;CXa;CZ0;CXq;CZu;CZa;CX0;CZq;Cmu;Cmq;CZadot;Cmadot;CXde;CZde;Cma;muc;c;V0;KY2;Cmde];
x0=x0';
%x0=ones(1,20);
%% BOUNDARIES PHUGOID
lbp=      [-0.5,CXa, CZ0, CXq, -2, CZa, CX0, CZq, Cmu, Cmq, CZadot, Cmadot, CXde, CZde, Cma, muc, c, V0, KY2, Cmde ]; %ones(1,20)*300*-1  ; [-500,-500, -500, CXq, -500, -500, CX0, CZq, -500, -500, CZadot, -100, -10, -10, Cma, muc, c, V0, KY2, Cmde]
ubp=      [-0.001,CXa, CZ0, CXq, -0.001, CZa, CX0, CZq, Cmu, Cmq, CZadot, Cmadot, CXde, CZde, Cma, muc, c, V0, KY2, Cmde]; %ones(1,20)*300;       [500,500, 500, CXq, 500, 500, CX0, CZq, 500, 500, CZadot, 100, 10, 10, Cma, muc, c, V0, KY2, Cmde]
%% PHUGOID
options=optimset('LargeScale','off','TolFun',0.1e-12,'MaxIter',1000,'MaxFunEvals',1000000);
xp = lsqcurvefit(@(x,timeused) mymodelp(x,pert0,alpha0,th0,rate0,timeused,elevatordeflection,th00),x0,timeused,cat(2,pertvelocityused/pert0,thetaused/th00,pitchrateused/rate0),lbp,ubp,options);
%%
yp=mymodelp(xp,pert0,alpha0,th0,rate0,timeused,elevatordeflection,th00);
subplot(5,1,1)
plot(timeused,pertvelocityused)
ylabel('u');
%title(['pertvelocity']);
hold on
plot(timeused,yp(:,1)'*pert0)
hold off
lgd=legend({'true','sim'});

subplot(5,1,2)
plot(timeused,alphaused)
ylabel('a');
%title('alpha');
lgd=legend({'true'});

subplot(5,1,3)
plot(timeused,thetaused)
ylabel('theta');
%title('theta');
hold on
plot(timeused,yp(:,2)'*th00)
hold off
lgd=legend({'true','sim'});

subplot(5,1,4)
plot(timeused,pitchrateused)
ylabel('pitch');
%title('pitch rate');
hold on
plot(timeused,yp(:,3)'*rate0)
hold off
lgd=legend({'true','sim'});
subplot(5,1,5)
plot(timeused,elevatordeflection)
ylabel('de');

xp

%% PREPARE DATA TO FIT SHORT PERIOD

timetrue=[flightdata.time.data];
windowSize = 200; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;

manouverstart=2580.1;%3237;2659 2587.1
simulationdelay=0;
manouverlenght=2587-manouverstart-simulationdelay;

index1= find(timetrue==manouverstart+simulationdelay);
index2= find(timetrue==manouverstart+manouverlenght);
index3= find(timetrue==manouverstart);
%index1=index3;
% Stationary flight condition

hp0    = flightdata.Dadc1_alt.data(index1)*0.305;      	  % pressure altitude in the stationary flight condition [m]
V0     = flightdata.Dadc1_tas.data(index1)*0.514444;            % true airspeed in the stationary flight condition [m/sec]
%alpha0 = flightdata.vane_AOA.data(index1);      %*pi/180 	  % angle of attack in the stationary flight condition [rad]
%th0    = flightdata.Ahrs1_Pitch.data(index1);   %*pi/180   	  % pitch angle in the stationary flight condition [rad]


alphatrue=[flightdata.vane_AOA.data]*pi/180;            %*pi/180
%alphatrue=filter(b,a,alphatrue);
alpha0 = alphatrue(index1);
alphatrue=alphatrue-alpha0;

velocitytrue=[flightdata.Dadc1_tas.data*0.514444];
velocitytrue=filter(b,a,velocitytrue);

thetatrue=[flightdata.Ahrs1_Pitch.data]*pi/180; %*pi/180
%thetatrue=filter(b,a,thetatrue);
th0=thetatrue(index1);
thetatrue=thetatrue-th0;

pitchratetrue=[flightdata.Ahrs1_bPitchRate.data]*pi/180;             %*pi/180
%pitchratetrue=filter(b,a,pitchratetrue);
pertvelocitytrue=velocitytrue-V0;

rate0  = pitchratetrue(index1);
pert0  = pertvelocitytrue(index1);

timeused=timetrue(index1:index2)-timetrue(index1);
alphaused=alphatrue(index1:index2);
velocityused=velocitytrue(index1:index2);
thetaused=thetatrue(index1:index2);
elevatordeflection=[flightdata.delta_e.data(index1:index2)]*pi/180;%*pi/180
%elevatordeflection= filter(b,a,elevatordeflection);
elevator0=elevatordeflection(1);
elevatordeflection=elevatordeflection-elevator0;

pertvelocityused=pertvelocitytrue(index1:index2);
pitchrateused=pitchratetrue(index1:index2);  
alpha00=alpha0;
th00=th0;
alpha0=0;
th0=0;
%% SAVING UPDATED X VECTOR
vals = xp;
vars = {'CXu','CXa','CZ0','CXq','CZu','CZa','CX0','CZq','Cmu','Cmq','CZadot','Cmadot','CXde','CZde','Cma','muc','c','V0','KY2','Cmde'};
for i = 1:20
    eval([vars{i} '=  vals(i)'])
end

V0     = flightdata.Dadc1_tas.data(index1)*0.514444; 
m      =weight_calculator(timetrue(index1),2);  
rho    = rho0*((1+(lambda*hp0/Temp0)))^(-((g/(lambda*R))+1)); 
W      = m*g;				                        
muc    = m/(rho*S*c);
%%
x0 = [CXu;CXa;CZ0;CXq;CZu;CZa;CX0;CZq;Cmu;Cmq;CZadot;Cmadot;CXde;CZde;muc;c;V0;KY2;Cmde];
x0=x0';
%% BOUNDARIES SHORT PERIOD
lbsp=      [CXu,CXa, CZ0, CXq, CZu, -Inf, CX0, -Inf, Cmu, -Inf, -Inf, -0.2, CXde, CZde, Cma, muc, c, V0, KY2, Cmde]; 
ubsp=      [CXu,CXa, CZ0, CXq, CZu, -4, CX0, 12, Cmu, -5, 0.5, Inf, CXde, CZde, Cma, muc, c, V0, KY2, Cmde];

%% SHORT PERIOD
%[y,syss]=mymodel(x,alpha0,th0,timeused,elevatordeflection);
options=optimset('LargeScale','off','TolFun',0.1e-12,'MaxIter',1000,'MaxFunEvals',1000000);

xsp = lsqcurvefit(@(x,timeused) mymodelsp(x,pert0,alpha0,th0,rate0,timeused,elevatordeflection,alpha00),x0,timeused,cat(2,alphaused/alpha00,pitchrateused/rate0),lbsp,ubsp,options);
%%
yp=mymodelsp(xsp,pert0,alpha0,th0,rate0,timeused,elevatordeflection,alpha00);
subplot(2,2,1)
plot(timeused,pertvelocityused)
ylabel('u');
%title(['pertvelocity']);
hold off
lgd=legend({'true'});

subplot(2,2,2)
plot(timeused,alphaused)
ylabel('a');
%title('alpha');
hold on
plot(timeused,yp(:,1)'*alpha00)
hold off
lgd=legend({'true','sim'});

subplot(2,2,3)
plot(timeused,thetaused)
ylabel('theta');
%title('theta');
lgd=legend({'true','sim'});

subplot(2,2,4)
plot(timeused,pitchrateused)
ylabel('pitch');
%title('pitch rate');
hold on
plot(timeused,yp(:,2)'*rate0)
hold off
lgd=legend({'true','sim'});
xsp
%%
pzmap(myspace(xsp));