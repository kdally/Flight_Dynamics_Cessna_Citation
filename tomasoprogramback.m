%% PREPARE DATA TO FIT PHUGOID
windowSize = 2; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;

manouverstart=2647.3;%2634.2;
simulationstart=2648.3;
manouverlenght=2822.8-manouverstart;

timetrue=[flightdata.time.data];

index1= find(timetrue==manouverstart);
index2= find(timetrue==manouverstart+manouverlenght);
index3= find(timetrue==simulationstart);
index3=index1;

hp0    = flightdata.Dadc1_alt.data(index1)*0.305;      	  % pressure altitude in the stationary flight condition [m]
V0     = flightdata.Dadc1_tas.data(index1)*0.514444;            % true airspeed in the stationary flight condition [m/sec]


timeused=timetrue(index3:index2)-timetrue(index3);

alphatrue=[flightdata.vane_AOA.data]*pi/180;            %*pi/180
alphatrue=filter(b,a,alphatrue);
alpha0 = alphatrue(index1);
alphatrue=alphatrue-alpha0;
alphaused=alphatrue(index3:index2);

velocitytrue=[flightdata.Dadc1_tas.data*0.514444];
velocitytrue=filter(b,a,velocitytrue);
velocityused=velocitytrue(index3:index2);

thetatrue=[flightdata.Ahrs1_Pitch.data]*pi/180; %*pi/180
thetatrue=filter(b,a,thetatrue);
th0=thetatrue(index1);
thetatrue=thetatrue-th0;
thetaused=thetatrue(index3:index2);

pitchratetrue=[flightdata.Ahrs1_bPitchRate.data]*pi/180;             %*pi/180
pitchratetrue=filter(b,a,pitchratetrue);
pitchrateused=pitchratetrue(index3:index2);
rate0  = pitchratetrue(index1);

pertvelocitytrue=velocitytrue-V0;
pertvelocityused=pertvelocitytrue(index3:index2);
pert0  = pertvelocitytrue(index1);

elevatordeflectiontrue=[flightdata.delta_e.data]*pi/180;%*pi/180
elevatordeflectionused=elevatordeflectiontrue(index3:index2);
elevatordeflectionused= filter(b,a,elevatordeflectionused);
elevator0=elevatordeflectiontrue(index1);
elevatordeflectionused=elevatordeflectionused-elevator0;

alpha00=alpha0;
th00=th0;
alpha0=0;
th0=0;

%% DATA OF THE AIRCRAFT
% Aircraft mass
m      =weight_calculator(manouverstart,2);         	  % mass [kg]

% % aerodynamic properties
e      = 0.85415;            % Oswald factor [ ]
CD0    = 0.021888;            % Zero lift drag coefficient [ ]
CLa    = 4.4358 ;%4.4358;            % Slope of CL-alpha curve [ ]


% Aircraft geometry

S      = 30.00;	          % wing area [m^2
rho0   = 1.2250;          % air density at sea level [kg/m^3] 
lambda = -0.0065;         % temperature gradient in ISA [K/m]
Temp0  = 288.15;          % temperature at sea level in ISA [K]
R      = 287.05;          % specific gas constant [m^2/sec^2K]
g      = 9.81;  % [m/sec^2] (gravity constant)
c      = 2.0569;	
rho    = rho0*((1+(lambda*hp0/Temp0)))^(-((g/(lambda*R))+1));   % [kg/m^3]  (air density)
W      = m*g;				                        % [N]       (aircraft weight)
b      = 15.911;	  % wing span [m]
A      = b^2/S;
CL = 2*W/(rho*V0^2*S);               % Lift coefficient [ ]
CD = CD0 + (CLa*alpha00)^2/(pi*A*e);  % Drag coefficient [ ] %QUI *pi/180
% Constant values concerning aircraft inertia
%%
muc    = m/(rho*S*c);
KY2    = 1.25*1.114;

Cma    = -0.7557;%-1.2;            % longitudinal stabilty [ ] -0.76
Cmde   = -1.6495;  %-1.1642          % elevator effectiveness [ ]

CX0    = W*sin(th00)/(0.5*rho*V0^2*S);
CXu    = -0.1;%-2*CD; %-0.02;
CXa    = -0.47966;
CXadot = +0.08330;
CXq    = -0.28170;
CXde   = -0.03728;

CZ0    = -W*cos(th00)/(0.5*rho*V0^2*S);
CZu    =-0.716;%-2*CL;%-0.8;%-0.0373;
CZa    = -5.74340;
CZadot =  -0.00350; %-
CZq    = 0 ;%-5.66290;%-5.66290;
CZde   = -0.69612;

Cmu    = +0.06990;
Cmadot = +0.17800;%+0.17800;
Cmq    = -8.79415;%-18



%% solve model 
x0 = [CXu;CXa;CZ0;CXq;CZu;CZa;CX0;CZq;Cmu;Cmq;CZadot;Cmadot;CXde;CZde;Cma;muc;c;V0;KY2;Cmde];
x0=x0';
yp=myspace(x0,elevatordeflectionused,timeused,pert0,alpha0,th0,rate0);
%% BOUNDARIES PHUGOID verification run
lbp=     [CXu,CXa, CZ0, CXq, CZu, CZa, CX0, CZq, Cmu, Cmq, CZadot, Cmadot, CXde, CZde, Cma, muc, c, V0, KY2, Cmde ]; 
ubp=   [CXu,CXa, CZ0, CXq, CZu, CZa, CX0, CZq, Cmu, Cmq, CZadot, Cmadot, CXde, CZde, Cma, muc, c, V0, KY2, Cmde];
%% BOUNDARIES PHUGOID improved
lbp=     [-1,CXa, CZ0, CXq, -1.5, CZa, CX0, CZq, Cmu, Cmq, CZadot, Cmadot, CXde-1, CZde-4, Cma, muc, c, V0, KY2, Cmde ];%[-1,-1, CZ0, CXq, -2, CZa, CX0, CZq, -1, -30, CZadot, Cmadot, CXde, CZde, Cma, muc, c, V0, KY2, Cmde] ; %ones(1,20)*300*-1  ; [-500,-500, -500, CXq, -500, -500, CX0, CZq, -500, -500, CZadot, -100, -10, -10, Cma, muc, c, V0, KY2, Cmde]
ubp=   [-0.001,CXa, CZ0, CXq, -0.001, CZa, CX0, CZq, Cmu, Cmq, CZadot, Cmadot, CXde+1, CZde+1, Cma, muc, c, V0, KY2, Cmde];% [-0.01,-0.01, CZ0, CXq, -0.01, CZa, CX0, CZq, 1, -0.01, CZadot, Cmadot, CXde, CZde, Cma, muc, c, V0, KY2, Cmde]   [CXu,CXa, CZ0, CXq, CZu, CZa, CX0, CZq, Cmu, Cmq, CZadot, Cmadot, CXde, CZde, 0, muc, c, V0, KY2, 0]; %ones(1,20)*300;       [500,500, 500, CXq, 500, 500, CX0, CZq, 500, 500, CZadot, 100, 10, 10, Cma, muc, c, V0, KY2, Cmde]
%% PHUGOID
options=optimset('LargeScale','off','TolFun',0.1e-12,'MaxIter',1000,'MaxFunEvals',1000000);
xp = lsqcurvefit(@(x,timeused) mymodelp(x,pert0,alpha0,th0,rate0,timeused,elevatordeflectionused,th00),x0,timeused,cat(2,pertvelocityused/pert0,thetaused/th00,pitchrateused/rate0),lbp,ubp,options);

%%
yp=mymodelp(xp,pert0,alpha0,th0,rate0,timeused,elevatordeflectionused,th00);

subplot(4,1,1)
plot(timeused,pertvelocityused,'k','LineWidth',1.5)
grid on
grid minor
ylabel('m/s');
%title(['pertvelocity']);
hold on
plot(timeused,yp(:,1)'*pert0,'--r','LineWidth',1.5)
hold off
lgd=legend({'u experimental','u simulation'},'Interpreter','latex');
lgd.FontSize=14;

subplot(4,1,2)
plot(timeused,thetaused,'k','LineWidth',1.5)
grid on
grid minor
ylabel('rad');
%title('theta');
hold on
plot(timeused,yp(:,2)'*th00,'--r','LineWidth',1.5)

hold off
lgd=legend({'$\theta$ experimental','$\theta$ simulation'},'Interpreter','latex');
lgd.FontSize=14;

subplot(4,1,3)
plot(timeused,pitchrateused,'k','LineWidth',1.5)
grid on
grid minor
ylabel('rad/s');
%title('pitch rate');
hold on
plot(timeused,yp(:,3)'*rate0,'--r','LineWidth',1.5)
hold off
lgd=legend({'q experimental','q simulation'},'Interpreter','latex');
lgd.FontSize=14;

subplot(4,1,4)
plot(timeused,elevatordeflectionused,'Color',[0 1 1],'LineWidth',1.5)
grid on
grid minor
ylabel('rad');
lgd=legend({'$\delta_e$'},'Interpreter','latex');
lgd.FontSize=14;
xp
%% Match natural frequecy and damping ratio
peaks=[9.3143 5.9679 3.5023];
timepeaks=[44 91.5 138.2];
periods=diff(timepeaks);
period=mean(periods);
delta = log(peaks(2:end)./peaks(1:end-1));
delta = sum(delta)/length(delta);
realeig = delta/period%%
complexeig = 2*pi*realeig/delta
%%
period=46;
CZu=(2*pi/period*c/V0)^2*4*muc^2/CZ0; 
syss=myspace2(xp);
[Wnm,zetam,Pm] = damp(syss);
%% PREPARE DATA TO FIT PHUGOID
windowSize = 2; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;

manouverstart=2580.5;%2634.2;
simulationstart=2580.5;
manouverlenght=2587-manouverstart;

timetrue=[flightdata.time.data];

index1= find(timetrue==manouverstart);
index2= find(timetrue==manouverstart+manouverlenght);
index3= find(timetrue==simulationstart);
index3=index1;

hp0    = flightdata.Dadc1_alt.data(index1)*0.305;      	  % pressure altitude in the stationary flight condition [m]
V0     = flightdata.Dadc1_tas.data(index1)*0.514444;            % true airspeed in the stationary flight condition [m/sec]


timeused=timetrue(index3:index2)-timetrue(index3);

alphatrue=[flightdata.vane_AOA.data]*pi/180;            %*pi/180
%alphatrue=filter(b,a,alphatrue);
alpha0 = alphatrue(index1);
alphatrue=alphatrue-alpha0;
alphaused=alphatrue(index3:index2);

velocitytrue=[flightdata.Dadc1_tas.data*0.514444];
velocitytrue=filter(b,a,velocitytrue);
velocityused=velocitytrue(index3:index2);

thetatrue=[flightdata.Ahrs1_Pitch.data]*pi/180; %*pi/180
thetatrue=filter(b,a,thetatrue);
th0=thetatrue(index1);
thetatrue=thetatrue-th0;
thetaused=thetatrue(index3:index2);

pitchratetrue=[flightdata.Ahrs1_bPitchRate.data]*pi/180;             %*pi/180
pitchratetrue=filter(b,a,pitchratetrue);
pitchrateused=pitchratetrue(index3:index2);
rate0  = pitchratetrue(index1);

pertvelocitytrue=velocitytrue-V0;
pertvelocityused=pertvelocitytrue(index3:index2);
pert0  = pertvelocitytrue(index1);

elevatordeflectiontrue=[flightdata.delta_e.data]*pi/180;%*pi/180
elevatordeflectionused=elevatordeflectiontrue(index3:index2);
%elevatordeflectionused= filter(b,a,elevatordeflectionused);
elevator0=elevatordeflectiontrue(index1);
elevatordeflectionused=elevatordeflectionused-elevator0;

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
%%
V0     = flightdata.Dadc1_tas.data(index1)*0.514444; 
m      =weight_calculator(timetrue(index1),2);  
rho    = rho0*((1+(lambda*hp0/Temp0)))^(-((g/(lambda*R))+1)); 
W      = m*g;				                        
muc    = m/(rho*S*c);
CZ0    = -W*cos(th00)/(0.5*rho*V0^2*S);
CX0    = W*sin(th00)/(0.5*rho*V0^2*S);
CZq =-5.6629;
Cmadot=-3.7;
CZadot=-3.00;
CXde=0.;
Cmq=-9.56;
x0 = [CXu;CXa;CZ0;CXq;CZu;CZa;CX0;CZq;Cmu;Cmq;CZadot;Cmadot;CXde;CZde;Cma;muc;c;V0;KY2;Cmde];
x0=x0';
%% BOUNDARIES SHORT PERIOD verification run
lbsp=      [CXu,CXa, CZ0, CXq, CZu, CZa, CX0,CZq, Cmu, Cmq, CZadot, Cmadot, CXde, CZde, Cma, muc, c, V0, KY2, Cmde]; 
ubsp=      [CXu,CXa, CZ0, CXq, CZu, CZa, CX0, CZq, Cmu, Cmq, CZadot, Cmadot, CXde, CZde, Cma, muc, c, V0, KY2, Cmde];
%% BOUNDARIES SHORT PERIOD improved
lbsp=      [CXu,CXa, CZ0, CXq, CZu, CZa-10, CX0,CZq, Cmu, Cmq-50, CZadot, Cmadot-50, CXde-5, CZde-5, Cma-1, muc, c, V0, KY2, Cmde-1]; 
ubsp=      [CXu,CXa, CZ0, CXq, CZu, CZa+10, CX0, CZq, Cmu, Cmq, CZadot, Cmadot+50, CXde+5, CZde+5, Cma+0.34, muc, c, V0, KY2, Cmde+1];

%% SHORT PERIOD
%[y,syss]=mymodel(x,alpha0,th0,timeused,elevatordeflection);
options=optimset('LargeScale','off','TolFun',0.1e-12,'MaxIter',1000,'MaxFunEvals',1000000);
xsp = lsqcurvefit(@(x,timeused) mymodelsp(x,pert0,alpha0,th0,rate0,timeused,elevatordeflectionused,alpha00),x0,timeused,cat(2,alphaused,thetaused,pitchrateused),lbsp,ubsp,options);
%%
yp=mymodelsp(xsp,pert0,alpha0,th0,rate0,timeused,elevatordeflectionused,alpha00);
subplot(4,1,1)
plot(timeused,alphaused,'k','LineWidth',1.5)
grid on
grid minor
ylabel('rad');
%title('alpha');
hold on
plot(timeused,yp(:,1)','--r','LineWidth',1.5)
hold off
lgd=legend({'$\alpha$ experimental','$\alpha$ simulation'},'Interpreter','latex');
lgd.FontSize=14;
subplot(4,1,2)
hold on
plot(timeused,thetaused,'k','LineWidth',1.5)
plot(timeused,yp(:,2)','--r','LineWidth',1.5)
hold off
grid on
grid minor
ylabel('rad');
lgd=legend({'$\theta$ experimental','$\theta$ simulation'},'Interpreter','latex');
lgd.FontSize=14;

subplot(4,1,3)
plot(timeused,pitchrateused,'k','LineWidth',1.5)
ylabel('rad/s');
%title('pitch rate');
hold on
plot(timeused,yp(:,3)','--r','LineWidth',1.5)
grid on
grid minor
hold off
lgd=legend({'q experimental','q simulation'},'Interpreter','latex');
lgd.FontSize=14;

subplot(4,1,4)
plot(timeused,elevatordeflectionused,'Color',[0 1 1],'LineWidth',1.5)
grid on
grid minor
ylabel('rad');
lgd=legend({'$\delta_e$'},'Interpreter','latex');
lgd.FontSize=14;
labelprinter=vars;
labelprinter(2,:) = num2cell(xsp);
labelprinter(3,:) = num2cell(x0-xsp);
fprintf('%10s %f %f\n',labelprinter{:})
%% find eigs short period
syss=myspace2(xsp);
[Wnm,zetam,Psp] = damp(syss);
timeselevator=0.55;
timeend=4.0;
index4= find(timeused==timeselevator);
index5= find(timeused==timeend);
initialpeak=pitchrateused(index4);
endvalue=pitchrateused(index5);
half1=endvalue-(endvalue-initialpeak)/2;
%th1=timeused(pitchrateused==half1)
hp1=1.28-timeselevator;
secondpeak=max(pitchrateused);
timesecondpeak=timeused(pitchrateused==secondpeak);
half2=(secondpeak-endvalue)/2+endvalue;
%th2=timeused(pitchrateused==half2)
hp2=2.54-timesecondpeak

%%

hp=(hp1+hp2)/2;
realeig = -log(2)/hp%%
period=timeend-timeselevator;
complex=2*pi/period
Psp
%complexeig = 2*pi*realeig/delta