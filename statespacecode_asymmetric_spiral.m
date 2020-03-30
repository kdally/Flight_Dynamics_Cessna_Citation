%Load Cit. par. data
Cit_par_spiral;

%Set parameter range
params1 = -0.125:0.0001:-0.12; %Clb   
params2 = 0.253:0.0001:0.2600; %Clr      
params6 = -0.0102:0.0001:-0.0099 ; %Cnda  
params5 = -0.0684:0.005:-0.068 ; %Cndr  

%Add measurement inputs, inital conditions, time vector
sysr = 0;
differencer = 1*10^1000;
roll = flightdata.Ahrs1_Roll.data(index:index+stop)*pi/180;
t = flightdata.time.data(index:index+stop);
delta_a = pi/180*(-flightdata.delta_a.data(index:index+stop) + flightdata.delta_a.data(index));
delta_r = pi/180*(-flightdata.delta_r.data(index:index+stop) + flightdata.delta_r.data(index));
u = [delta_a.'; delta_r.'];
x0 = [0;flightdata.Ahrs1_Roll.data(index)*pi/180;flightdata.Ahrs1_bRollRate.data(index)*pi/180;flightdata.Ahrs1_bYawRate.data(index)*pi/180];


% 4 nested four loops to optinize four coefficients
warning('off','Control:analysis:LsimStartTime')
for i=1:length(params5)
    Cndr = params5(i);
    for j=1:length(params6)
        Cnda = params6(j)
        for k=1:length(params1)
            Clb = params1(k);
            for w=1:length(params2)
                Clr = params2(w);

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
                Ca = [1 0 0 0;
                    0 1 0 0;
                    0 0 1 0;
                    0 0 0 1];
                Da = [0 0 0 0;
                    0 0 0 0]';
                sysa = ss(Aa,Ba,Ca,Da);
                y = lsim(sysa,u,t,x0);
                
                %Compute squared differences
                differencet = sum(abs(roll-y(:,2)).^2);
                if differencet < differencer
                    sysr = sysa;
                    differencer = differencet;
                    
                    %Record optimized coefficients
                    AClb = Clb;
                    AClr = Clr;
                    ACnda = Cnda;
                    ACndr = Cndr;
                end
            end
        end
    end
end 

%Review changes with roll output
clf
y = lsim(sysr,u,t,x0);
plot(t,y(:,2),'DisplayName','Roll Sim')
hold on
plot(t,roll,'DisplayName','Roll test')
legend();



%%

%Load Cit. par. data
Cit_par

%Compute State Space system
warning('off','Control:analysis:LsimStartTime')
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
Ca = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
Da = [0 0 0 0;
    0 0 0 0]';
sysa = ss(Aa,Ba,Ca,Da);

%Extract inputs, time, initial state from measurement data
t       = flightdata.time.data(index:index+stop);
x0 = [0;flightdata.Ahrs1_Roll.data(index)*pi/180;flightdata.Ahrs1_bRollRate.data(index)*pi/180;flightdata.Ahrs1_bYawRate.data(index)*pi/180];

delta_a = pi/180*(-flightdata.delta_a.data(index:index+stop) + flightdata.delta_a.data(index)); %convert inputs to radians
delta_r = pi/180*(-flightdata.delta_r.data(index:index+stop) + flightdata.delta_r.data(index)); %convert inputs to radians
[t_d_rudder,t_index_rudderstop] = min(abs(t-time_rudderstop)); %Make the rudder input start at zero, for aileron it is already the case
u = [delta_a.'; delta_r.']; 
y = lsim(sysa,u,t,x0);


%% PLOTTING RESPONSES

%Extract measurement data corresponding to state space sytem outputs
pitch =  flightdata.Ahrs1_Pitch.data(index:index+stop)*pi/180;
roll = flightdata.Ahrs1_Roll.data(index:index+stop)*pi/180;
rollrate = flightdata.Ahrs1_bRollRate.data(index:index+stop)*pi/180;
yawrate = flightdata.Ahrs1_bYawRate.data(index:index+stop)*pi/180;

figure
subplot(3,1,1);
plot(t,rad2deg(delta_r),'--b','DisplayName','\delta_r','LineWidth',1.5)
hold on
plot(t,rad2deg(delta_a),'DisplayName','\delta_a','LineWidth',1.5,'Color',[0 1 1])
grid on
ylabel('[deg]');
set(legend,'FontName','Helvetica','Location','Northeast');
legend();
set(findall(gcf,'-property','FontSize'),'FontSize',13)
xlim([timebegin-5 3.5092e+03]); %make the time start at 0 seconds
xticks(linspace(timebegin, 3.5005e+03, 8))
xticklabels(num2cell(linspace(0, 280, 8)))

subplot(3,1,2);
plot(t,rad2deg(roll),'DisplayName','\phi experimental','LineWidth',1.5,'Color','k')
hold on
plot(t,rad2deg(y(:,2)),'--r','DisplayName','\phi numerical','LineWidth',1.5)
ylabel('[deg]');
set(legend,'FontName','Helvetica','Location','Northeast');
grid on
legend();
set(findall(gcf,'-property','FontSize'),'FontSize',13)
xlim([timebegin-5 3.5092e+03]);
xticks(linspace(timebegin, 3.5005e+03, 8))
xticklabels(num2cell(linspace(0, 280, 8)))

subplot(3,1,3);
plot(t,rad2deg(yawrate),'DisplayName','r experimental','LineWidth',1.5,'Color','k')
hold on
plot(t,rad2deg(y(:,4)),'--r','DisplayName','r numerical','LineWidth',1.5)
xlabel('Time [s]');
ylabel('[deg/s]');
set(legend,'FontName','Helvetica','Location','Northeast');
legend();
grid on
set(findall(gcf,'-property','FontSize'),'FontSize',13)
xlim([timebegin-5 3.5092e+03]);
xticks(linspace(timebegin, 3.5005e+03, 8))
xticklabels(num2cell(linspace(0, 280, 8)))





























