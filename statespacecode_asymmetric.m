load('FTISxprt-20190306_102158.mat')
%% Aperiodic Roll
sysr = 0;
cldar = 0;
differencer = 1*10^1000;
params1 = -0.15:0.0001:-0.13; %Clda
params2 = -1:0.01:-0.5; %Clp
warning('off','Control:analysis:LsimStartTime')
for i=1:length(params1)
    Clda = params1(i);
    for j=1:length(params2)
        Clp = params2(j);
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
        Ca = [0 0 0 0;
            0 1 0 0;
            0 0 1 0;
            0 0 0 0];
        Da = [0 0 0 0;
            0 0 0 0]';
        sysa = ss(Aa,Ba,Ca,Da);
        t = flightdata.time.data(index:index+110);
        u = -flightdata.delta_a.data(index:index+110)*pi/180;
        u(:,2) = 0;
        x0 = [0;flightdata.Ahrs1_Roll.data(index)*pi/180;0;0];
        y = lsim(sysa,u,t,x0);
        roll = flightdata.Ahrs1_Roll.data(index:index+110)*pi/180;
        differencet = sum(abs(roll-y(:,2)).^2);
        if differencet < differencer
            Cldar = Clda;
            Clpr = Clp;
            sysr = sysa;
            differencer = differencet;
        end
    end
end %for Clda calculation
y = lsim(sysr,u,t,x0);
roll = flightdata.Ahrs1_Roll.data(index:index+110)*pi/180;
rollrate = flightdata.Ahrs1_bRollRate.data(index:index+110)*pi/180;
hold on
plot(t,y)
plot(t,roll)
plot(t,rollrate)