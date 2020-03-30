load('FTISxprt-20190306_102158.mat')
timebegin = 2580; %3059  + 2584
[d,index] = min(abs(flightdata.time.data-timebegin));
timeend = 2587;
[d2,index2] = min(abs(flightdata.time.data-timeend));


time = flightdata.time.data(index:index2);
alpha = (flightdata.vane_AOA.data(index:index2));
elevator = flightdata.delta_e.data(index:index2);
theta=[flightdata.Ahrs1_Pitch.data(index:index2)];
pitchrate=[flightdata.Ahrs1_bPitchRate.data(index:index2)];
hold on
plot(time,elevator,'DisplayName','Elevator');
plot(time,alpha,'DisplayName','alpha');
%plot(time,pitchrate,'Displayname','thetadot');
%plot(time,theta,'Displayname','theta');
legend();
%%
plot(timeused,pitchrateused)
hold on 
plot(timeused,elevatordeflection)
plot(timeused,alphaused)