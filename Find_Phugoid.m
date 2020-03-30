load('FTISxprt-20190306_102158.mat')
timebegin = 2600; %3059  + index(110) aperiodic roll || %2883.6 + index(154) dutch roll
[d,index] = min(abs(flightdata.time.data-timebegin));
timeend = 2860;
[d2,index2] = min(abs(flightdata.time.data-timeend));


time = flightdata.time.data(index:index2);
velocity = (flightdata.Dadc1_tas.data(index:index2))*0.5144444;
elevator = flightdata.delta_e.data(index:index2)*6000*pi/180;
hold on
plot(time,elevator);
plot(time,velocity);
