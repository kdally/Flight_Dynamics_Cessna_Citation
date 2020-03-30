clear
timebegin = 2883.6;
Cit_par_fitted_new

warning('off','Control:analysis:LsimStartTime')
t = flightdata.time.data(index:index+156);
rudder = flightdata.delta_r.data(index:index+156)*pi/180;
roll = flightdata.Ahrs1_Roll.data(index:index+156)*pi/180;
rollrate = flightdata.Ahrs1_bRollRate.data(index:index+156)*pi/180;
yawrate = flightdata.Ahrs1_bYawRate.data(index:index+156)*pi/180;
tp = t - t(1);
[pks_yaw,indexes_yaw] = findpeaks(yawrate);%%
x = tp(indexes_yaw(2:end))';%%
y = pks_yaw(2:end);
g = fittype('a-b*exp(-c*x)');
f0 = fit(x,y,g,'StartPoint',[-0.3,-0.5,0]);
xx = linspace(tp(50),tp(end),50);
c = coeffvalues(f0); c = c(3);
T = x(2:end)-x(1:end-1);%%
T = sum(T)/length(T);%%
damping = 1/sqrt((2*pi/((c*T)^2))+1);
wd = 2*pi/T;
dampingsim = (-2*mub*(Cnr+2*KZ2*CYb)/(2*sqrt((8*(mub^2)*KZ2)*(4*mub*Cnb+CYb*Cnr))));
delta = log(pks_yaw(2:end)./pks_yaw(1:end-1));%%
delta = sum(delta)/length(delta);%%
realeig = delta/T;%%
complexeig = 2*pi*realeig/delta;%%
%plot
hold on
plot(x,y,'o',xx,f0(xx),'k--');
% plot(tp(indexes_yaw),pks_yaw)
plot(tp,rudder,'DisplayName','Rudder Input')
%plot(tp,roll,'DisplayName','Roll Flight');
% plot(tp,rollrate,'DisplayName','Roll Rate Flight');
plot(tp,yawrate,'DisplayName','Yaw Rate Flight');
hold off
legend('Peaks','Model Exponential');