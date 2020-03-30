%% CHOOSE TO LOAD REFERENCE DATA OR TEST FLIGHT 

data = 2; %1 for reference data, 2 for test flight data

if data==1
    load('reference.mat', 'flightdata')
    t_trim = [1157, 1297, 1426, 1564, 1787, 1920];
    Thrust = [3647.09 + 3764.05, 2968.01+3046.75, 2386.3+2519.72, 1850.13+2003.73, 1880.04+2065.54, 2200.43+2386.52];
    
elseif data==2
    load('FTISxprt-20190306_102158.mat', 'flightdata')
    t_trim = [875, 1033, 1124, 1243, 1365, 1540];
    Thrust = [3390.37+3787.53,2666.88+3098.62,2060.84+2379.75,1771.65+2077.73,1776+2042.91,1693.52+1920.13];
end

for i=1:length(t_trim)
    index(i) = find(flightdata.time.data==t_trim(i));  %Find indices corresponding to measurement times
end


%% EXTRACT MEASUREMENT VALUES

% general constants
g =        9.80665; %m/s2
lambda =  -0.0065; %K\m
gamma=     1.4; %gas constant
p_0 =      101325; %Pa
Temp0 =    288.15; %K
rho0 =     1.225; %kg/m3
R =        287;

%Cessna citation parameters
S =     30.00; %m2
m =     6033; %kg
b =     15.911; %m
AR =    (b^2/S);
W_s =   60500;

V_cas =             flightdata.Dadc1_cas.data(index)*0.514444;
alpha =             flightdata.vane_AOA.data(index);
pressure_altitude = flightdata.Dadc1_alt.data(index)*0.3048;

for i = 1:size(alpha)
    p(i) =       p_0*(1+lambda*pressure_altitude(i)/Temp0)^(-(g/(lambda*R)));
    M(i) =       sqrt(2/(gamma-1)*((1+p_0/p(i)*((1+(gamma-1)/(2*gamma)* rho0/p_0*V_cas(i)^2)^(gamma/(gamma-1))-1))^((gamma-1)/gamma)-1));
    Temp(i) =    Temp0/(1+(gamma-1)/2*M(i)^2);
    a_sound(i) = sqrt(gamma*R*Temp(i));
    V_tas(i) =   M(i)*a_sound(i); 
    rho(i) =     p(i)/R/Temp(i); 
    V_eas(i) =   V_tas(i)*sqrt(rho(i)/rho0);
    mass(i) =    weight_calculator(t_trim(i),data);
    Ve_red(i) =  V_eas(i)*sqrt(W_s/(mass(i)*g));  %Equivalent Reduced Airspeed
end


%% COMPUTE LIFT CURVE

for i = 1:size(alpha)
    CL(i) = (W_s)/(0.5*rho0*Ve_red(i)^2*S);  %find CL values based on measurement values
end 

%LINEAR REFRESSION TO FIND CLa
regress = polyfit(alpha,CL.',1);  
CLa = regress(1);
alpha0 = -regress(2)/CLa;
CLa_ref = 5.084*pi/180; %Reference CLa value

for i = 1:size(alpha)
    CL_regress(i) = CLa*(alpha(i)-alpha0); %find CL values based on regress. parameters
end

%PLOTTING CL-alpha
clf
grid on
scatter(alpha,CL);
hold on
plot(alpha,CL_regress);
xlabel('Angle of attack [deg]');
ylabel('Lift coefficient [-]');
lgd = legend({'Flight data','Regression'},'Location','Southeast');
lgd.FontSize = 14;
set(lgd,'FontName','Helvetica');
r_square = sprintf('%.2f',mdl.Rsquared.Ordinary);
pvalue = sprintf('%.2e',mdl.Coefficients.pValue(2));
mdl = fitlm(alpha.',CL,'linear');
str = {'Calculated C_{L\alpha} = '+string(CLa*180/pi)'+' rad^{-1}, R^2 = '+r_square+', p-value = '+pvalue};
box = annotation('textbox',[0.148 0.6 0.3 0.3],'String',str,'FitBoxToText','on');
box.FontSize = 14;
set(box,'FontName','Helvetica');


%% COMPUTE DRAG POLAR

CD0_ref = 0.04;  %Reference CDO value
e_ref = 0.8;     %Reference Oswald efficiency factor value

for i = 1:size(CL,2)
    CD_ref(i) = CD0_ref + CL(i)^2/(pi*e_ref*AR); %find CL values based on reference parameters
    CD(i) = Thrust(i)/(0.5*rho0*V_eas(i)^2*S);   %find CL values based on measurement values
    CL_square(i) = CL(i)^2;
    
end

%LINEAR REFRESSION TO FIND CD0, e
regress = polyfit(CL_square,CD,1);
CD0 = regress(2);
e = 1/(regress(1)*AR*pi);
CD_regress = CD0 + CL_square/(pi*AR*e);

%PLOTTING  DRAG POLAR
clf
hold on
plot(CD_ref,CL)
scatter(CD,CL)
plot(CD_regress,CL)
xlabel('Drag Coefficient [-]');
ylabel('Lift coefficient [-]');
lgd = legend({'with given CD0, e','Regressed values with CD0 = '+string(CD0)+' , e = '+string(e)},'Location','Northwest');
lgd.FontSize = 14;
lgd = legend({'Theoretical data','Flight data','Regression'},'Location','Southeast');
lgd.FontSize = 14;
grid on
set(legend,'FontName','Helvetica');


%% PLOTTING CL^2 - CD

clf
hold on
scatter(CL_square,CD)
plot(CL_square,CD_regress)
xlabel('Lift Coefficient^2 [-]');
ylabel('Drag coefficient [-]');
lgd = legend({'with given CD0, e','Regressed values with CD0 = '+string(CD0)+' , e = '+string(e)},'Location','Northwest');
lgd.FontSize = 14;
lgd = legend({'Flight data','Regression'},'Location','Southeast');
lgd.FontSize = 14;
grid on
set(legend,'FontName','Helvetica');
mdll = fitlm(CL_square,CD,'linear');
r_square = sprintf('%.2f',mdll.Rsquared.Ordinary);
pvalue = sprintf('%.2e',mdll.Coefficients.pValue(2));
str = {'Calculated C_{D0} = '+string(CD0)+' , e = '+string(e)+', R^2 = '+r_square+', p-value = '+pvalue};
box = annotation('textbox',[0.148 0.6 0.3 0.3],'String',str,'FitBoxToText','on');
box.FontSize = 14;
set(box,'FontName','Helvetica');

%% OUTPUT CALCULATED COEFFICIENTS

disp('CLalpha calculated = '+string(CLa*180/pi)+', pre-calculated = '+string(CLa_ref*180/pi)+' 1/rad')
disp('Alpha0 calculated = '+string((alpha0))+' deg')
disp('CD0 calculated = '+string(CD0)+', pre-calculated = '+string(CD0_ref))
disp('e calculated = '+string(e)+', pre-calculated = '+string(e_ref))



%%
A(:,1) = flightdata.Dadc1_alt.data(index);
A(:,2) = flightdata.Dadc1_cas.data(index);
A(:,3) = flightdata.vane_AOA.data(index);
A(:,4) = flightdata.lh_engine_FMF.data(index);
A(:,5) = flightdata.rh_engine_FMF.data(index);
A(:,6) = flightdata.lh_engine_FU.data(index) + flightdata.rh_engine_FU.data(index);
A(:,7) = flightdata.Dadc1_tat.data(index);
A(:,8) = flightdata.delta_e.data(index);
A(:,9) = ones(size(index,2),1)*4.4;
A(:,10) = flightdata.column_fe.data(index);
