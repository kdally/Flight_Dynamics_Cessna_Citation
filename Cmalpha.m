load('variables.mat') % load the variables needed

data = 2;% determine which set of files to use

if data==1
    load('reference.mat', 'flightdata')
    t_trim = [2239
    2351
    2484
    2576
    2741
    2840
    2920];

    t_cg_shift=[3062
    3166];

    thrust_le=[1909.27
    1950.31
    1986.93
    2020.02
    1886.19
    1873.62
    1846.95];

    thrust_re=[2077.30
    2121.42
    2158.92
    2198.14
    2050.73
    2040.42
    2006.02];

    thrust_le_red=[1329.78
    1390.87
    1444.36
    1503.71
    1294.79
    1245.19
    1160.12];

    thrust_re_red=[1329.78
    1390.87
    1444.36
    1503.71
    1294.79
    1245.19
    1160.12];

    Cla=0.0799; %lift slope curve in degrees

    thrust_standard=thrust_le_red+thrust_re_red;
    
    thrust=thrust_re+thrust_le;
    
    Delta_deltae_eq=-0.5; %change in degrees in elevator angle due to moving passenger
    
    %REFERENCE DATA for cg before the shift cg_data==3
    %REFERENCE DATA for cg after the shift cg_data==4
    
    [xcg2,xcg2_datum]=cg_calculator(4);
    [xcg1,xcg1_datum]=cg_calculator(3);
    
    Delta_xcg=xcg2_datum-xcg1_datum;
    

elseif data==2
    load('FTISxprt-20190306_102158.mat', 'flightdata')
    t_trim = [1808
    1902
    1995
    2062
    2146
    2302];

    t_cg_shift=[2415
    2492];

    thrust_le=[1674.58
    1727.77
    1764.75
    1780.12
    1663.92
    1641.48];

    thrust_re=[2120.01
    2173.42
    2213.05
    2240.09
    2101.63
    2077.32];

    thrust_le_red=[1235.06
    1298.38
    1354.70
    1400.86
    1201.58
    1129.79];

    thrust_re_red=[1235.06
    1298.38
    1354.70
    1400.86
    1201.58
    1129.79];

    
    thrust_standard=thrust_le_red+thrust_re_red;
    thrust=thrust_re+thrust_le;
    
    Cla=0.077419; %lift slope curve in degrees
   
    
    Delta_deltae_eq= -0.67; %change in degrees in elevator angle due to moving passenger
    
    %FLIGHT DATA for cg before the shift data==1
    %FLIGHT DATA for cg after the shift data==2
    
    [xcg2,xcg2_datum]=cg_calculator(2);
    [xcg1,xcg1_datum]=cg_calculator(1);
    
    Delta_xcg=xcg2_datum-xcg1_datum;
    
end

%Reference, stationary meas. 2
%% retrieve the necessary indices
for i=1:length(t_trim)
    index(i) = find(flightdata.time.data==t_trim(i));
end

%cg change
for i=1:length(t_cg_shift)
    index_cg(i) = find(flightdata.time.data==t_cg_shift(i));
end
index_cg=index_cg';
%% extract measurement values for stationary and cg shift data
V_cas = flightdata.Dadc1_cas.data(index)*0.514444;
%V_tas1 = flightdata.Dadc1_tas.data(index)*0.514444;

a = flightdata.vane_AOA.data(index); % angle of attack in degrees

deltae_eq=flightdata.delta_e.data(index); %reduced elevator deflection

pressure_altitude=flightdata.Dadc1_alt.data(index)*0.3048; %pressure altitude

F_e=flightdata.column_fe.data(index); % stick force in Newton

%%
pressure_altitude_cg=flightdata.Dadc1_alt.data(index_cg)*0.3048
V_cas_cg=flightdata.Dadc1_cas.data(index_cg)*0.514444;


%% general constants
%rho = 1.225; %kg/m3
g = 9.80665; %m/s2

%cessna citation parameters
S = 30.00; %m2
m = 6033; %kg
b = 15.911; %m
A = (b^2/S); % aspect ration unitless
c = 2.0569; % m
W_s=60500; %standard aircraft weight

%% constant input
lambda = -0.0065; %K\m
gamma=1.4; %gas constant
p_0 = 101325; %Pa
Temp0 = 288.15; %K
rho0 = 1.225; %kg/m3
R = 287; %gas constant
mu=1.778e-5;

%% calculate most paramters
for i = 1:size(a)
    
    p(i)=p_0*(1+lambda*pressure_altitude(i)/Temp0)^(-(g/(lambda*R))); %pressure

    M(i)=sqrt(2/(gamma-1)*((1+p_0/p(i)*((1+(gamma-1)/(2*gamma)* rho0/p_0*V_cas(i)^2)^(gamma/(gamma-1))-1))^((gamma-1)/gamma)-1)); % mac number

    Temp(i)=Temp0/(1+(gamma-1)/2*M(i)^2); % temperature

    a_sound(i)=sqrt(gamma*R*Temp(i)); % speed of soun

    V_tas(i)=M(i)*a_sound(i); %true airspeed
    
    rho(i) = p(i)/R/Temp(i); % density at the datapoint
    
    V_e(i) = V_tas(i)*sqrt(rho(i)/rho0); %equivalent airspeed
    
    mass(i) = weight_calculator(t_trim(i), data);  % mass
    
    W(i) = mass(i)*g;  % weight 
    
    Ve_red(i)=V_e(i)*sqrt(W_s/W(i)); %reduced airspeed as function of weight
    
    Tc(i)=thrust(i)/(0.5*rho(i)*S*V_tas(i)^2); % thrust coefficient
    
    Tc_s(i)=thrust_standard(i)/(0.5*rho0*S*Ve_red(i)^2); %standard thrust coefficient
    
    CN(i)=W(i)/(0.5*rho(i) *V_tas(i)^2*S); % Cn different times
    
    F_e_reduced(i)=F_e(i)*W_s/W(i);
    
    Re(i)=rho(i)*c*V_tas(i)/mu;
    
end

%% calculate Cn for cg shift

for i = 1:size(index_cg)
    
    p_cg(i)=p_0*(1+lambda*pressure_altitude_cg(i)/Temp0)^(-(g/(lambda*R))); %pressure

    M_cg(i)=sqrt(2/(gamma-1)*((1+p_0/p_cg(i)*((1+(gamma-1)/(2*gamma)* rho0/p_0*V_cas_cg(i)^2)^(gamma/(gamma-1))-1))^((gamma-1)/gamma)-1)); % mac number

    Temp_cg(i)=Temp0/(1+(gamma-1)/2*M_cg(i)^2); % temperature

    a_sound_cg(i)=sqrt(gamma*R*Temp_cg(i)); % speed of soun

    V_tas_cg(i)=M_cg(i)*a_sound_cg(i); %true airspeed
    rho_cg(i) = p_cg(i)/R/Temp_cg(i); % density at the datapoint
    mass_cg(i) = weight_calculator(t_cg_shift(i), data);  % mass
    
    W_cg(i) = mass_cg(i)*g;  % weight 
    CN_cg(i)=W_cg(i)/(0.5*rho_cg(i) *V_tas_cg(i)^2*S); % Cn different times
    
end

%% calculate Cm_delta
% use the average of the cg before and after fabio moves
Cm_delta=-1/deg2rad(Delta_deltae_eq)*mean(CN_cg)*Delta_xcg/c;

%% find delta alpha curve

hold on
scatter(a, deltae_eq);
regress = polyfit(a, deltae_eq,1);
m=regress(1);
b=regress(2);
a_plot=-2:0.1:10;
plot(a_plot, m*a_plot+b);

% regression to find cm alpha
regress_radians = polyfit(deg2rad(a), deg2rad(deltae_eq),1);
m_radians=regress_radians(1);
b_radians=regress_radians(2);

xlabel('Angle of attack [deg]');
ylabel('Deflection Angle [deg]');
lgd_slope=legend({'Flight Data Angle of Deflection', '^{-C_{m_{\alpha}}}/_{C_{m_{\delta}}} \alpha_a Regression'}, 'Location','Northeast');
lgd_slope.FontSize=24;
set(lgd_slope,'FontName','Helvetica');
grid on;

%Adding annotations
mdl = fitlm(a,deltae_eq,'linear');
r_square = sprintf('%.2f',mdl.Rsquared.Ordinary);
pvalue = sprintf('%.2e',mdl.Coefficients.pValue(2));
str = {'e = '+string(e)+', R^2 = '+r_square+', p-value = '+pvalue};
box = annotation('textbox',[0.148 0.6 0.3 0.3],'String',str,'FitBoxToText','on');
box.FontSize = 24;
set(gca,'FontSize',24)
pbaspect([1 1 1])

%% calculate Cm_alpha
Cm_alpha=-m_radians*Cm_delta;

CLa_theoretical=0.0887; %lift slope curve in degrees %input data from table C.2

%% Coefficients of Normal Force

CNa=Cla; %normal force slope is equal to lift one

%% reduced elevator deflection curve

Cm_deltaf=0; % input the cm delta fuselage, assumed to be zero
delta_f=0; % delta fuselage, assumed to be zero
Cm_lg=0; %the configuration is gear up,
Cm_Tc=-0.0064; %input data from table C.2

for i = 1:size(a) 
    deltae_eqred(i)=deltae_eq(i)-(Cm_Tc*(Tc_s(i)-Tc(i))/(Cm_delta));
end

%% Plotting reduced delta
hold on
regress = polyfit(Ve_red, deltae_eqred,2);

b1=regress(1);
b2=regress(2);
b3=regress(3);
Ve_red_plot=80:0.1:110;

% plot(Ve_red_plot, b1*Ve_red_plot.^2+b2*Ve_red_plot+b3);
scatter(Ve_red,deltae_eqred);
ylabel('Reduced Elevator Deflection [deg]');
xlabel('Reduced Equivalent Airspeed [m/s]');
set(gca, 'YDir','reverse');
set(gca,'FontSize',24);
pbaspect([1 1 1]);
grid on;


%Adding annotations
% mdl = fitlm(Ve_red,deltae_eqred,'linear');
% r_square = sprintf('%.2f',mdl.Rsquared.Ordinary);
% pvalue = sprintf('%.2e',mdl.Coefficients.pValue(2));
% str = {'e = '+string(e)+', R^2 = '+r_square+', p-value = '+pvalue};
% box = annotation('textbox',[0.148 0.6 0.3 0.3],'String',str,'FitBoxToText','on');
% box.FontSize = 14;

%% Plotting reduced delta without outlier

Ve_red_final=Ve_red;
deltae_eqred_final=deltae_eqred;

hold on
Ve_red_final(4)=[];
deltae_eqred_final(4)=[];
regress = polyfit(Ve_red_final, deltae_eqred_final,2);

b1=regress(1);
b2=regress(2);
b3=regress(3);
Ve_red_plot=80:0.1:110;

plot(Ve_red_plot, b1*Ve_red_plot.^2+b2*Ve_red_plot+b3);
scatter(Ve_red_final,deltae_eqred_final);
ylabel('Reduced Elevator Deflection [deg]');
xlabel('Reduced Equivalent Airspeed [m/s]');
set(gca,'FontSize',24);
set(gca, 'YDir','reverse');
grid on;


%Adding annotations
mdl = fitlm(Ve_red_final,deltae_eqred_final,'linear');
r_square = sprintf('%.2f',mdl.Rsquared.Ordinary);
pvalue = sprintf('%.2e',mdl.Coefficients.pValue(2));
str = {'e = '+string(e)+', R^2 = '+r_square+', p-value = '+pvalue};
box = annotation('textbox',[0.148 0.6 0.3 0.3],'String',str,'FitBoxToText','on');
box.FontSize = 24;
pbaspect([1 1 1]);


%% reduced force stick curve
hold on
regress = polyfit(Ve_red, F_e_reduced,2);

b1=regress(1);
b2=regress(2);
b3=regress(3);
Ve_red_plot=80:0.1:110;

plot(Ve_red_plot, b1*Ve_red_plot.^2+b2*Ve_red_plot+b3);

scatter(Ve_red,F_e_reduced);
ylabel('Reduced Elevator Stick Force [N]');
xlabel('Reduced Equivalent Airspeed [m/s]');

set(gca, 'YDir','reverse');
grid on;

%Adding annotations
mdl = fitlm(Ve_red,F_e_reduced,'linear');
r_square = sprintf('%.2f',mdl.Rsquared.Ordinary);
pvalue = sprintf('%.2e',mdl.Coefficients.pValue(2));
str = {'e = '+string(e)+', R^2 = '+r_square+', p-value = '+pvalue};
box = annotation('textbox',[0.148 0.6 0.3 0.3],'String',str,'FitBoxToText','on');
box.FontSize = 24;

%% reduced force stick curve without outlier
F_e_reduced_final=F_e_reduced;

hold on
F_e_reduced_final(4)=[];

regress = polyfit(Ve_red_final, F_e_reduced_final,2);

b1=regress(1);
b2=regress(2);
b3=regress(3);
Ve_red_plot=80:0.1:110;

plot(Ve_red_plot, b1*Ve_red_plot.^2+b2*Ve_red_plot+b3);

scatter(Ve_red_final,F_e_reduced_final);
ylabel('Reduced Elevator Stick Force [N]');
xlabel('Reduced Equivalent Airspeed [m/s]');

set(gca, 'YDir','reverse');
grid on;

%Adding annotations
mdl = fitlm(Ve_red_final,F_e_reduced_final,'linear');
r_square = sprintf('%.2f',mdl.Rsquared.Ordinary);
pvalue = sprintf('%.2e',mdl.Coefficients.pValue(2));
str = {'e = '+string(e)+', R^2 = '+r_square+', p-value = '+pvalue};
box = annotation('textbox',[0.148 0.6 0.3 0.3],'String',str,'FitBoxToText','on');
box.FontSize = 24;
set(gca,'FontSize',24)
pbaspect([1 1 1])


%% Plot elevator deflection fabio's movement window
plot(flightdata.delta_e.data(index_cg(1):index_cg(2)))
ylabel('Deflection Angle [deg]');
xlabel('Instances');
set(gca,'FontSize',24)
pbaspect([1 1 1]);
grid on;

%% additional relations that can be used
%deltae_eqred=-(1/Cm_delta)*(Cm_0+(Cm_alpha*W)/(CN_alpha*0.5*rho*Ve_red^2*S)+Cm_deltaf*delta_f+Cm_Tc*Tc_s+Cm_lg); %reduced elevator deflection 
%deltae_eq=-(1/Cm_delta)*(Cm_0+Cm_alpha*W)/(CN_alpha*0.5*rho*V^2*S)+Cm_deltaf*delta_f+Cm_Tc*Tc+Cm_lg); %elevator angle required for longitudinal moment equilibrium 




