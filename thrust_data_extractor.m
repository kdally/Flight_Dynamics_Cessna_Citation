%% MEASUREMENT SERIES 1

%Find indices of measurement times
times = [2239,2351,2484,2576,2741,2840,2920];   
for i=1:length(times)
    index(i) = find(flightdata.time.data==times(i));
end

fuel_flow_right_engine  = flightdata.rh_engine_FMF.data(index)*0.45359/3600;
fuel_flow_left_engine   = flightdata.lh_engine_FMF.data(index)*0.45359/3600;
mach_number             = flightdata.Dadc1_mach.data(index);
pressure_altitude       = flightdata.Dadc1_alt.data(index)*0.3048;
delta_temperature       = flightdata.Dadc1_sat.data(index)+273.15 - (286.40 - (0.0065/3.28084)*flightdata.Dadc1_alt.data(index));
x                       = [pressure_altitude mach_number delta_temperature fuel_flow_left_engine fuel_flow_right_engine];

save('matlab.dat','x', '-ascii');


%% MEASUREMENT SERIES 2

%Find indices of measurement times
times = [875, 1033, 1124, 1243, 1365, 1540];
for i=1:length(times)
    index(i) = find(flightdata.time.data==times(i));
end

fuel_flow_right_engine  = flightdata.rh_engine_FMF.data(index)*0.45359/3600;
fuel_flow_left_engine   = flightdata.lh_engine_FMF.data(index)*0.45359/3600;
mach_number             = flightdata.Dadc1_mach.data(index);
pressure_altitude       = flightdata.Dadc1_alt.data(index)*0.3048;
delta_temperature       = flightdata.Dadc1_sat.data(index)+273.15 - (286.40 - (0.0065/3.28084)*flightdata.Dadc1_alt.data(index));
x                       = [pressure_altitude mach_number delta_temperature fuel_flow_left_engine fuel_flow_right_engine];

save('matlab.dat','x', '-ascii');


