%% Function which outputs the weight of the aircraft at any time (in kg)

function[weight]=weight_calculator(time,data)
% for time input the time in seconds
% for data input of reference data input 1, for flight data input 2

if data==1
    
    load('reference.mat', 'flightdata')
    total_fuel=4050*0.453592; %kg
    
elseif data==2
    
    load('FTISxprt-20190306_102158.mat', 'flightdata')
    total_fuel=2700*0.453592; %kg
end

p_w = 696;      %pass. weight (kg)
a_w = 4157.174; %aircraft empty weight (kg)

index = find(flightdata.time.data==time);
left_fu   = flightdata.lh_engine_FU.data(index)*0.453592;
right_fu  = flightdata.rh_engine_FU.data(index)*0.453592;
fuel_used = left_fu+right_fu;                               %add used left and right fuel
weight    = p_w + a_w-fuel_used + total_fuel;               %calculate aircraft weight

return              
