function[xcg,x_cg_datum]=cg_calculator(data)
%here's a program to calculate the cg position before and after Fabio moves


%FLIGHT DATA for cg before the shift data==1
%FLIGHT DATA for cg after the shift data==2
%REFERENCE DATA for cg before the shift data==3
%REFERENCE DATA for cg after the shift data==4

if data==1

    fuel_remain=8909.783;    %N
    xcg_datum_fuel=7.612;    %m
    xcg_datum_8=7.315;       %m
    
    %weights all in N
    w_1=872.792;
    w_2=902.212;
    w_3=637.432;
    w_4=715.885;
    w_5=745.305;
    w_6=764.919;
    w_7=666.852;
    w_8=833.565;
    w_10=686.466;
    w_ac=40767.951;
    
elseif data==2

    fuel_remain=8811.922;    %N
    xcg_datum_fuel=7.252;    %m
    xcg_datum_8=3.327;       %m
    
    %weights all in N
    w_1=872.792;
    w_2=902.212;
    w_3=637.432;
    w_4=715.885;
    w_5=745.305;
    w_6=764.919;
    w_7=666.852;
    w_8=833.565;
    w_10=686.466;
    w_ac=40767.951;
   
%Note that in the reference data, passenger 7 is the one moving forward, not 8.     
elseif data==3

    fuel_remain=14096.414;    %N
    xcg_datum_fuel=7.243;    %m
    xcg_datum_8=7.315;       %m
    
    %weights all in N
    w_1=931.632;
    w_2=902.212;
    w_3=647.329;
    w_4=598.206;
    w_5=735.499;
    w_6=764.919;
    w_8=843.372;
    w_7=666.852;
    w_10=725.692;
    w_ac=40767.951;
elseif data==4

    fuel_remain=13967.416;    %N
    xcg_datum_fuel=7.242;    %m
    xcg_datum_8=3.404;       %m
   
    %weights all in N
    w_1=931.632;
    w_2=902.212;
    w_3=647.329;
    w_4=598.206;
    w_5=735.499;
    w_6=764.919;
    w_8=843.372;
    w_7=666.852;
    w_10=725.692;
    w_ac=40767.951;
end

%starting point of the m.a.c wrt the beginning of the reference datum
x_le_mac=6.644;     %m

%xcg all in meters with respect to the reference datum
xcg_datum_1=3.327;
xcg_datum_2=3.327;
xcg_datum_3=5.436;
xcg_datum_4=5.436;
xcg_datum_5=6.375;
xcg_datum_6=6.375;
xcg_datum_7=7.315;
xcg_datum_10=4.318;
xcg_datum_ac=7.421;

M=((w_1*xcg_datum_1)+(w_2*xcg_datum_2)+(w_3*xcg_datum_3)+(w_4*xcg_datum_4)+(w_5*xcg_datum_5)+(w_6*xcg_datum_6)+(w_7*xcg_datum_7)+(w_8*xcg_datum_8)+(w_10*xcg_datum_10)+(w_ac*xcg_datum_ac)+(fuel_remain*xcg_datum_fuel));
W=(w_1+w_2+w_3+w_4+w_5+w_6+w_7+w_8+w_10+w_ac+fuel_remain);
x_cg_datum=M/W
xcg=x_cg_datum-x_le_mac
return


