lambda1 = Clp*V0/(4*mub*KX2*b);
lambda2 = (2*CL*(Clb*Cnr - Clr*Cnb))/((b/V0)*(Clp*(Cnr*CYb + 4*mub*Cnb) - Cnp*(CYb*Clr + 4*mub*Clb)));
lambda2_3 = roots([4*(mub^2)*KZ2*(b^3)/(V0^3) -2*mub*(b^2)*(KZ2*CYb+(Cnr/2))/(V0^2) (b/(2*V0))*(CYb*Cnr+4*mub*Cnb)]);
lambda2_3_2 = roots([-2*mub*KZ2*(b^2/V0^2) (b/(2*V0))*Cnr -Cnb]);