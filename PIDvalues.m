function [Kp,Ti,Td]=PIDvalues(r,Omega,P,L);
Complex=L/P; a=real(Complex); b=imag(Complex);
Kp=a; Ti=r*(b+sqrt(b^2+4*a^2/r))/(2*a*Omega); Td=Ti/r;