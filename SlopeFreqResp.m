function [FR,dFR]=SlopeFreqResp(Sys,Omega);
FR=evalfr(Sys,j*Omega);
Omega1=0.99*Omega; FR1=evalfr(Sys,j*Omega1);
Omega2=1.01*Omega; FR2=evalfr(Sys,j*Omega2);
dFR=(FR2-FR1)/(Omega2-Omega1);
end