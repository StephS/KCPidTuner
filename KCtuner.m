function [Kp,Ti,Td]=KCtuner(GM,PM,Omeg,P,dP);
%Robin De Keyser - Ghent University - RDK050619 - Robain.DeKeyser@UGent.be

s=tf('s');
C=(GM^2-1)/(2*GM*(GM*cosd(PM)-1)); R=C-1/GM; AlfaMin=0; AlfaMax=60; Alfa=[AlfaMin:AlfaMax]';
ReVek=-C+R*cosd(Alfa); ImVek=-R*sind(Alfa); AlfaC=90-Alfa;
rMin=3; rMax=5; %r=rMax at AlfaMin; r=rMin at AlfaMax 
rVek=rMax-(rMax-rMin)*(Alfa-AlfaMin)/(AlfaMax-AlfaMin);

AlfaL=[];
for n=1:length(Alfa), 
    L=ReVek(n)+j*ImVek(n); r=rVek(n);
    [Kp,Ti,Td]=PIDvalues(r,Omeg,P,L); PID=Kp*(1+1/(Ti*s)+Td*s);
    [C,dC]=SlopeFreqResp(PID,Omeg);
    dL=P*dC+C*dP; AlfaL=[AlfaL; atan2(imag(dL),real(dL))*180/pi];
end;

Err=abs(AlfaL-AlfaC); [MinValue,i]=min(Err);
L=ReVek(i)+j*ImVek(i); r=rVek(i); [Kp,Ti,Td]=PIDvalues(r,Omeg,P,L);

figure; plot(ReVek,ImVek,'r','LineWidth',3); axis('equal'); hold;
Y=ImVek(i)-0.2:0.01:ImVek(i)+0.2; X=ReVek(i)+(Y-ImVek(i))/tand(AlfaL(i)); plot(X,Y,'k','LineWidth',3);
end