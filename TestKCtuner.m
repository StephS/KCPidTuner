%Robin De Keyser - Ghent University - RDK050619 - Robain.DeKeyser@UGent.be

s=tf('s');
%1 Sys=32/(s*(s+4)*(s+16)); PIDref=25.1361*(1+1/(0.6272*s)+0.1568*s); %integrating damped 2nd order;
%2 Sys=tf(2,[50 15 1],'InputDelay',25); PIDref=0.3*(1+1/(20*s)+5*s); %delayed 2nd order;
%3 Sys=tf(1,[1 1 0],'InputDelay',1); PIDref=0.75*(1+1/(6*s)+1.333*s); %FOIPDT;
%4 Sys=((1-0.2*s)*exp(-0.1*s))/(s+1)^2; PIDref=2.4*(1+1/(3*s)+0.4167*s); %NMP
%5 Sys=(s+6)^2/(s*(s+1)^2*(s+36)); PIDref=6.843*(1+1/(6.5637*s)+1.461*s); %integrating and zeros
%6 Sys=1/((s+1)*(s^2+0.2*s+1)); PIDref=0.9*(1+1/(4.5*s)+1.111*s); %poorly damped;
%7 Sys=1/(s+1)^5; PIDref=0.7*0.921*(1+1/(1.961*s)+1.969*s); %Chen example A
%8 Sys=1/(s*(s+1)^3); PIDref=0.33*(1+1/(6.53*s)+1.89*s); %Chen example B
 Sys=exp(-s)/(s+1)^3; PIDref=1.024*(1+1/(1.241*s)+1.539*s); %Chen example C
%10 Sys=exp(-s)/(s*(s+1)^3); PIDref=0.212*(1+1/(9.52*s)+2.061*s); %Chen example D

[Gm,Pm,Wcg,Wcp]=margin(Sys); Omeg=Wcg; [P,dP]=SlopeFreqResp(Sys,Omeg); %in reality via a sine test
GM=2; PM=45; [Kp,Ti,Td]=KCtuner(GM,PM,Omeg,P,dP);

PID=Kp*(1+1/(Ti*s)+Td*s); K=Kp*Td; z=roots([1 1/Td 1/(Ti*Td)]);
disp('      Kp        Ti        Td        K        z1        z2'); disp([Kp Ti Td K ctranspose(z)]);

nyquist(PID*Sys,'r',PIDref*Sys,'b:'); axis([-1.5 0 -2 2]); axis('equal');
title('Nyquist plot - KC circle DEFAULT GM=2 and PM=45�'); legend('KC','Slope','KC-PID','REF-PID');

figure; step(feedback(ss(PID*Sys),1),'r',feedback(ss(PIDref*Sys),1),'b:');
title('Setpoint change'); legend('KC-PID','REF-PID');
figure; step(feedback(ss(Sys),ss(PID)),'r',feedback(ss(Sys),ss(PIDref)),'b:');
title('Load disturbance'); legend('KC-PID','REF-PID');