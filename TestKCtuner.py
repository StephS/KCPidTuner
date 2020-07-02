#from matplotlib.figure import *
import numpy as np
import control.matlab as cm
import SlopeFreqResp as sfr
import KCtuner as kct
import matplotlib as mpl
import matplotlib.pyplot as plt


def formatF(n):
    if isinstance(n, float):
        n = f'{n:0.3f}'
    elif not isinstance(n, str):
        n = str(n)
    #a = "{0:{1}}".format(n, '0.2f')
    return n

def printCols(rows):
    rows_formatted = [[formatF(cell) for cell in row] for row in rows]
    widths = [max(map(len, col)) for col in zip(*rows_formatted)]

    for row in rows_formatted:
        print("  ".join((val.ljust(width) for val, width in zip(row, widths))))

# Robin De Keyser - Ghent University - RDK050619 - Robain.DeKeyser@UGent.be
#s = cm.TransferFunction.s
s = cm.tf('s')

# TestKCtuner.m:4 # 1 Sys=32/(s*(s+4)*(s+16)); PIDref=25.1361*(1+1/(0.6272*s)+0.1568*s); #integrating damped 2nd order;
#Sys=32 / (np.dot(np.dot(s,(s + 4)),(s + 16))) #integrating damped 2nd order;
#PIDref=np.dot(25.1361,(1 + 1 / (np.dot(0.6272,s)) + np.dot(0.1568,s))) #integrating damped 2nd order;

# TestKCtuner.m:5 # 2 Sys=tf(2,[50 15 1],'InputDelay',25); PIDref=0.3*(1+1/(20*s)+5*s); #delayed 2nd order;
#Sys = cm.tf(2, np.asarray([50, 15, 1]), 'InputDelay', 25)  # delayed 2nd order;
#PIDref = np.dot(0.3, (1 + 1 / (np.dot(20, s)) + np.dot(5, s))) # delayed 2nd order;

# TestKCtuner.m:6 # 3 Sys=tf(1,[1 1 0],'InputDelay',1); PIDref=0.75*(1+1/(6*s)+1.333*s); #FOIPDT;
#Sys=cm.tf(1,np.asarray([1,1,0], dtype='object'),'InputDelay',1) #FOIPDT;
#PIDref=np.dot(0.75,(1 + 1 / (np.dot(6,s)) + np.dot(1.333,s))) #FOIPDT;

# 4 Sys=((1-0.2*s)*exp(-0.1*s))/(s+1)^2; PIDref=2.4*(1+1/(3*s)+0.4167*s); #NMP

# TestKCtuner.m:8 # 5 Sys=(s+6)^2/(s*(s+1)^2*(s+36)); PIDref=6.843*(1+1/(6.5637*s)+1.461*s); #integrating and zeros
#Sys=(s + 6) ** 2 / (np.dot(np.dot(s,(s + 1) ** 2),(s + 36)))
#PIDref=np.dot(6.843,(1 + 1 / (np.dot(6.5637,s)) + np.dot(1.461,s)))

# TestKCtuner.m:9 # 6 Sys=1/((s+1)*(s^2+0.2*s+1)); PIDref=0.9*(1+1/(4.5*s)+1.111*s); #poorly damped;
#Sys = 1 / (np.dot((s + 1), (s ** 2 + np.dot(0.2, s) + 1)))  # poorly damped;
#PIDref = np.dot(0.9, (1 + 1 / (np.dot(4.5, s)) + np.dot(1.111, s)))  # poorly damped;

# TestKCtuner.m:10 # 7 Sys=1/(s+1)^5; PIDref=0.7*0.921*(1+1/(1.961*s)+1.969*s); #Chen example A
#Sys = 1 / (s + 1) ** 5
#PIDref=np.dot(np.dot(0.7,0.921),(1 + 1 / (np.dot(1.961,s)) + np.dot(1.969,s)))

# TestKCtuner.m:11 # 8 Sys=1/(s*(s+1)^3); PIDref=0.33*(1+1/(6.53*s)+1.89*s); #Chen example B
Sys = 1 / (np.dot(s, (s + 1) ** 3))  # Chen example B
PIDref = np.dot(0.33, (1 + 1 / (np.dot(6.53, s)) + np.dot(1.89, s)))  # Chen example B

# TestKCtuner.m:12 # 9 Sys=exp(-s)/(s+1)^3; PIDref=1.024*(1+1/(1.241*s)+1.539*s); #Chen example C
#Sys = np.exp( np.dot(-1, s)) / (s + 1) ** 3  # Chen example C
#PIDref = np.dot(1.024, (1 + 1 / (np.dot(1.241, s)) + np.dot(1.539, s)))  # Chen example C

# 10 Sys=exp(-s)/(s*(s+1)^3); PIDref=0.212*(1+1/(9.52*s)+2.061*s); #Chen example D

# [Gm,Pm,Wcg,Wcp]=margin(Sys);
Gm, Pm, Wcg, Wcp = cm.margin(Sys)
# Omeg=Wcg;
Omeg = np.copy(Wcg)
# [P,dP]=SlopeFreqResp(Sys,Omeg);
P, dP = sfr.SlopeFreqResp(Sys, Omeg)  # in reality via a sine test

GM = 2
# TestKCtuner.m:16
PM = 45
# TestKCtuner.m:16
# Kp,Ti,Td=kct.KCtuner(GM,PM,Omeg,P,dP,nargout=3)
Kp, Ti, Td = kct.KCtuner(GM, PM, Omeg, P, dP)
# calculate Ki and Kd
# Ki = Kp / Ti and Kd = Kp * Td.
Ki = Kp / Ti
Kd = Kp * Td

# PID=Kp*(1+1/(Ti*s)+Td*s);
PID = np.dot(Kp, (1 + 1 / (np.dot(Ti, s)) + np.dot(Td, s)))
# for whatever the numer and denom are reversed. change it back
PID2 = cm.tf(PID.den, PID.num)
PIDref2 = cm.tf(PIDref.den, PIDref.num)

print(PID)
print(PID2)
print(PIDref)
print(PIDref2)

# K=Kp*Td;
K = np.dot(Kp, Td)

# z=roots([1 1/Td 1/(Ti*Td)]);
z = np.roots(np.asarray([1, 1 / Td, 1 / (np.dot(Ti, Td))]))

# TestKCtuner.m:18 # disp('      Kp        Ti        Td        K        z1        z2'); disp([Kp Ti Td K z']);
# Note that ' is the CTRANSPOSE operator in MATLAB.
printCols([['Kp', 'Ti', 'Td', 'Ki', 'Kd', 'K', 'z1', 'z2'],
           [Kp, Ti, Td, Ki, Kd, K, z[0], z[1]]])

# nyquist(PID*Sys,'r',PIDref*Sys,'b:');
cm.nyquist(np.dot(PID, Sys), color='r')
cm.nyquist(np.dot(PIDref, Sys), color='b')

# axis([-1.5 0 -2 2]);
plt.axis(np.asarray([- 1.5, 0, - 2, 2]))
# axis('equal');
plt.axis('equal')
# title('Nyquist plot - KC circle DEFAULT GM=2 and PM=45�');
plt.title('Nyquist plot - KC circle DEFAULT GM=2 and PM=45(deg)')
# legend('KC','Slope','KC-PID','REF-PID');
plt.legend(['KC', 'Slope', 'KC-PID', 'REF-PID'])

# figure; step(feedback(ss(PID*Sys),1),'r',feedback(ss(PIDref*Sys),1),'b:');
plt.figure()
yout1, T1 = cm.step(cm.feedback(cm.ss(np.dot(PID2, Sys)), 1))
yout2, T2 = cm.step(cm.feedback(cm.ss(np.dot(PIDref2, Sys)), 1))
plt.plot(T1, yout1, label='KC-PID')
plt.plot(T2, yout2, label='REF-PID')
# title('Setpoint change'); legend('KC-PID','REF-PID');
plt.title('Setpoint change')
plt.legend('KC-PID', 'REF-PID')

#from figure import *
# plt.step(cm.feedback(ss(Sys),ss(PID)),'r',cm.feedback(ss(Sys),ss(PIDref)),'b:')
# figure; step(feedback(ss(Sys),ss(PID)),'r',feedback(ss(Sys),ss(PIDref)),'b:');
plt.figure()
#plt.step(cm.feedback(cm.ss(Sys), cm.ss(PID)), 'r')
#plt.step(cm.feedback(cm.ss(Sys), cm.ss(PIDref)), 'b')
# figure; step(feedback(ss(Sys),ss(PID)),'r',feedback(ss(Sys),ss(PIDref)),'b:');

# sbla = cm.ss(Sys)
# pbla = cm.ss(PID2)
# bla = cm.feedback(sbla, pbla)
yout3, T3 = cm.step(cm.feedback(cm.ss(Sys), cm.ss(PID2)))
plt.plot(T3, yout3, label='KC-PID')

#cm.step(cm.feedback(cm.ss(Sys), cm.ss(PID)))
yout4, T4 = cm.step(cm.feedback(cm.ss(Sys), cm.ss(PIDref2)))

plt.plot(T4, yout4, label='REF-PID')
# title('Load disturbance'); legend('KC-PID','REF-PID');
plt.title('Load disturbance')
plt.legend('KC-PID', 'REF-PID')
plt.show()
