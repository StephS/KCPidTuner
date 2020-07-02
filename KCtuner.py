import numpy as np
import matplotlib.pyplot as plt
import control.matlab as cm
import PIDvalues as pv
#from SlopeFreqResp import SlopeFreqResp
import SlopeFreqResp as sfr
import math as m

from matplotlib.figure import *
#from matplotlib.hold import *

def cosd(x):
    return np.cos(x * np.pi / 180)

def sind(x):
    return np.sin(x * np.pi / 180)

def tand(x):
    return np.tan(x * np.pi / 180)
    #f = -(-n +sum(  np.log(b*c*np.power(data,(c-1))*    exp(-b*np.power(data,c)))                 ))
    #f = -(-n +sum([sym.log(b*c*        (num**(c-1))*sym.exp(-b*        (num**c))) for num in data]))
#import sympy as sp
#def tand(x):
#    return sp.tan(x * sp.pi / 180)

#def sind(x):
#    return sp.sin(x * sp.pi / 180)

#def cosd(x):
#    return sp.cos(x * sp.pi / 180)

def KCtuner(GM=None,PM=None,Omeg=None,P=None,dP=None):
    #Robin De Keyser - Ghent University - RDK050619 - Robain.DeKeyser@UGent.be
    s=cm.tf('s')
    # KCtuner.m:4
    C=(GM ** 2 - 1) / (np.dot(np.dot(2,GM),(np.dot(GM, cosd(PM)) - 1)))
    # KCtuner.m:5
    R=C - 1 / GM
    # KCtuner.m:5
    AlfaMin=0
    # KCtuner.m:5
    AlfaMax=60
    # C=(GM^2-1)/(2*GM*(GM*cosd(PM)-1)); R=C-1/GM; AlfaMin=0; AlfaMax=60; Alfa=[AlfaMin:AlfaMax]';
    # KCtuner.m:5
    # The colon operator also allows you to create an equally spaced vector of values using the more general form start:step:end.
    # If you omit the middle step, as in start:end, MATLAB uses the default step value of 1.
    # Alfa=np.asarray([AlfaMin:AlfaMax], dtype='object').T
    # numpy.arange([start, ]stop, [step, ]dtype=None)
    # *.T = The transposed array. Same as self.transpose().
    #Alfa=np.arange(AlfaMin, AlfaMax, dtype='object').T
    Alfa=np.arange(AlfaMin, AlfaMax).T
    # KCtuner.m:5
    ReVek=- C + np.dot(R,cosd(Alfa))
    # KCtuner.m:6
    ImVek=np.dot(- R,sind(Alfa))
    # KCtuner.m:6
    AlfaC=90 - Alfa
    # KCtuner.m:6
    rMin=3
    # KCtuner.m:7
    rMax=5
    # KCtuner.m:7
    
    rVek=rMax - np.dot((rMax - rMin),(Alfa - AlfaMin)) / (AlfaMax - AlfaMin)
    # KCtuner.m:8
    #AlfaL=np.asarray([], dtype='object')
    #AlfaL=np.array([]) #  np.asarray([], dtype='object')
    AlfaL=np.asarray([])
    # KCtuner.m:10
    for n in range(1,Alfa.size+1):
        L=ReVek[n-1] + np.dot(1j,ImVek[n-1])
        # KCtuner.m:12
        r=rVek[n-1]
        # KCtuner.m:12
        # Kp,Ti,Td=pv.PIDvalues(r,Omeg,P,L,nargout=3)
        Kp,Ti,Td=pv.PIDvalues(r,Omeg,P,L)
        # KCtuner.m:13
        PID=np.dot(Kp,(1 + 1 / (np.dot(Ti,s)) + np.dot(Td,s)))
        # KCtuner.m:13
        # C,dC=sfr.SlopeFreqResp(PID,Omeg,nargout=2)
        C,dC=sfr.SlopeFreqResp(PID,Omeg)
        # KCtuner.m:14
        dL=np.dot(P,dC) + np.dot(C,dP)
        # KCtuner.m:15
        # To create a matrix that has multiple rows, separate the rows with semicolons.
        # dL=P*dC+C*dP; AlfaL=[AlfaL; atan2(imag(dL),real(dL))*180/pi];
        # matlab syntax = P = atan2(Y,X)
        # AlfaL=np.asarray([[AlfaL],[np.dot(atan2(cm.imag(dL),cm.real(dL)),180) / np.pi]], dtype='object')
        #AlfaL=np.asarray([[AlfaL],[np.dot( np.arctan2(np.imag(dL),np.real(dL)),180) / np.pi]], dtype='object')
        #AlfaL=np.asarray([[AlfaL],[np.dot( np.arctan2(np.imag(dL),np.real(dL)),180) / np.pi]])
        #AlfaL=np.asarray([[AlfaL],[np.dot( np.arctan2(np.imag(dL),np.real(dL)),180) / np.pi]]).T
        AlfaL=np.append(AlfaL, [np.dot( np.arctan2(np.imag(dL),np.real(dL)),180) / np.pi])
        # AlfaL=np.asarray([[AlfaL],[np.dot( np.arctan2(np.imag(dL),np.real(dL)),180) / np.pi]])
    # KCtuner.m:15
    # Err=abs(AlfaL-AlfaC); [MinValue,i]=min(Err);
    Err=np.fabs(AlfaL - AlfaC)
    # KCtuner.m:18
    # MinValue,i=min(Err,nargout=2)
    # [M,I] = min(___) also returns the index into the operating dimension that corresponds to the minimum value of A for any of the previous syntaxes.
    #  i=min(Err)
    i = np.argmin(Err)
    #MinValue=i
    # KCtuner.m:18
    L=ReVek[i-1] + np.dot(1j,ImVek[i-1])
    # KCtuner.m:19
    r=rVek[i-1]
    # KCtuner.m:19
    # Kp,Ti,Td=PIDvalues(r,Omeg,P,L,nargout=3)
    Kp,Ti,Td=pv.PIDvalues(r,Omeg,P,L)
    # KCtuner.m:19
    
    plt.plot(ReVek,ImVek,'r','LineWidth',3)
    plt.axis('equal')
    
    Y=np.arange(ImVek[i-1] - 0.2,ImVek[i-1] + 0.2+0.01,0.01)
    # KCtuner.m:22
    X=ReVek[i-1] + (Y - ImVek[i-1]) / tand(AlfaL[i-1])
    # KCtuner.m:22
    plt.plot(X,Y,'k','LineWidth',3)
    return Kp,Ti,Td
