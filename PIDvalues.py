import numpy as np
import control.matlab as cm
def PIDvalues(r=None,Omega=None,P=None,L=None):

    Complex=L / P
    # PIDvalues.m:2
    a=np.real(Complex)
    # PIDvalues.m:2
    b=np.imag(Complex)
    # PIDvalues.m:2
    Kp=np.copy(a)
    # PIDvalues.m:3
    Ti=np.dot(r,(b + np.sqrt(b ** 2 + np.dot(4,a ** 2) / r))) / (np.dot(np.dot(2,a),Omega))
    # PIDvalues.m:3
    Td=Ti / r
    # PIDvalues.m:3
    return Kp, Ti, Td