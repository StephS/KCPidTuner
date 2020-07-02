import numpy as np
import control.matlab as cm


def SlopeFreqResp(Sys=None, Omega=None):
    # j = Imaginary unit sqrt(-1) identify in python as 1j
    FR = cm.evalfr(Sys, np.dot(1j, Omega))
    # SlopeFreqResp.m:2
    Omega1 = np.dot(0.99, Omega)
    # SlopeFreqResp.m:3
    FR1 = cm.evalfr(Sys, np.dot(1j, Omega1))
    # SlopeFreqResp.m:3
    Omega2 = np.dot(1.01, Omega)
    # SlopeFreqResp.m:4
    FR2 = cm.evalfr(Sys, np.dot(1j, Omega2))
    # SlopeFreqResp.m:4
    dFR = (FR2 - FR1) / (Omega2 - Omega1)
    # SlopeFreqResp.m:5
    return FR, dFR
