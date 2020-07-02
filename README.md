# KCPidTuner
A port of the KissingCircle PID Autotuning algorithm from the paper Universal Direct Tuner for Loop Control in Industry by Robin De Keyser et al. (10 June 2019)

Please see the original paper here: https://ieeexplore.ieee.org/document/8733787
The Original code was from: Clara Ionescu - UGent (2020). Universal tuner for all types of processes (https://www.mathworks.com/matlabcentral/fileexchange/71759-universal-tuner-for-all-types-of-processes), MATLAB Central File Exchange. Retrieved July 2, 2020.

The code was ported by using a branch of smop https://github.com/PatrickFURI/smop

All functions were recoded to use the python control library https://python-control.readthedocs.io/en/0.8.1/matlab.html
I cannot guarentee the accuracy of the code. Not all functions are supported (for example, any time delay using exp())

Most of the examples don't work at all. Example #8 is the most stable.

More information on this method has been found in the following papers:
A Robust PID Autotuning Method Applied to the Benchmark PID18
https://www.sciencedirect.com/science/article/pii/S2405896318304452

Experimental Validation of a Novel Auto-Tuning Method for a Fractional Order PI Controller on an UR10 Robot
https://www.mdpi.com/1999-4893/11/7/95/htm

Robust fractional-order auto-tuning for highly-coupled MIMO systems
https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6675949/
