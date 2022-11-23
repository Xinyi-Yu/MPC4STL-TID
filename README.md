# MPC4STL-TID
We propose an effective STL synthesis framework - Model Predictive Control for Signal Temporal Logic with Time Interval Decomposition. Here is the paper [link](https://arxiv.org/pdf/2211.08031.pdf). This project is collaborated with Chuwei Wang (main). 

#### Instructions

This project consists of two parts, online part and offline parts, which are conducted by Python 3 and Julia respectively.

##### Online parts

We compared computational efficiency by two ways, with Time Interval Decomposition (TID) and without TID. Both codes can be found in folder `online codes - with TID` and `online codes - without TID`.

##### Offline parts

We will publish them as soon as possible.

#### Dependencies

In the online part, we use the following packages

```
matplotlib    v3.6.1
numpy     v1.23.4
Gurobi   v9.5.2
```

In the offline part, we use the following packages

```
CDDLib v0.9.1
GR v0.69.5
IntervalArithmetic v0.20.7
LaTeXStrings v1.3.0
LazySets v2.3.0
Plots v1.36.0
Polyhedra v0.7.5
ReachabilityAnalysis v0.21.0
```

#### Acknowledgement

