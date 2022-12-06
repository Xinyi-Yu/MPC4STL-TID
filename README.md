# MPC4STL-TID
We propose an effective STL synthesis framework - Model Predictive Control for Signal Temporal Logic with Time Interval Decomposition. Here is the paper [link](https://arxiv.org/pdf/2211.08031.pdf). This project is collaborated with Chuwei Wang (main contributor). 

#### Instructions

This project consists of two parts, online part and offline part, which are conducted by Python 3 and Julia respectively.

- We compared computational efficiency by two ways, with Time Interval Decomposition (TID) and without TID. Both codes can be found in folders `online codes - with TID` and `online codes - without TID` respectively.
- For the offline part, you can check the final terminal set by figure `offline codes/terminalSetFigure.png` and see some of "intermediate product" by animation `offline codes/animation.gif`. Note that the offline results shown in the figure and animation is the projection to 2 dimensions and you can also plot them in other dimensions by your own.

#### Dependencies

In the online part, we use the following packages

```
matplotlib    v3.6.1
numpy     v1.23.4
Gurobi   v9.5.2
```

In the offline part, we use the following packages

```
GR v0.69.5
IntervalArithmetic v0.20.7
LaTeXStrings v1.3.0
LazySets v2.3.0
Plots v1.36.0
Polyhedra v0.7.5
```

#### Acknowledgement

We would like to thank our colleague Xiaoduo for discussion and his great help in this work.
