const blue = RGB(0.8, 0.8, 1);
const green = RGB(0.74, 0.88, 0.88);
const L = 1.281;

A = [1.0 0.5 0.0 0.0;
    0.0 1.0 0.0 0.0;
    0.0 0.0 1.0 0.5;
    0.0 0.0 0.0 1.0
];
B = [0.125 0;
    0.5 0.0;
    0.0 0.125;
    0.0 0.5
];

X = Hyperrectangle([5, 0.0, 5, 0.0], [5, 2.5, 5, 2.5]);
U = Hyperrectangle([0.0, 0.0], [3.0, 3.0]);
goal = [Hyperrectangle([8.75, 0.0, 1.25, 0.0], [1.25, 2.5, 1.25, 2.5]), Hyperrectangle([1.5, 0.0, 1.5, 0.0], [1.5,2.5, 1.5, 2.5])];
w = Hyperrectangle([0.0, 0.0, 0.0, 0.0],[0.01,0.01,0.01,0.01]);





