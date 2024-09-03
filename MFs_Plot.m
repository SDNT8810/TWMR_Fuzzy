clear; clc
FuzzIn = -6:0.05:6;
C = 0.2;
C1 = 0.2;
C2 = 0.6;
MC = 0.02;
L = 2;
L1 = 1;
L2 = 3;
L3 = 3;
s = 5;


x_params = [-s,-L2;C,-L;MC,0;C,L1;L2,s];
y_params = [-s,-L2;C,-L;MC,0;C,L1;L2,s];
t_params = [-s,-L2;C,-L;MC,0;C,L1;L2,s];

x_params = [-s,-L3;C1,-L1;C2,-L2;MC,0;C2,L2;C1,L1;L3,s];
y_params = [-s,-L3;C1,-L1;C2,-L2;MC,0;C2,L2;C1,L1;L3,s];
t_params = [-s,-L3;C1,-L1;C2,-L2;MC,0;C2,L2;C1,L1;L3,s];


mf(1,:) = zmf(FuzzIn,x_params(1,:));
mf(2,:) = gaussmf(FuzzIn,x_params(2,:));
mf(3,:) = gaussmf(FuzzIn,x_params(3,:));
mf(4,:) = gaussmf(FuzzIn,x_params(4,:));
mf(5,:) = gaussmf(FuzzIn,x_params(5,:));
mf(6,:) = gaussmf(FuzzIn,x_params(6,:));
mf(7,:) = smf(FuzzIn,x_params(7,:));

plot(FuzzIn, mf)