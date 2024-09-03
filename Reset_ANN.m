% Reset Network
magnitude = 2;
ANN_W_3 = magnitude*(rand(2,3^3)-0.5);
ANN_W_5 = magnitude*(rand(2,5^3)-0.5);
ANN_W_7 = magnitude*(rand(2,7^3)-0.5);
save Best_Results.mat  ANN_W_3 ANN_W_5 ANN_W_7 
