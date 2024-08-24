function Robot_init = RBT_TWMR_GLB_INIT(Theta, xrbt, yrbt, Robot_Model)
    
    % Initialize Robot
    RM.Theta = Theta;
    RM.xrbt = xrbt;
    RM.yrbt = yrbt;
    
    % Define some parameters
    RM.omega = Robot_Model.omega;
    RM.V = Robot_Model.V;
    RM.m = Robot_Model.m;
    RM.M11 = Robot_Model.M11;
    RM.M22 = Robot_Model.M22;



    RM.d = Robot_Model.d;  % check kon in chie ?!!??!?!?!?!?!?
    RM.Tou_Max = Robot_Model.Tou_Max;

    % Define wheel parameters
    RM.rR = Robot_Model.rR; % radius of right wheel
    RM.rL = Robot_Model.rL; % radius of left wheel
    RM.L = Robot_Model.L;    % distance between the two wheels

    % Dynamic of TWMR
    RM.B = [1/RM.rL 1/RM.rL;RM.L/RM.rL -RM.L/RM.rL];
    RM.B_invers = inv(RM.B);
    RM.M = [RM.M11 0;0 RM.M22];
    RM.M_invers = inv(RM.M);
    syms omega;
    RM.FX(omega) = [0 RM.m*RM.d*omega;RM.m*RM.d*omega 0];

    Robot_init = RM;

end
