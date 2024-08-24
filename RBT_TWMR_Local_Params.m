function Robot_Model = RBT_TWMR_Local_Params(m, M11, M22, d, rR, rL, L, Tou_Max)

    % Define some parameters
    RM.omega = 0;
    RM.V = 0;
    RM.m = m;
    RM.M11 = M11;
    RM.M22 = M22;
    RM.d = d;
    RM.Tou_Max = Tou_Max;

    % Define wheel parameters
    RM.rR = rR; % radius of right wheel
    RM.rL = rL; % radius of left wheel
    RM.L = L;    % distance between the two wheels
    
    Robot_Model = RM;

end