function Psi_hat = RBT_TWMR_ANN_Fuzzy(Global_E_X, Global_E_Y, Global_E_T, Fuzzy_Phi)
    load Best_Results.mat ANN_W;
    
    Psi_hat = ANN_W *  Fuzzy_Phi * [Global_E_X;Global_E_Y;Global_E_T];

end
