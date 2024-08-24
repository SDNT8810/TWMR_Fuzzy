function Results = RBT_TWMR_GLB_Simulation(Model, Controller, T, Desired_Condition, RBT_States)
    
    % Initializing Parameters
    Fuzzification_Type = "gaussmf";

    MFs_Size = 3;
    O_MFs_Size = 3;
    load Best_Results.mat ANN_W_3;
    ANN_W_Matrix = ANN_W_3;

    omega = Model.omega;
    d = Model.d;
    m = Model.m;
    V = Model.V;
    B = Model.B;
    Theta = Model.Theta;
    xrbt = Model.xrbt;
    yrbt = Model.yrbt;
    xrbt_d = Desired_Condition.Xd;
    yrbt_d = Desired_Condition.Yd;
    Theta_d = Desired_Condition.THETAd;

    M_invers = Model.M_invers;
    B_invers = Model.B_invers;
    M = Model.M;
    Vd_dot = Desired_Condition.Vd_dot;
    Vd = Desired_Condition.Vd;
    OMEGA_d = Desired_Condition.OMEGA_d;
    OMEGA_d_dot = Desired_Condition.OMEGA_d_dot;
    E = RBT_States.E;
    int_E = RBT_States.int_E;
    E_dot = RBT_States.E_dot;
    %Tou = RBT_States.Tou;    
    q = RBT_States.X;
    q_dot = RBT_States.X_dot;
    CL_Tou = Controller.Function;
    landa1 = Controller.landa_1;
    landa2 = Controller.landa_2;
    Controller_Type = Controller.Controller_Type;
    
    Tou = RBT_States.Tou;
    
    
    for i = 2:length(T.t)
        clc
        disp([int2str(i) ' / ' int2str(length(T.t))])
        % States & Errors
        qd_dot = [Vd_dot(i-1);OMEGA_d_dot(i-1)];
        E(:,i) = [Vd(i-1) - V;OMEGA_d(i-1) - omega];
        int_E(:,i) = int_E(:,i-1) + E(:,i-1)*T.dt_sim ;
        E_dot(:,i) = (E(:,i) - E(:,i-1))/T.dt_sim ;
    
        % Controll Law
        %syms Global_E_X Global_E_Y Global_E_T Tou_Psi;
        Global_E_X = xrbt_d(i) - xrbt(i-1);
        Global_E_Y = yrbt_d(i) - yrbt(i-1);
        Global_E_T = Theta_d(i) - Theta(i-1);
        Global_E = [Global_E_X,Global_E_Y,Global_E_T];
        
        switch Controller_Type
            case 'Unit_Mass_MBC'
                FX = [0 m*d*omega;m*d*omega 0];
                Tou(:,i) = B_invers * (FX * [V;omega] + M * (qd_dot + landa2 * int_E(:,i) + landa1 * E(:,i)));

            case 'NN_Dynamic_Model'
                I_Max = [3 3 0.5];
                I_Min = - I_Max;
                O_Max = [3 3 1];
                O_Min = - O_Max;
                % I_Min = [-3 -3 -0.5];

                Fuzzy_Phi = Fuzzy_Phi_Designer(MFs_Size, Fuzzification_Type, I_Min , I_Max, O_MFs_Size, O_Min, O_Max, Global_E_X, Global_E_Y, Global_E_T);

                %load('Random_TSK_FIS.mat', 'fis');
                %Phi_E = evalfis(fis, [Global_E_X Global_E_Y Global_E_T]);
                %Phi_E = Fuzzy_Phi * [Global_E_X;Global_E_Y;Global_E_T];
                
                Psi_hat = (ANN_W_Matrix * Fuzzy_Phi)';

                %                          (V, omega, Vd_dot   , omega_d_dot, E_V   , E_omega, int_E_V   , int_E_omega, E_dot_V   , E_dot_omega, Psi_hat);
                Tou_Psi = @(Psi_hat) CL_Tou(V, omega, qd_dot(1), qd_dot(2)  , E(1,i), E(2,i) , int_E(1,i), int_E(2,i) , E_dot(1,i), E_dot(2,i) , Psi_hat);

                Tou_F = Tou_Psi(1*Psi_hat);

                Tou(:,i) = Tou_F(V, omega, qd_dot(1), qd_dot(2), E(1,i), E(2,i), int_E(1,i), int_E(2,i), E_dot(1,i), E_dot(2,i), Psi_hat);

            otherwise
                disp('undefined controller type')
                disp(' ')
        end

        
        % saturartion 
        % Tou(:,i) = saturate(Tou(:,i),Model.Tou_Max);
    
        % System Dynamic Simulation
        q_dot(:,i) = M_invers * (B * Tou(:,i) - Model.FX(q(2,i-1)) * q(:,i-1));
        q(:,i) = q(:,i-1) + q_dot(:,i) * T.dt_sim;
        V = q(1,i);
        omega = q(2,i);
        
        % Global Configuration
        Theta(i) = Theta(i-1) + T.dt_sim*omega;
        xrbt(i) = xrbt(i-1) + T.dt_sim*V*cos(Theta(i));
        yrbt(i) = yrbt(i-1) + T.dt_sim*V*sin(Theta(i));
        
        % update_ANN(epsilon, MFs_Size, Global_ERROR, Theta(i), Phi_E);
        update_ANN(0.001   , MFs_Size, Global_E    , Theta(i), Fuzzy_Phi);
    end

    % Saving States and Results
    RS.E = E;
    RS.int_E = int_E;
    RS.E_dot = E_dot;
    RS.Tou = Tou;    
    RS.X = q;
    RS.X_dot = q_dot;

    Results = RS;
    Results.Theta = Theta;
    Results.xrbt = xrbt;
    Results.yrbt = yrbt;

end
