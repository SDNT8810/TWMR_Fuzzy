function Controller_Model = RBT_TWMR_Local_Controller(Controller_Type, Coefficients, Model)
    
    switch Controller_Type
        case 'Unit_Mass_MBC'
            disp(['Controller Type  is ' Controller_Type ' with landa_1 = ' num2str(Coefficients.landa1) ' , and landa_2 = ' num2str(Coefficients.landa2) ' .'])
            disp(' ')
            CM.landa_1 = Coefficients.landa1;
            CM.landa_2 = Coefficients.landa2;
            CM.Model = Model;
            syms V omega Vd_dot omega_d_dot E_V E_omega int_E_V int_E_omega E_dot_V E_dot_omega;
            % FX(omega) = [0 Model.m*Model.d*omega;Model.m*Model.d*omega 0];
            FX(omega) = Model.FX(omega);
            Tou(V, omega, Vd_dot, omega_d_dot, E_V, E_omega, int_E_V, int_E_omega, E_dot_V, E_dot_omega) = Model.B_invers * (FX * [V;omega] + Model.M * ([Vd_dot;omega_d_dot] + CM.landa_2 * [int_E_V;int_E_omega] + CM.landa_1 * [E_V;E_omega]));
           %Tou(V, omega, Vd_dot, omega_d_dot, E_V, E_omega, int_E_V, int_E_omega, E_dot_V, E_dot_omega) = Model.B_invers * (FX * [V;omega] + Model.M * ([Vd_dot;omega_d_dot] + CM.landa_2 * [int_E_V;int_E_omega] + CM.landa_1 * [E_dot_V;E_dot_omega]));
                        
            CM.Function(V, omega, Vd_dot, omega_d_dot, E_V, E_omega, int_E_V, int_E_omega, E_dot_V, E_dot_omega) = Tou;
            
        case 'NN_Dynamic_Model'
            disp(['Controller Type  is ' Controller_Type ' with landa_1 = ' num2str(Coefficients.landa1) ' , and landa_2 = ' num2str(Coefficients.landa2) ' .'])
            disp(' ')

            CM.landa_1 = Coefficients.landa1;
            CM.landa_2 = Coefficients.landa2;
            CM.Model = Model;
            syms V omega Vd_dot omega_d_dot E_V E_omega int_E_V int_E_omega E_dot_V E_dot_omega Psi_hat;

            FX(omega) = Model.FX(omega);
            
            Tou = @(V, omega, Vd_dot, omega_d_dot, E_V, E_omega, int_E_V, int_E_omega, E_dot_V, E_dot_omega, Psi_hat) (Model.B_invers * (FX(omega) * [V;omega] + Model.M * ([Vd_dot;omega_d_dot] + CM.landa_2 * [int_E_V;int_E_omega] + CM.landa_1 * [E_V;E_omega] + Psi_hat)));      
                        
            CM.Function = @(V, omega, Vd_dot, omega_d_dot, E_V, E_omega, int_E_V, int_E_omega, E_dot_V, E_dot_omega, Psi_hat) Tou;
            
            
        otherwise
            disp('undefined controller type')
            disp(' ')
    end
    
    CM.Controller_Type = Controller_Type;
    Controller_Model = CM;
end