
function update_ANN(epsilon, MFs_Size, Global_E, Theta, Phi_E)
    
    % load Network data
    G_Theta = [cos(Theta) 0;sin(Theta) 0;0 1];
    switch MFs_Size
        case 3
            load Best_Results.mat ANN_W_3
            ANN_W_3 = ANN_W_3 + epsilon * Global_E * G_Theta * Phi_E';
            save Best_Results.mat ANN_W_3

        case 5
            load Best_Results.mat ANN_W_5
            ANN_W_5 = ANN_W_5 + epsilon * Global_E * G_Theta * Phi_E';
            save Best_Results.mat ANN_W_5

        case 7
            load Best_Results.mat ANN_W_7
            ANN_W_7 = ANN_W_7 + epsilon * Global_E * G_Theta * Phi_E';
            save Best_Results.mat ANN_W_7
    
        otherwise
            disp('MFs_Size is out of range : {3,5,7}')
            disp(' ')
    end

end