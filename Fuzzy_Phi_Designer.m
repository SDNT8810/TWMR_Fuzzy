function Fuzzy_Phi = Fuzzy_Phi_Designer(MFs_Size, Type, I_Min, I_Max, O_MFs_Size, O_Min, O_Max, Global_E_X, Global_E_Y, Global_E_T)
    
    Gaussian_C_X = 0.3 * I_Max(1);
    Gaussian_Center_X = linspace(I_Min(1), I_Max(1), MFs_Size+2);
    Gaussian_Center_X(1) = [];
    Gaussian_Center_X(end) = [];
    Gaussian_C_Y = 0.3 * I_Max(2);
    Gaussian_Center_Y = linspace(I_Min(2), I_Max(2), MFs_Size+2);
    Gaussian_Center_Y(1) = [];
    Gaussian_Center_Y(end) = [];
    Gaussian_C_T = 0.3 * I_Max(3);
    Gaussian_Center_T = linspace(I_Min(3), I_Max(3), MFs_Size+2);
    Gaussian_Center_T(1) = [];
    Gaussian_Center_T(end) = [];
    
    for i = 1 : MFs_Size
        mfx(i) = fismf(Type,[Gaussian_C_X Gaussian_Center_X(i)]);
        mfy(i) = fismf(Type,[Gaussian_C_Y Gaussian_Center_Y(i)]);
        mft(i) = fismf(Type,[Gaussian_C_T Gaussian_Center_T(i)]);
        Phi_X(i) = evalmf(mfx(i),Global_E_X);
        Phi_Y(i) = evalmf(mfy(i),Global_E_Y);
        Phi_T(i) = evalmf(mft(i),Global_E_T);
    end
    
    Fuzzy_Rulls_Matrix_1 = zeros(MFs_Size^3, 3);
    m = 0;
    for i = 1 : MFs_Size
        for j = 1 : MFs_Size
            for k = 1 : MFs_Size
                m = m + 1;
                Fuzzy_Rulls_Matrix_1(m, :) = [Phi_X(i) Phi_Y(j) Phi_T(k)];
            end
        end
    end
    
    Phi_E_1 = ones(MFs_Size^3,1);
    for j = 1 : O_MFs_Size
        Phi_E_1 = Phi_E_1 .* Fuzzy_Rulls_Matrix_1(:,j);

    end
    
    
    Fuzzy_Phi = [Phi_E_1 Phi_E_1]/sum(Phi_E_1);

end
