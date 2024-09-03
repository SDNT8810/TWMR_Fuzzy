
clc

N = 100;
I_Max = [3 3 1];
I_Min = -[3 3 1];
MFs_Size = 7;
Type = "gaussmf";
Global_E_X = 0;
Global_E_Y = 0;
Global_E_T = 0;

O_Max = [1 1 1];
O_Min = -[1 1 1];
O_MFs_Size = 3;

switch MFs_Size
    case 3
        Types = {'zmf' ,'gaussmf' ,'smf'};
        x_params = [I_Min(1) -0.05;0.02 0;0.05 I_Max(1)];
        y_params = [I_Min(2) -0.05;0.02 0;0.05 I_Max(2)];
        t_params = [I_Min(3) -0.05;0.02 0;0.05 I_Max(3)];

    case 5
        Types = {'zmf' ,'gaussmf' ,'gaussmf' ,'gaussmf' ,'smf'};
        Gaussian_C = 0.3;
        mid_Gaussian_C = 0.02;
        x_params = [I_Min(1) I_Min(1)/2;Gaussian_C I_Min(1)/2;mid_Gaussian_C 0;Gaussian_C I_Max(1)/2;I_Max(1)/2 I_Max(1)];
        y_params = [I_Min(2) I_Min(2)/2;Gaussian_C I_Min(2)/2;mid_Gaussian_C 0;Gaussian_C I_Max(2)/2;I_Max(2)/2 I_Max(2)];
        t_params = [I_Min(3) I_Min(3)/2;Gaussian_C I_Min(3)/2;mid_Gaussian_C 0;Gaussian_C I_Max(3)/2;I_Max(3)/2 I_Max(3)];
    
    case 7
        Types = {'zmf' ,'gaussmf' ,'gaussmf' ,'gaussmf' ,'gaussmf' ,'gaussmf' ,'smf'};
        Gaussian_C = 0.25;
        mid_Gaussian_C = 0.01;
        x_params = [I_Min(1) 2*I_Min(1)/3;Gaussian_C 2*I_Min(1)/3;Gaussian_C I_Min(1)/3;mid_Gaussian_C 0;Gaussian_C I_Max(1)/3;Gaussian_C 2*I_Max(1)/3;2*I_Max(1)/3 I_Max(1)];
        y_params = [I_Min(2) 2*I_Min(2)/3;Gaussian_C 2*I_Min(2)/3;Gaussian_C I_Min(2)/3;mid_Gaussian_C 0;Gaussian_C I_Max(2)/3;Gaussian_C 2*I_Max(2)/3;2*I_Max(2)/3 I_Max(2)];
        t_params = [I_Min(3) 2*I_Min(3)/3;Gaussian_C 2*I_Min(3)/3;Gaussian_C I_Min(3)/3;mid_Gaussian_C 0;Gaussian_C I_Max(3)/3;Gaussian_C 2*I_Max(3)/3;2*I_Max(3)/3 I_Max(3)];

    otherwise
        disp('undefined MFs_Size')
        disp(' ')
end


for i = 1 : MFs_Size
    mfx(i) = fismf(Types{i},x_params(i,:));
    mfy(i) = fismf(Types{i},y_params(i,:));
    mft(i) = fismf(Types{i},t_params(i,:));
    Phi_X(i) = evalmf(mfx(i),Global_E_X);
    Phi_Y(i) = evalmf(mfy(i),Global_E_Y);
    Phi_T(i) = evalmf(mft(i),Global_E_T);
end

Fuzzy_Rulls_Matrix_1 = zeros(MFs_Size^3, 3);
Fuzzy_Rulls_Matrix_2 = zeros(MFs_Size^3, 3);
m = 0;
for i = 1 : MFs_Size
    for j = 1 : MFs_Size
        for k = 1 : MFs_Size
            m = m + 1;
            Fuzzy_Rulls_Matrix_1(m, :) = [Phi_X(i) Phi_Y(j) Phi_T(k)];
            Fuzzy_Rulls_Matrix_2(m, :) = [Phi_X(i) Phi_Y(j) Phi_T(k)];
        end
    end
end

Phi_E_1 = ones(MFs_Size^3,1);
Phi_E_2 = ones(MFs_Size^3,1);
for j = 1 : O_MFs_Size
    Phi_E_1 = Phi_E_1 .* Fuzzy_Rulls_Matrix_1(:,j);
    Phi_E_2 = Phi_E_2 .* Fuzzy_Rulls_Matrix_2(:,j);
end


Fuzzy_Phi = [Phi_E_1 Phi_E_2];

x = -5 : 0.1 : 5;

Phi_X_1 = evalmf(mfx(1),x);
Phi_X_2 = evalmf(mfx(2),x);
Phi_X_3 = evalmf(mfx(3),x);
Phi_X_4 = evalmf(mfx(4),x);
Phi_X_5 = evalmf(mfx(5),x);
Phi_X_6 = evalmf(mfx(6),x);
Phi_X_7 = evalmf(mfx(7),x);


plot(x,Phi_X_1,x,Phi_X_2,x,Phi_X_3,x,Phi_X_4,x,Phi_X_5,x,Phi_X_6,x,Phi_X_7)
