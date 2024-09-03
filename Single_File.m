%% Start Clean project
clc
clear
close all

tic

%% Define General Params
%load params.mat

O_MFs_Size = 3;
MFs_Size = 7;
ANN_W_Name = ['ANN_W_' num2str(MFs_Size)];
ANN_W_Struct = load('Best_Results.mat',ANN_W_Name);
ANN_W_Matrix = struct2array(ANN_W_Struct);

% Robot Params
m = 2;
M11 = 5;
M22 = 6;
d = 0.3;
rR = 0.075;
rL = 0.075;
L = 0.3;
Tou_Max = 100;

% Initial Conditions
Theta_0 = 0;
xrbt_0 = 0;
yrbt_0 = -2;

% Time params
t_zero = 0;
dt_sim = 0.002;
t_Max  = 8;

% Solution params
% Path_Type => { 'circle' , 'spiral' , 'line' , 'sin' }
Path_Type = 'circle';
Path_Size = 3;

% Design Local Controller
Landa.landa1 = 5;
Landa.landa2 = 15;
%Controller_Type = 'Fast_NN_Dynamic_Model';

I_Max = [5 5 2];
I_Min = - I_Max;
O_Max = [10 10 10];
O_Min = - O_Max;
fis = fuzzy_def(MFs_Size, I_Min , I_Max);

epsilon = 0.001;

%% initializing

Robot_Params = RBT_TWMR_Local_Params(m, M11, M22, d, rR, rL, L, Tou_Max);

Robot_Dynamic_Model = RBT_TWMR_GLB_INIT(Theta_0, xrbt_0, yrbt_0, Robot_Params);

% Define the Time Struct (t_zero,dt_sim,t_Max)
Time_Struct = RBT_TWMR_Time_Condition(t_zero,dt_sim,t_Max);

% Desired Conditions
d_c = RBT_TWMR_GLB_desired_condition(Path_Type, Path_Size, Time_Struct);

% Initialize actual robot state
RBT_States = RBT_TWMR_GLB_States(Robot_Dynamic_Model, d_c, Time_Struct);    

%MBC = RBT_TWMR_Local_Controller('Unit_Mass_MBC', Landa, Robot_Dynamic_Model);
%MBC = RBT_TWMR_Local_Controller(Controller_Type, Landa, Robot_Dynamic_Model);
MBC.landa_1 = Landa.landa1;
MBC.landa_2 = Landa.landa2;
MBC.Model = Robot_Dynamic_Model;
MBC.Function = 1;

% Initializing Controller Parameters

T = Time_Struct;
Desired_Condition = d_c;
omega = Robot_Dynamic_Model.omega;

d = Robot_Dynamic_Model.d;
m = Robot_Dynamic_Model.m;
V = Robot_Dynamic_Model.V;
B = Robot_Dynamic_Model.B;
Theta = Robot_Dynamic_Model.Theta;
xrbt = Robot_Dynamic_Model.xrbt;
yrbt = Robot_Dynamic_Model.yrbt;
xrbt_d = Desired_Condition.Xd;
yrbt_d = Desired_Condition.Yd;
Theta_d = Desired_Condition.THETAd;

M_invers = Robot_Dynamic_Model.M_invers;
B_invers = Robot_Dynamic_Model.B_invers;
M = Robot_Dynamic_Model.M;
Vd_dot = Desired_Condition.Vd_dot;
Vd = Desired_Condition.Vd;
OMEGA_d = Desired_Condition.OMEGA_d;
OMEGA_d_dot = Desired_Condition.OMEGA_d_dot;
E = RBT_States.E;
int_E = RBT_States.int_E;
E_dot = RBT_States.E_dot;
q = RBT_States.X;
q_dot = RBT_States.X_dot;
%CL_Tou = MBC.Function;
landa1 = Landa.landa1;
landa2 = Landa.landa2;
%Controller_Type = MBC.Controller_Type;

Tou = RBT_States.Tou;
int_Global_E_P = 0;

save params.mat


%% Simulation

for i = 2:length(T.t)
    clc
    disp([int2str(i) ' / ' int2str(length(T.t))])

    % States & Errors
    qd_dot = [Vd_dot(i-1);OMEGA_d_dot(i-1)];
    E(:,i) = [Vd(i-1) - V;OMEGA_d(i-1) - omega];
    int_E(:,i) = int_E(:,i-1) + E(:,i-1)*T.dt_sim ;
    E_dot(:,i) = (E(:,i) - E(:,i-1))/T.dt_sim ;

    % Global Errors
    Global_E_X = xrbt_d(i) - xrbt(i-1);
    Global_E_Y = yrbt_d(i) - yrbt(i-1);
    Global_E_T = Theta_d(i) - Theta(i-1);
    Global_E = [Global_E_X Global_E_Y Global_E_T];
    Global_E_P = [Global_E_X;Global_E_Y];
    int_Global_E_P = int_Global_E_P + Global_E_P*T.dt_sim 

    Fuzzy_Phi = Fuzzy_Phi_Evaluation(fis, Global_E);
    RBT_States.Fuzzy_Phi(i,:) = Fuzzy_Phi(:,1)';
    Psi_hat = (ANN_W_Matrix * Fuzzy_Phi)'


    RBT_States.psi(i,:) = Psi_hat';
    FX = [0 m*d*omega;m*d*omega 0];
    Tou(:,i) = B_invers * (FX * [V;omega] + M * (qd_dot + landa2 * int_E(:,i) + landa1 * E(:,i) + 10 * Psi_hat));
    RBT_States.U_local(:,i) =  qd_dot + landa2 * int_E(:,i) + landa1 * E(:,i);

    % saturartion
    % Tou(:,i) = saturate(Tou(:,i),Model.Tou_Max);

    % System Dynamic Simulation
    q_dot(:,i) = M_invers * (B * Tou(:,i) - FX * q(:,i-1));
    q(:,i) = q(:,i-1) + q_dot(:,i) * T.dt_sim;
    V = q(1,i);
    omega = q(2,i);

    % Global Configuration
    Theta(i) = Theta(i-1) + T.dt_sim*omega;
    xrbt(i) = xrbt(i-1) + T.dt_sim*V*cos(Theta(i));
    yrbt(i) = yrbt(i-1) + T.dt_sim*V*sin(Theta(i));

    % update_ANN(epsilon, Global_ERROR, Theta(i), Phi_E);
      update_ANN(epsilon  , MFs_Size, Global_E    , Theta(i), Fuzzy_Phi);

end


% Saving States and Results
RBT_States.E = E;
RBT_States.int_E = int_E;
RBT_States.E_dot = E_dot;
RBT_States.Tou = Tou;
RBT_States.X = q;
RBT_States.X_dot = q_dot;

Results = RBT_States;
Results.Theta = Theta;
Results.xrbt = xrbt;
Results.yrbt = yrbt;

%% Report Results and Plots
RBT_TWMR_GLB_Report(Results, d_c, Time_Struct);

toc

%% Functions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Evaluate Fuzzy Function
function Fuzzy_Phi = Fuzzy_Phi_Evaluation(fis, Global_E)

Global_E_X = Global_E(1);
Global_E_Y = Global_E(2);
Global_E_T = Global_E(3);
mfx = fis.mfx;
mfy = fis.mfy;
mft = fis.mft;
MFs_Size = fis.MFs_Size;

for ii = 1 : MFs_Size
    Phi_X(ii) = evalmf(mfx(ii),Global_E_X);
    Phi_Y(ii) = evalmf(mfy(ii),Global_E_Y);
    Phi_T(ii) = evalmf(mft(ii),Global_E_T);
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

Phi_E_1 = max(Fuzzy_Rulls_Matrix_1,[],2);

Fuzzy_Phi = [Phi_E_1 Phi_E_1]/abs(sum(Phi_E_1));

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Saturate Outputs
function G = saturate(T,T_Maximum)

% saturartion
if (T(1) >  T_Maximum)
    T(1) =  T_Maximum;
end

if (T(1) < -T_Maximum)
    T(1) = -T_Maximum;
end

if (T(2) >  T_Maximum)
    T(2) =  T_Maximum;
end

if (T(2) < -T_Maximum)
    T(2) = -T_Maximum;
end

G = T;
end


% update_ANN
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


% Robot_Parameters 
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


% Initialize Robot
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
    RM.FX(omega) = [0 Robot_Model.m*Robot_Model.d*omega;Robot_Model.m*Robot_Model.d*omega 0];

    Robot_init = RM;

end


% Define the Time Struct
function Time_Struct = RBT_TWMR_Time_Condition(t_zero,dt_sim,t_Max)
    
    T.t_zero = t_zero;
    T.dt_sim = dt_sim;
    T.t_Max = t_Max;
    T.t_span = [t_zero t_Max];  % from 0 to t_Max seconds
    T.t = 0 : dt_sim : t_Max;   % from 0 to t_Max seconds with a step of dt_sim seconds
    Time_Struct = T;

end


% Desired Conditions (Path_Type, Path_Size, Time)
function D_C = RBT_TWMR_GLB_desired_condition(Path_Type, Path_Size, Time)

    switch Path_Type
        case 'circle'
            disp(['desired path is a circle with R = ' num2str(Path_Size) 'm'])
            disp(' ')
        
            % Define the desired path with time-varying components
            Path_Params.Xd = Path_Size * sin(2 * pi * Time.t);               % Desired X positions (sinusoidal motion along X axis)
            Path_Params.Yd = -Path_Size * cos(2 * pi * Time.t);              % Desired Y positions (sinusoidal motion along Y axis)
            Path_Params.THETAd = 2 * pi * Time.t;                            % Desired orientation (linear angle change)

        case 'spiral'
            disp(['desired path is a spiral with R(t = 1s) = ' num2str(Path_Size) 'm'])
            disp(' ')
        
            % Define the desired path with time-varying components
            Path_Params.Xd = Path_Size * sin(2 * pi * Time.t) .* Time.t;     % Desired X positions (exponential sinusoidal motion along X axis)
            Path_Params.Yd = -Path_Size * cos(2 * pi * Time.t) .* Time.t;    % Desired Y positions (exponential sinusoidal motion along Y axis)
            Path_Params.THETAd = 2 * pi * Time.t;                            % Desired orientation (linear angle change)
            
        case 'line'
            disp(['desired path is a line with b = ' num2str(Path_Size.b) ' and theta = ' num2str(Path_Size.theta)])
            disp(' ')  

            % Define the desired path with time-varying components
            Path_Params.Xd = Time.t;                                         % Desired X positions (linear motion along X axis)
            Path_Params.Yd = Time.t;                                         % Desired Y positions (linear motion along Y axis)
            Path_Params.THETAd = 0 * Time.t + Path_Size;                     % Desired orientation (linear angle change)
        
        case 'sin'
            disp(['desired path is sin with A = ' num2str(Path_Size)])
            disp(' ')   

            % Define the desired path with time-varying components
            Path_Params.Xd = Path_Size * Time.t;                             % Desired X positions (linear motion along X axis)
            Path_Params.Yd = Path_Size * sin(2 * pi * Time.t);               % Desired Y positions (sinusoidal motion along Y axis)
            Path_Params.THETAd = 2 * pi * Path_Size * cos(2 * pi * Time.t) ;                    % Desired orientation (linear angle change)

        otherwise
            disp('undefined path type')
            disp(' ')
    end

    % Calculate numerical derivatives using finite differences
    Path_Params.Xd_dot = diff(Path_Params.Xd) / Time.dt_sim;
    Path_Params.Yd_dot = diff(Path_Params.Yd) / Time.dt_sim;
    Path_Params.THETAd_dot = diff(Path_Params.THETAd) / Time.dt_sim;

    % To keep arrays the same length, append the last element again
    Path_Params.Xd_dot = [Path_Params.Xd_dot, Path_Params.Xd_dot(end)];
    Path_Params.Yd_dot = [Path_Params.Yd_dot, Path_Params.Yd_dot(end)];
    Path_Params.THETAd_dot = [Path_Params.THETAd_dot, Path_Params.THETAd_dot(end)];

    % Calculate desired velocities
    Path_Params.Vd = sqrt(Path_Params.Xd_dot.^2 + Path_Params.Yd_dot.^2);
    Path_Params.OMEGA_d = Path_Params.THETAd_dot;
    % Calculate numerical derivatives using finite differences
    Path_Params.Vd_dot = diff(Path_Params.Vd) / Time.dt_sim;
    Path_Params.OMEGA_d_dot = diff(Path_Params.OMEGA_d) / Time.dt_sim;

    % To keep arrays the same length, append the last element again
    Path_Params.Vd_dot = [Path_Params.Vd_dot, Path_Params.Vd_dot(end)];
    Path_Params.OMEGA_d_dot = [Path_Params.OMEGA_d_dot, Path_Params.OMEGA_d_dot(end)];

    D_C = Path_Params;
end


% Initialize actual robot state
function RBT_States = RBT_TWMR_GLB_States(Robot_Dynamic_Model, d_c, T)
    % Initialize arrays to store actual state
    RS.X = zeros(2,length(T.t));
    RS.X_dot = zeros(2,length(T.t));
    RS.E = zeros(2,length(T.t));
    RS.int_E = zeros(2,length(T.t));
    RS.E_dot = zeros(2,length(T.t));
    RS.Tou = zeros(2,length(T.t));
    RS.xrbt = zeros(1,length(T.t));
    RS.yrbt = zeros(1,length(T.t));
    RS.Theta = zeros(1,length(T.t));
    
    % Initialize robot state
    RS.X(:,1)   = [Robot_Dynamic_Model.V;Robot_Dynamic_Model.omega];
    RS.E(:,1)   = [Robot_Dynamic_Model.V(1);Robot_Dynamic_Model.omega(1)]-[d_c.Vd(1);d_c.OMEGA_d(1)];
    RS.xrbt(1)  =  Robot_Dynamic_Model.xrbt;
    RS.yrbt(1)  =  Robot_Dynamic_Model.yrbt;
    RS.Theta(1) =  Robot_Dynamic_Model.Theta;
        
    RBT_States = RS;
end


% Define Fuzzy System
function fis = fuzzy_def(MFs_Size, I_Min , I_Max)

    switch MFs_Size
        case 3
            Types = {'zmf' ,'gaussmf' ,'smf'};
            x_params = [I_Min(1) -0.05;0.02 0;0.05 I_Max(1)];
            y_params = [I_Min(2) -0.05;0.02 0;0.05 I_Max(2)];
            t_params = [I_Min(3) -0.05;0.02 0;0.05 I_Max(3)];
    
        case 5
            Types = {'zmf' ,'gaussmf' ,'gaussmf' ,'gaussmf' ,'smf'};
            % Gaussian_C = 0.3;
            % mid_Gaussian_C = 0.02;
            % x_params = [I_Min(1) I_Min(1)/2;Gaussian_C I_Min(1)/2;mid_Gaussian_C 0;Gaussian_C I_Max(1)/2;I_Max(1)/2 I_Max(1)];
            % y_params = [I_Min(2) I_Min(2)/2;Gaussian_C I_Min(2)/2;mid_Gaussian_C 0;Gaussian_C I_Max(2)/2;I_Max(2)/2 I_Max(2)];
            % t_params = [I_Min(3) I_Min(3)/2;Gaussian_C I_Min(3)/2;mid_Gaussian_C 0;Gaussian_C I_Max(3)/2;I_Max(3)/2 I_Max(3)];
            %
            C = 0.2;
            MC = 0.02;
            L = 1;
            L2 = 1;
            s = 2;


            x_params = [-s,-L2;C,-L;MC,0;C,L;L2,s];
            y_params = [-s,-L2;C,-L;MC,0;C,L;L2,s];
            t_params = [-s,-L2;C,-L;MC,0;C,L;L2,s];

        case 7
            Types = {'zmf' ,'gaussmf' ,'gaussmf' ,'gaussmf' ,'gaussmf' ,'gaussmf' ,'smf'};
            C1 = 0.2;
            C2 = 0.6;
            MC = 0.02;
            L1 = 1;
            L2 = 3;
            L3 = 3;
            s = 5;
            x_params = [-s,-L3;C1,-L1;C2,-L2;MC,0;C2,L2;C1,L1;L3,s];
            y_params = [-s,-L3;C1,-L1;C2,-L2;MC,0;C2,L2;C1,L1;L3,s];
            t_params = [-s,-L3;C1,-L1;C2,-L2;MC,0;C2,L2;C1,L1;L3,s];
        otherwise
            disp('undefined MFs_Size')
            disp(' ')
    end

    
    for i = 1 : MFs_Size
        mfx(i) = fismf(Types{i},x_params(i,:));
        mfy(i) = fismf(Types{i},y_params(i,:));
        mft(i) = fismf(Types{i},t_params(i,:));
    end

    fis.mfx = mfx;
    fis.mfy = mfy;
    fis.mft = mft;
    fis.MFs_Size = MFs_Size;
    
end


% Report
function RBT_TWMR_GLB_Report(Results, Desired_Conditions, Time)
    
    t = Time.t;
    X = Results.X;
    E = Results.E; 
    xrbt = Results.xrbt; 
    yrbt = Results.yrbt; 
    Theta = Results.Theta; 
    Xd = Desired_Conditions.Xd;     
    Yd = Desired_Conditions.Yd; 
    Vd = Desired_Conditions.Vd;
    OMEGA_d = Desired_Conditions.OMEGA_d; 
    THETAd = Desired_Conditions.THETAd; 

    % Plots
    figure(1)
    subplot(3,1,1)
    plot(t, X(1,:),'r-', 'LineWidth', 2)
    hold on;
    plot(t, Vd,'b-.', 'LineWidth', 2)
    xlabel('Time (s)');
    ylabel('V (m/s)');
    legend('DESIRED VELOCITY', 'ACTUAL VELOCITY');
    title(' DESIRED VELOCITY VS ACTUAL VELOCITY');
    grid on;
    title(' X Vs Time(s) ');
    
    subplot(3,1,2)
    plot(t, X(2,:),'r-', 'LineWidth', 2)
    hold on;
    plot(t, OMEGA_d,'b-.', 'LineWidth', 2)
    xlabel('Time (s)');
    ylabel1 = ylabel('$\omega$ (Rad/s)');
    legend1 = legend('DESIRED $\omega$', 'ACTUAL $\omega$');
    title1 = title(' DESIRED $\omega$ VS ACTUAL $\omega$');
    set(title1,'Interpreter','latex');
    set(legend1,'Interpreter','latex');
    set(ylabel1,'Interpreter','latex');
    grid on;
    
    subplot(3,1,3)
    plot(t, E(1,:), 'r', 'LineWidth', 2);
    hold on;
    plot(t, E(2,:), 'b-.', 'LineWidth', 2)
    xlabel('Time (s)');
    ylabel('ERROR');
    legend('VELOCITY ERROR', 'OMEGA ERROR');
    title('ERROR');
    grid on;
    
    
    figure(2);
    subplot(3,2,1)
    plot(t, Xd,'r-', 'LineWidth', 2)
    hold on;
    plot(t, xrbt,'b-.', 'LineWidth', 2)
    xlabel('Time (s)');
    ylabel('X (m)');
    legend('DESIRED X', 'ACTUAL X');
    grid on;
    
    subplot(3,2,3)
    plot(t, Yd,'r-', 'LineWidth', 2)
    hold on;
    plot(t, yrbt,'b-.', 'LineWidth', 2)
    xlabel('Time (s)');
    ylabel('Y (m)');
    legend('DESIRED Y', 'ACTUAL Y');
    grid on;
    
    subplot(3,2,5)
    plot(t, THETAd,'r-', 'LineWidth', 2)
    hold on;
    plot(t, Theta,'b-.', 'LineWidth', 2)
    legend1 = legend('DESIRED $\theta$', 'ACTUAL $\theta$');
    set(legend1,'Interpreter','latex');
    xlabel('Time (s)');
    ylabel1 = ylabel('$\theta$ (Rad)');
    set(ylabel1,'Interpreter','latex');
    grid on;
    
    subplot(3,2,2)
    plot(t, Xd-xrbt,'LineWidth', 2)
    xlabel('Time (s)');
    ylabel('Error X (m)');
    legend('Error X');
    grid on;
    
    subplot(3,2,4)
    plot(t, Yd-yrbt, 'LineWidth', 2)
    xlabel('Time (s)');
    ylabel('Error Y (m)');
    legend('Error Y');
    grid on;
    
    subplot(3,2,6)
    plot(t, THETAd-Theta,'LineWidth', 2)
    legend1 = legend('Error $\theta$');
    set(legend1,'Interpreter','latex');
    xlabel('Time (s)');
    ylabel1 = ylabel('Error $\theta$ (Rad)');
    set(ylabel1,'Interpreter','latex');
    grid on;
    
    figure(3);
    plot(Xd,Yd,'r-', 'LineWidth', 2)
    hold on;
    plot(xrbt, yrbt,'b-.', 'LineWidth', 2)
    xlabel('X (m)');
    ylabel('Y (m)');
    legend('DESIRED Location', 'ACTUAL Location');
    grid on;
    title('DESIRED Location Vs ACTUAL Location');
    axis equal

    
    MFs_Size = round((1+size(Results.Fuzzy_Phi,2))^(1/3));
    switch MFs_Size
        case 3
            load Best_Results.mat ANN_W_3
            ANN_W = ANN_W_3;

        case 5
            load Best_Results.mat ANN_W_5
            ANN_W = ANN_W_5;

        case 7
            load Best_Results.mat ANN_W_7
            ANN_W = ANN_W_7;

        otherwise
            disp('MFs_Size is out of range : {3,5,7}')
            disp(' ')
    end

    figure(4);
    plot(ANN_W)


    figure(5)
    subplot(321)
    plot(Results.psi(:,1))
    subplot(322)
    plot(Results.psi(:,2))
    subplot(323)
    plot(Results.U_local(1,:))
    subplot(324)
    plot(Results.U_local(2,:))
    subplot(325)
    plot(Results.Tou(1,:))
    subplot(326)
    plot(Results.Tou(2,:))

    figure(6)
    plot(Results.Fuzzy_Phi)

end



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
            
        case 'Fast_NN_Dynamic_Model'
            disp(['Controller Type  is ' Controller_Type ' with landa_1 = ' num2str(Coefficients.landa1) ' , and landa_2 = ' num2str(Coefficients.landa2) ' .'])
            disp(' ')

            CM.landa_1 = Coefficients.landa1;
            CM.landa_2 = Coefficients.landa2;
            CM.Model = Model;
            CM.Function = 1;
            
        otherwise
            disp('undefined controller type')
            disp(' ')
    end
    
    CM.Controller_Type = Controller_Type;
    Controller_Model = CM;
end