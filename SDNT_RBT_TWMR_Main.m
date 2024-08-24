clc
clear 
close all

% Robot_Parameters                  (m, M11, M22, d  , rR   , rL   , L  , Tou_Max)
Robot_Params = RBT_TWMR_Local_Params(2, 5  , 6  , 0.3, 0.075, 0.075, 0.3, 100);

% Initialize Robot                     (Theta_0, xrbt_0, yrbt_0, Robot_Params)
Robot_Dynamic_Model = RBT_TWMR_GLB_INIT(0,       0,      -3    , Robot_Params);

% Define the Time Struct (t_zero,dt_sim,t_Max)
Time_Struct = RBT_TWMR_Time_Condition(0,0.001,1);

% Desired Conditions (Path_Type, Path_Size, Time) 
% Path_Type => { 'circle' , 'spiral' , 'line' , 'sin' }
d_c = RBT_TWMR_GLB_desired_condition('circle', 3, Time_Struct);

% Initialize actual robot state
RBT_States = RBT_TWMR_GLB_States(Robot_Dynamic_Model, d_c, Time_Struct);    

% Design Controller
Landa.landa1 = 10;
Landa.landa2 = 30;

%MBC = RBT_TWMR_Local_Controller('Unit_Mass_MBC', Landa, Robot_Dynamic_Model);
MBC = RBT_TWMR_Local_Controller('NN_Dynamic_Model', Landa, Robot_Dynamic_Model);

% Simulation loop to integrate the actual state (Model, Controller, Time, Desired_Condition, Robot_States)
Results = RBT_TWMR_GLB_Simulation(Robot_Dynamic_Model, MBC, Time_Struct, d_c, RBT_States);

% Report Results and Plots
RBT_TWMR_GLB_Report(Results, d_c, Time_Struct);

load Best_Results.mat ANN_W_5
figure;
plot(ANN_W_5)
ANN_W_5

