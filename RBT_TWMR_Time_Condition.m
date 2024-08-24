function Time_Struct = RBT_TWMR_Time_Condition(t_zero,dt_sim,t_Max)
    
    T.t_zero = t_zero;
    T.dt_sim = dt_sim;
    T.t_Max = t_Max;
    T.t_span = [t_zero t_Max];  % from 0 to t_Max seconds
    T.t = 0 : dt_sim : t_Max;   % from 0 to t_Max seconds with a step of dt_sim seconds
    Time_Struct = T;

end