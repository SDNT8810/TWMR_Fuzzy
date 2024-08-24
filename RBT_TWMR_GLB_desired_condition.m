function D_C = RBT_TWMR_GLB_desired_condition(Path_Type, Path_Size, Time)

    switch Path_Type
        case 'circle'
            disp(['desired path is a circle with R = ' num2str(Path_Size) 'm'])
            disp(' ')
        
            % Define the desired path with time-varying components
            Path_Params.Xd = Path_Size * sin(2 * pi * Time.t);       % Desired X positions (linear motion along X axis)
            Path_Params.Yd = -Path_Size * cos(2 * pi * Time.t);      % Desired Y positions (sinusoidal motion along Y axis)
            Path_Params.THETAd = 2 * pi * Time.t;            % Desired orientation (sinusoidal angle change)
            
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
        
        case 'spiral'
            disp(['desired path is a spiral with R(t = 1s) = ' num2str(Path_Size) 'm'])
            disp(' ')
        
            % Define the desired path with time-varying components
            Path_Params.Xd = Path_Size * sin(2 * pi * Time.t) .* Time.t;       % Desired X positions (linear motion along X axis)
            Path_Params.Yd = -Path_Size * cos(2 * pi * Time.t) .* Time.t;      % Desired Y positions (sinusoidal motion along Y axis)
            Path_Params.THETAd = 2 * pi * Time.t;            % Desired orientation (sinusoidal angle change)
            
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

        case 'line'
            disp(['desired path is a line with b = ' num2str(Path_Size.b) ' and theta = ' num2str(Path_Size.theta)])
            disp(' ')   

        case 'sin'
            disp(['desired path is sin with A = ' num2str(Path_Size)])
            disp(' ')   
            
        otherwise
            disp('undefined path type')
            disp(' ') 
    end

D_C = Path_Params;
end