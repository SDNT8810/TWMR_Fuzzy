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

end
