
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
    RS.X(:,1) = [Robot_Dynamic_Model.V;Robot_Dynamic_Model.omega];
    RS.E(:,1) = [Robot_Dynamic_Model.V(1);Robot_Dynamic_Model.omega(1)]-[d_c.Vd(1);d_c.OMEGA_d(1)];
    RS.xrbt(1) = Robot_Dynamic_Model.xrbt;
    RS.yrbt(1) = Robot_Dynamic_Model.yrbt;
    RS.Theta(1) = Robot_Dynamic_Model.Theta;
        
    RBT_States = RS;
end