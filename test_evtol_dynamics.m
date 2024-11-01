% Test function to verify dynamics implementation
function test_evtol_dynamics()
    % Setup test parameters matching your current configuration
    p.environment.g = 9.78;
    p.environment.rho = 1.2;
    p.aircraft.m = 240.0;
    p.aircraft.A = 1.6;
    p.aircraft.chi = 1.0;
    p.aircraft.fuselage.CD = 1.0;
    p.aircraft.fuselage.Sf = 2.11;
    p.aircraft.fuselage.St = 1.47;
    p.aircraft.wing.S = 4.0;
    p.aircraft.wing.e = 0.9; 
    p.aircraft.wing.CD_p = 0.0437;
    p.aircraft.wing.AR = 20.0;
    p.aircraft.wing.CL_zero = 0.28;
    p.aircraft.wing.CL_alpha = 4.00;
    p.aircraft.wing.alpha_fus = 0 * (pi / 180);
    p.aircraft.wing.sigmoid.M = 4.0;
    p.aircraft.wing.sigmoid.alpha_0 = 20 * (pi / 180);

    % Test Case 1: Hover condition
    x_hover = [0; 0; 0; 0; 0];
    u_hover = [0; p.aircraft.m * p.environment.g];
    disp('Hover Test:');
    dx_hover = dyn_evtol(x_hover, u_hover, p);
    disp(['    Vertical acceleration: ' num2str(dx_hover(4))]);
    disp(['    Power consumption: ' num2str(dx_hover(5))]);
    % Debug output
    disp('Hover calculations:');
    disp(['    Total thrust: ' num2str(u_hover(2)) ' N']);
    disp(['    Thrust per rotor: ' num2str(u_hover(2)/4) ' N']);
    disp(['    Hover induced velocity: ' num2str(sqrt(u_hover(2)/(8*p.environment.rho*p.aircraft.A))) ' m/s']);
    disp(['    Total power (computed): ' num2str(dx_hover(5)) ' W']);

    disp(' ');
    
    % Test Case 2: Forward flight
    x_forward = [0; 0; 25; 0; 0];
    % Calculate required thrust for steady forward flight
    [L, D] = compute_lift_drag(x_forward(3), x_forward(4), p);
    u_forward = [D; p.aircraft.m * p.environment.g - L];
    disp('Forward Flight Test:');
    dx_forward = dyn_evtol(x_forward, u_forward, p);
    disp(['    Lift force: ' num2str(L) ' N']);
    disp(['    Drag force: ' num2str(D) ' N']); 
    disp(['    Required thrust - horizontal: ' num2str(u_forward(1)) ' N']);
    disp(['    Required thrust - vertical: ' num2str(u_forward(2)) ' N']);
    disp(['    Horizontal acceleration: ' num2str(dx_forward(3))]);
    disp(['    Vertical acceleration: ' num2str(dx_forward(4))]);
    
    disp(' ');
    
    % Test Case 3: Climbing flight
    x_climb = [0; 0; 20; 5; 0];
    gamma = atan2(5, 20);
    [L, D] = compute_lift_drag(x_climb(3), x_climb(4), p);
    F_req = [D*cos(gamma) + L*sin(gamma); 
    D*sin(gamma) - L*cos(gamma) + p.aircraft.m * p.environment.g];
    u_climb = F_req;
    disp('Climbing Flight Test:');
    dx_climb = dyn_evtol(x_climb, u_climb, p);
    disp(['    Lift force: ' num2str(L) ' N']);
    disp(['    Drag force: ' num2str(D) ' N']);
    disp(['    Required thrust - horizontal: ' num2str(u_climb(1)) ' N']);
    disp(['    Required thrust - vertical: ' num2str(u_climb(2)) ' N']);
    disp(['    Horizontal acceleration: ' num2str(dx_climb(3))]);
    disp(['    Vertical acceleration: ' num2str(dx_climb(4))]);
    
    disp(' ');
    
    % Verify energy model consistency
    % For hover: P = T*v_induced
    T_hover = u_hover(2)/4;  % Per rotor
    vh_hover = sqrt(T_hover/(2*p.environment.rho*p.aircraft.A));
    P_hover_theoretical = 4*T_hover*vh_hover*(1 + p.aircraft.chi);
    disp('Energy Model Test:');
    disp(['    Hover power (computed): ' num2str(dx_hover(5)) ' W']);
    disp(['    Hover power (theoretical): ' num2str(P_hover_theoretical) ' W']);
end
