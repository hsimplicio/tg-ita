function params = evtolParams()
    % Returns the EVTOL problem parameters
    
    % Environment parameters
    params.environment.GRAVITY = 9.81;          % Gravitational acceleration [m/s^2]
    params.environment.AIR_DENSITY = 1.2;       % Air density [kg/m^3]
    
    % Aircraft parameters
    params.aircraft.mass = 240;                 % Vehicle mass [kg]
    params.aircraft.rotorArea = 1.6;            % Rotor disk area [m^2]
    params.aircraft.chi = 1.0;                  % Rotor-to-rotor interference factor [-]
    
    params.aircraft.fuselage.CD = 1.0;         % Fuselage drag coefficient [-]
    params.aircraft.fuselage.Sf = 2.11;        % Fuselage cross-sectional area [m^2]
    params.aircraft.fuselage.St = 1.47;        % Fuselage top area [m^2]
    
    params.aircraft.wing.S = 4.0;              % Wing area [m^2]
    params.aircraft.wing.e = 0.9;              % Wing span efficiency factor [-]
    params.aircraft.wing.CDp = 0.0437;         % Wing parasite drag coefficient [-]
    params.aircraft.wing.AR = 20.0;            % Wing aspect ratio [-]
    params.aircraft.wing.CL0 = 0.28;           % Wing lift coefficient at zero AoA [-]
    params.aircraft.wing.CLalpha = 4.00;       % Wing lift curve slope [rad^-1]
    params.aircraft.wing.alphaFus = 0 * (pi / 180);  % Wing mounting angle [rad]
    params.aircraft.wing.sigmoid.M = 4.0;      % Sigmoid steepness parameter [-]
    params.aircraft.wing.sigmoid.alpha0 = 20 * (pi / 180);  % Stall angle [rad]

end 