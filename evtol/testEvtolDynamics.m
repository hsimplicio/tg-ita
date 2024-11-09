% Test function to verify dynamics implementation
function testEvtolDynamics()
    % Setup test parameters matching your current configuration
    params = evtolParams();
    GRAVITY = params.environment.GRAVITY;
    AIR_DENSITY = params.environment.AIR_DENSITY;
    mass = params.aircraft.mass;
    rotorArea = params.aircraft.rotorArea;
    chi = params.aircraft.chi;

    % Test Case 1: Hover condition
    hoverState = [0; 0; 0; 0; 0];
    hoverControl = [0; mass * GRAVITY];
    disp('Hover Test:');
    dxHover = evtolDynamics(hoverState, hoverControl, params);
    disp(['    Vertical acceleration: ' num2str(dxHover(4))]);
    disp(['    Power consumption: ' num2str(dxHover(5))]);
    % Debug output
    disp('Hover calculations:');
    disp(['    Total thrust: ' num2str(hoverControl(2)) ' N']);
    disp(['    Thrust per rotor: ' num2str(hoverControl(2)/4) ' N']);
    disp(['    Hover induced velocity: ' num2str(sqrt(hoverControl(2)/(8*AIR_DENSITY*rotorArea))) ' m/s']);
    disp(['    Total power (computed): ' num2str(dxHover(5)) ' W']);

    disp(' ');
    
    % Test Case 2: Forward flight
    xForward = [0; 0; 25; 0; 0];
    % Calculate required thrust for steady forward flight
    [lift, drag] = computeLiftDrag(xForward(3), xForward(4), params);
    uForward = [drag; mass * GRAVITY - lift];
    disp('Forward Flight Test:');
    dxForward = evtolDynamics(xForward, uForward, params);
    disp(['    Lift force: ' num2str(lift) ' N']);
    disp(['    Drag force: ' num2str(drag) ' N']); 
    disp(['    Required thrust - horizontal: ' num2str(uForward(1)) ' N']);
    disp(['    Required thrust - vertical: ' num2str(uForward(2)) ' N']);
    disp(['    Horizontal acceleration: ' num2str(dxForward(3))]);
    disp(['    Vertical acceleration: ' num2str(dxForward(4))]);
    
    disp(' ');
    
    % Test Case 3: Climbing flight
    xClimb = [0; 0; 20; 5; 0];
    gamma = atan2(5, 20);
    [lift, drag] = computeLiftDrag(xClimb(3), xClimb(4), params);
    requiredThrust = [drag*cos(gamma) + lift*sin(gamma); 
                     drag*sin(gamma) - lift*cos(gamma) + mass * GRAVITY];
    climbControl = requiredThrust;
    disp('Climbing Flight Test:');
    dxClimb = evtolDynamics(xClimb, climbControl, params);
    disp(['    Lift force: ' num2str(lift) ' N']);
    disp(['    Drag force: ' num2str(drag) ' N']);
    disp(['    Required thrust - horizontal: ' num2str(climbControl(1)) ' N']);
    disp(['    Required thrust - vertical: ' num2str(climbControl(2)) ' N']);
    disp(['    Horizontal acceleration: ' num2str(dxClimb(3))]);
    disp(['    Vertical acceleration: ' num2str(dxClimb(4))]);
    
    disp(' ');
    
    % Verify energy model consistency
    % For hover: P = T*v_induced
    hoverThrust = hoverControl(2)/4;  % Per rotor
    hoverInducedVel = sqrt(hoverThrust/(2*AIR_DENSITY*rotorArea));
    hoverPowerTheoretical = 4*hoverThrust*hoverInducedVel*(1 + chi);
    disp('Energy Model Test:');
    disp(['    Hover power (computed): ' num2str(dxHover(5)) ' W']);
    disp(['    Hover power (theoretical): ' num2str(hoverPowerTheoretical) ' W']);
end
