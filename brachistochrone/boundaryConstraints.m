function [c, ceq] = boundaryConstraints(x0, xF, t0, tF, boundaryConditions)
    arguments
        x0 double
        xF double
        t0 double
        tF double
        boundaryConditions struct
    end
    
    % Initial conditions
    ceq = [
        x0(1) - boundaryConditions.x0(1);  % Initial x position
        x0(2) - boundaryConditions.x0(2);  % Initial y position
        x0(3) - boundaryConditions.x0(3);  % Initial velocity
        xF(1) - boundaryConditions.xF(1);  % Final x position
        xF(2) - boundaryConditions.xF(2);  % Final y position
    ];

    % Debug output
    % disp('Current boundary constraint values:');
    % disp(['Initial x: ', num2str(ceq(1))]);
    % disp(['Initial y: ', num2str(ceq(2))]);
    % disp(['Initial v: ', num2str(ceq(3))]);
    % disp(['Final x: ', num2str(ceq(4))]);
    % disp(['Final y: ', num2str(ceq(5))]);
    
    % No inequality constraints
    c = [];
end 