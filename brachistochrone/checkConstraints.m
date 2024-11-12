function violations = checkConstraints(time, state, control, params)
    violations = struct();
    violations.state = struct();
    violations.state.x = struct();
    violations.state.y = struct();
    violations.state.v = struct();
    violations.control = struct();
    violations.control.theta = struct();
    violations.energy = struct();
    violations.velocity = struct();
    
    % Check state bounds
    if any(state(1,:) < 0) || any(state(1,:) > 10)
        violations.state.x = struct(...
            'max', max(state(1,:)), ...
            'min', min(state(1,:)), ...
            'bounds', [0, 10]);
    end
    
    if any(state(2,:) < -10) || any(state(2,:) > 0)
        violations.state.y = struct(...
            'max', max(state(2,:)), ...
            'min', min(state(2,:)), ...
            'bounds', [-10, 0]);
    end
    
    if any(state(3,:) < 0) || any(state(3,:) > 20)
        violations.state.v = struct(...
            'max', max(state(3,:)), ...
            'min', min(state(3,:)), ...
            'bounds', [0, 20]);
    end
    
    % Check control bounds
    if any(control < -pi/2) || any(control > pi/2)
        violations.control.theta = struct(...
            'max', max(control), ...
            'min', min(control), ...
            'bounds', [-pi/2, pi/2]);
    end
    
    % Check energy conservation
    E = 0.5 * state(3,:).^2 + params.GRAVITY * state(2,:);
    dE = gradient(E, time);
    
    if any(abs(dE) > 1e-6)
        violations.energy = struct(...
            'maxChange', max(abs(dE)), ...
            'meanChange', mean(abs(dE)));
    end
    
    % Check velocity consistency
    vFromEnergy = sqrt(2 * params.GRAVITY * (state(2,1) - state(2,:)));
    velocityError = abs(state(3,:) - vFromEnergy);
    
    if any(velocityError > 1e-6)
        violations.velocity = struct(...
            'maxError', max(velocityError), ...
            'meanError', mean(velocityError));
    end
    
    % Debug: print violations if any
    % if ~isempty(fieldnames(violations))
    %     disp('Constraint Violations Found:');
    %     if ~isempty(fieldnames(violations.energy))
    %         disp('Energy:');
    %         disp(violations.energy);
    %     end
    %     if ~isempty(fieldnames(violations.velocity))
    %         disp('Velocity:');
    %         disp(violations.velocity);
    %     end
    %     if ~isempty(fieldnames(violations.state))
    %         disp('State:');
    %         if ~isempty(fieldnames(violations.state.x))
    %             disp('sx:');
    %             disp(violations.state.x);
    %         end
    %         if ~isempty(fieldnames(violations.state.y))
    %             disp('sy:');
    %             disp(violations.state.y);
    %         end
    %         if ~isempty(fieldnames(violations.state.v))
    %             disp('v:');
    %             disp(violations.state.v);
    %         end
    %     end
    %     if ~isempty(fieldnames(violations.control))
    %         disp('Control:');
    %         if ~isempty(fieldnames(violations.control.theta))
    %             disp('theta:');
    %             disp(violations.control.theta);
    %         end
    %     end
    % end
end 