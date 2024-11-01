% Auxiliary function to check constraints
function check_constraints(t, x, u, p)
    % Verify state bounds
    disp('State Bounds Violations:');
    if any(x(1,:) > 1000 | x(1,:) < 0)
        disp('sx violation');
    end
    if any(x(2,:) > 110 | x(2,:) < -10)
        disp('sy violation');
    end
    if any(x(3,:) > 35 | x(3,:) < 0)
        disp('vx violation');
    end
    if any(x(4,:) > 6 | x(4,:) < -5)
        disp('vy violation');
    end
    
    % Verify control bounds
    disp('Control Bounds Violations:');
    if any(u(1,:) > 1800 | u(1,:) < 0)
        disp('Tx violation');
    end
    if any(u(2,:) > 2600 | u(2,:) < 0)
        disp('Ty violation');
    end
    
    % Check physical consistency
    V = sqrt(x(3,:).^2 + x(4,:).^2);
    gamma = atan2(x(4,:), x(3,:));
    disp('Physical Consistency:');
    disp(['Max velocity: ' num2str(max(V))]);
    disp(['Max flight path angle: ' num2str(max(abs(gamma))*180/pi) ' deg']);
end
